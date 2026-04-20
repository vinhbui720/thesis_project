/**
 * @file motomini_planning_tracking.cpp
 * @brief MotoMiniPlanning::runTrackingPlanner() — Ruckig-smoothed real-time tracking.
 *
 * Pipeline per tick:
 *   Step 0 — Guard checks (env_, caches, DOF validation)
 *   Step 1 — Seed last command / zero-vel initial state
 *   Step 2 — FK to extract current EE orientation (position-only tracking)
 *   Step 3 — IK: compute target joint angles
 *   Step 4 — Per-tick joint-step clamp (safety)
 *   Step 5 — Joint-space linear interpolation (N waypoints)
 *   Step 6 — Build CompositeInstruction with vel/acc set AT CONSTRUCTION TIME
 *            (NEVER call formatProgram after — it copies InstructionPoly objects,
 *             invalidating InstructionsTrajectory's reference_wrappers → SIGSEGV -11)
 *   Step 7 — CI integrity check (positions/velocities/accelerations all correct DOF)
 *   Step 8 — Ruckig smoothing
 *   Step 9 — Convert to JointTrajectory
 *   Step 10 — MPC lookahead cache update
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Kinematics
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/types.h>

// Command language
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

// Environment + state
#include <tesseract_common/joint_state.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_motion_planners/core/utils.h>

// Time parameterization
#include <tesseract_time_parameterization/core/utils.h>
#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>

// TrajOpt-ifopt (lightweight optimization headers — keep for future use)
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

#include <algorithm>
#include <limits>

using namespace tesseract_kinematics;
using namespace tesseract_planning;

namespace Vinhtesseract_examples
{

    // ---------------------------------------------------------------------------
    // ensureTrackingCaches — one-time init of expensive objects
    // ---------------------------------------------------------------------------
    void MotoMiniPlanning::ensureTrackingCaches()
    {
        if (tracking_caches_valid_)
            return;

        tracking_manip_ = env_->getKinematicGroup(manipulator_group_);
        if (!tracking_manip_)
        {
            CONSOLE_BRIDGE_logError("[Tracking] Kinematic group '%s' not found",
                                    manipulator_group_.c_str());
            return;
        }

        const auto &limits = tracking_manip_->getLimits();
        tracking_joint_limits_ = limits.joint_limits;
        tracking_velocity_limits_ = limits.velocity_limits;
        tracking_caches_valid_ = true;
        CONSOLE_BRIDGE_logInform("[Tracking] Caches initialized. DOF=%zu",
                                 tracking_manip_->getJointNames().size());
    }

    // ---------------------------------------------------------------------------
    // runTrackingPlanner — Ruckig-smoothed real-time MPC tracking
    // ---------------------------------------------------------------------------
    bool MotoMiniPlanning::runTrackingPlanner(const Eigen::Isometry3d &target_pose)
    {
        // ========================================================================
        // Step 0 — Guard checks
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 0] runTrackingPlanner entered.");

        if (!env_)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 0] env_ is null!");
            return false;
        }

        std::shared_lock<std::shared_mutex> lock(env_mutex_);
        CONSOLE_BRIDGE_logInform("[Tracking][Step 0] env_ lock acquired.");

        ensureTrackingCaches();
        if (!tracking_caches_valid_)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 0] Tracking caches invalid.");
            return false;
        }

        const std::vector<std::string> joint_names = tracking_manip_->getJointNames();
        const int n_dof = static_cast<int>(joint_names.size());
        CONSOLE_BRIDGE_logInform("[Tracking][Step 0] n_dof=%d", n_dof);

        Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);
        CONSOLE_BRIDGE_logInform("[Tracking][Step 0] start_pos.size()=%d", (int)start_pos.size());

        if (static_cast<int>(start_pos.size()) != n_dof)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 0] start_pos size mismatch: got %d, expected %d",
                                    (int)start_pos.size(), n_dof);
            return false;
        }

        // ========================================================================
        // Step 1 — Seed state (MPC continuity from last output)
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 1] Setting IK seed and velocity boundary.");
        Eigen::VectorXd ik_seed = start_pos;
        Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(n_dof);
        Eigen::VectorXd current_acc = Eigen::VectorXd::Zero(n_dof);

        if (has_last_tracking_command_ &&
            static_cast<int>(last_tracking_command_.size()) == n_dof)
        {
            ik_seed = last_tracking_command_;
            if (static_cast<int>(last_tracking_velocity_.size()) == n_dof)
                current_vel = last_tracking_velocity_;
            if (static_cast<int>(last_tracking_acceleration_.size()) == n_dof)
                current_acc = last_tracking_acceleration_;
            CONSOLE_BRIDGE_logInform("[Tracking][Step 1] Cached seed loaded. |vel|=%.4f", current_vel.norm());
        }
        else
        {
            CONSOLE_BRIDGE_logInform("[Tracking][Step 1] No cache — using start_pos as seed.");
        }

        // ========================================================================
        // Step 2 — FK: get current EE orientation (position-only tracking)
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 2] FK orientation check for ee_link='%s'...", ee_link_.c_str());
        Eigen::Isometry3d target_adjusted = target_pose;
        {
            const auto fk_map = tracking_manip_->calcFwdKin(start_pos);
            auto fk_it = fk_map.find(ee_link_);
            if (fk_it == fk_map.end())
            {
                CONSOLE_BRIDGE_logError("[Tracking][Step 2] FK: ee_link '%s' not found in FK result.",
                                        ee_link_.c_str());
                return false;
            }
            target_adjusted.linear() = fk_it->second.linear();
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 2] FK orientation obtained.");

        // ========================================================================
        // Step 3 — IK: compute target joint angles
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 3] Computing IK (base='%s', ee='%s')...",
                                 base_link_.c_str(), ee_link_.c_str());
        KinGroupIKInput ik_input(target_adjusted, base_link_, ee_link_);
        IKSolutions solutions = tracking_manip_->calcInvKin(ik_input, ik_seed);
        if (solutions.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking][Step 3] IK returned 0 solutions — skipping tick.");
            return false;
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 3] IK: %zu solutions.", solutions.size());

        Eigen::VectorXd best_sol = solutions.front();
        double best_dist = std::numeric_limits<double>::max();
        for (const auto &sol : solutions)
        {
            const double d = (sol - ik_seed).squaredNorm();
            if (d < best_dist) { best_dist = d; best_sol = sol; }
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 3] Best solution dist=%.4f", std::sqrt(best_dist));

        // Clamp to joint limits
        for (Eigen::Index i = 0; i < best_sol.size() && i < tracking_joint_limits_.rows(); ++i)
            best_sol[i] = std::clamp(best_sol[i],
                                     tracking_joint_limits_(i, 0),
                                     tracking_joint_limits_(i, 1));

        // ========================================================================
        // Step 4 — Per-tick joint-step clamp (safety)
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 4] Applying max_joint_step=%.3f clamp.",
                                 tracking_max_joint_step_);
        Eigen::VectorXd target_joints = best_sol;
        for (Eigen::Index i = 0; i < target_joints.size(); ++i)
        {
            const double delta = best_sol[i] - start_pos[i];
            target_joints[i] = start_pos[i] +
                               std::clamp(delta, -tracking_max_joint_step_, tracking_max_joint_step_);
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 4] target_joints[0]=%.4f start_pos[0]=%.4f",
                                 target_joints[0], start_pos[0]);

        // ========================================================================
        // Step 5 — Joint-space linear interpolation
        // ========================================================================
        int num_steps = std::max(2, tracking_num_steps_);
        CONSOLE_BRIDGE_logInform("[Tracking][Step 5] Interpolating %d waypoints.", num_steps);

        std::vector<Eigen::VectorXd> waypoints(num_steps);
        for (int s = 0; s < num_steps; ++s)
        {
            const double alpha = static_cast<double>(s) / (num_steps - 1);
            waypoints[s] = (1.0 - alpha) * start_pos + alpha * target_joints;
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 5] First wpt[0]=%.4f  Last wpt[0]=%.4f",
                                 waypoints.front()[0], waypoints.back()[0]);

        // ========================================================================
        // Step 6 — Build CompositeInstruction
        //
        // ROOT CAUSE OF RUCKIG INFINITE-LOOP / SIGSEGV:
        //   If StateWaypoints have no timestamps (swp.setTime() never called),
        //   all getTimeFromStart(i) return 0. Ruckig then computes:
        //     timestep = sum(zeros) / (n - 1) = 0
        //   and passes timestep=0 to its internal ODE solver, causing division by zero,
        //   NaN propagation, and an infinite duration-extension while-loop that eventually
        //   corrupts memory → SIGSEGV -11.
        //
        // THE FIX:
        //   Compute a realistic total horizon time from the joint displacements
        //   and velocity limits, then assign linearly-spaced timestamps.
        //
        // ALSO: Never call formatProgram() or modify ci[i] after construction—
        //   this corrupts InstructionsTrajectory's reference_wrappers → crash.
        // ========================================================================

        // --- Compute a kinematically-valid time horizon ---
        double total_time = 0.1; // minimum 100 ms
        const Eigen::MatrixX2d& vel_lim = tracking_velocity_limits_;
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(joint_names.size()); ++j)
        {
            const double max_vel = std::max(1e-6, vel_lim(j, 1));
            const double delta   = std::abs(target_joints[j] - start_pos[j]);
            total_time = std::max(total_time, delta / max_vel);
        }
        total_time *= 1.3; // 30% margin so Ruckig rarely needs to extend
        CONSOLE_BRIDGE_logInform("[Tracking][Step 6] total_time=%.3f s, building CI with %d waypoints.",
                                 total_time, num_steps);

        CompositeInstruction ci(
            "DEFAULT",
            tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));

        for (int s = 0; s < num_steps; ++s)
        {
            StateWaypoint swp(joint_names, waypoints[static_cast<size_t>(s)]);

            // --- Linearly-spaced timestamp (REQUIRED: keeps Ruckig timestep > 0) ---
            const double t = (num_steps == 1) ? total_time
                           : (static_cast<double>(s) / (num_steps - 1)) * total_time;
            swp.setTime(t);

            // --- Boundary conditions: set BEFORE push_back, BEFORE any flatten() ---
            Eigen::VectorXd vel = Eigen::VectorXd::Zero(n_dof);
            Eigen::VectorXd acc = Eigen::VectorXd::Zero(n_dof);
            if (s == 0)
            {
                vel = current_vel;
                acc = current_acc;
            }
            swp.setVelocity(vel);
            swp.setAcceleration(acc);

            ci.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE, "FREESPACE"));
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 6] CI built: %zu elements, t=[0 → %.3f s].",
                                 ci.size(), total_time);

        // ========================================================================
        // Step 7 — CI integrity verification (catch dimension bugs before Ruckig)
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 7] Verifying CI integrity...");
        for (size_t i = 0; i < ci.size(); ++i)
        {
            if (!ci[i].isMoveInstruction())
            {
                CONSOLE_BRIDGE_logError("[Tracking][Step 7] ci[%zu] is not a MoveInstruction!", i);
                return false;
            }
            const auto& mi_ref = ci[i].as<MoveInstructionPoly>();
            if (!mi_ref.getWaypoint().isStateWaypoint())
            {
                CONSOLE_BRIDGE_logError("[Tracking][Step 7] ci[%zu].waypoint is not StateWaypoint!", i);
                return false;
            }
            const auto& swp_ref = mi_ref.getWaypoint().as<StateWaypointPoly>();
            const int pos_sz = static_cast<int>(swp_ref.getPosition().size());
            const int vel_sz = static_cast<int>(swp_ref.getVelocity().size());
            const int acc_sz = static_cast<int>(swp_ref.getAcceleration().size());
            if (pos_sz != n_dof || vel_sz != n_dof || acc_sz != n_dof)
            {
                CONSOLE_BRIDGE_logError(
                    "[Tracking][Step 7] ci[%zu] dim error: pos=%d vel=%d acc=%d expected %d",
                    i, pos_sz, vel_sz, acc_sz, n_dof);
                return false;
            }
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 7] CI verified OK.");

        // ========================================================================
        // Step 8 — Ruckig smoothing
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 8] Setting up Ruckig profile...");
        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        auto ruckig_profile = std::make_shared<RuckigTrajectorySmoothingCompositeProfile>();
        ruckig_profile->max_duration_extension_factor = 20.0;
        profiles->addProfile("RuckigTrajectorySmoothing", "DEFAULT", ruckig_profile);

        RuckigTrajectorySmoothing ruckig("RuckigTrajectorySmoothing");
        CONSOLE_BRIDGE_logInform("[Tracking][Step 8] Calling ruckig.compute()...");
        if (!ruckig.compute(ci, *env_, *profiles))
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 8] Ruckig smoothing FAILED. Dropping command.");
            return false;
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 8] Ruckig compute() SUCCESS.");

        // ========================================================================
        // Step 9 — Convert to JointTrajectory
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 9] Converting CI → JointTrajectory...");
        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        if (trajectory.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking][Step 9] toJointTrajectory() returned empty result.");
            return false;
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 9] Trajectory: %zu pts, %.3f s",
                                 trajectory.size(), trajectory.back().time);

        // ========================================================================
        // Step 10 — MPC lookahead cache update (seed next tick's boundary)
        // ========================================================================
        CONSOLE_BRIDGE_logInform("[Tracking][Step 10] Updating MPC state cache (lookahead=0.05 s)...");
        const double lookahead_time = 0.05;
        has_last_tracking_command_ = true;
        last_tracking_command_ = target_joints;

        // Default: use very first point's vel/acc
        last_tracking_velocity_ = (static_cast<int>(trajectory.front().velocity.size()) == n_dof)
                                      ? trajectory.front().velocity
                                      : Eigen::VectorXd::Zero(n_dof);
        last_tracking_acceleration_ = (static_cast<int>(trajectory.front().acceleration.size()) == n_dof)
                                          ? trajectory.front().acceleration
                                          : Eigen::VectorXd::Zero(n_dof);

        // Advance to the lookahead point for smoother MPC seeding
        for (const auto &state : trajectory)
        {
            if (state.time >= lookahead_time)
            {
                last_tracking_command_ = state.position;
                last_tracking_velocity_ = (static_cast<int>(state.velocity.size()) == n_dof)
                                              ? state.velocity
                                              : Eigen::VectorXd::Zero(n_dof);
                last_tracking_acceleration_ = (static_cast<int>(state.acceleration.size()) == n_dof)
                                                  ? state.acceleration
                                                  : Eigen::VectorXd::Zero(n_dof);
                break;
            }
        }
        CONSOLE_BRIDGE_logInform("[Tracking][Step 10] Cache updated. |cached_vel|=%.4f",
                                 last_tracking_velocity_.norm());

        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        // === Debug visualization ===
        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());

        CONSOLE_BRIDGE_logInform("[Tracking][DONE] %zu pts, %.3f s — SUCCESS.",
                                 trajectory.size(), trajectory.back().time);
        return true;
    }

} // namespace Vinhtesseract_examples