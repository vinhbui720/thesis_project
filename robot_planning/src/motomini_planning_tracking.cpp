/**
 * @file motomini_planning_tracking.cpp
 * @brief MotoMiniPlanning::runTrackingPlanner() — TrajOpt-smoothed tracking with ISP velocities.
 *
 * Pipeline per tick:
 *   1. IK  → target joint position (seeded from last command for branch continuity)
 *   2. Interpolate N waypoints in joint space
 *   3. (Optional) TrajOpt-ifopt optimization — smoothing costs + collision constraints
 *   4. ISP time parameterization → proper velocities/accelerations respecting URDF limits
 *   5. Store trajectory; node publishes it with full velocity data
 *
 * Two modes controlled by tracking_use_trajopt_:
 *   false → steps 1-2-4-5 only (~1 ms per tick, suitable for 50+ Hz)
 *   true  → full pipeline 1-2-3-4-5 (~5-20 ms per tick, suitable for 20-30 Hz)
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

// ISP time parameterization
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract_time_parameterization/core/utils.h>

// TrajOpt-ifopt (lightweight optimization)
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
    }

    // ---------------------------------------------------------------------------
    // runTrackingPlanner — TrajOpt + ISP tracking pipeline
    // ---------------------------------------------------------------------------
    bool MotoMiniPlanning::runTrackingPlanner(const Eigen::Isometry3d &target_pose)
    {
        if (!env_)
            return false;

        CONSOLE_BRIDGE_logDebug("[Tracking] Starting TrajOpt tracking planner...");

        std::shared_lock<std::shared_mutex> lock(env_mutex_);

        // ---- Cache kinematic group + limits (once) ----
        ensureTrackingCaches();
        if (!tracking_caches_valid_)
            return false;

        const std::vector<std::string> joint_names = {
            "joint_1_s", "joint_2_l", "joint_3_u",
            "joint_4_r", "joint_5_b", "joint_6_t"};
        const int n_dof = static_cast<int>(joint_names.size());
        const Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);

        // ---- IK seed: prefer last command for branch continuity ----
        const Eigen::VectorXd ik_seed =
            (has_last_tracking_command_ && last_tracking_command_.size() == start_pos.size())
                ? last_tracking_command_
                : start_pos;

        // ---- FK: keep current orientation (position-only tracking) ----
        const auto fk_map = tracking_manip_->calcFwdKin(start_pos);
        auto fk_it = fk_map.find(ee_link_);
        if (fk_it == fk_map.end())
        {
            CONSOLE_BRIDGE_logError("[Tracking] FK did not contain ee link '%s'", ee_link_.c_str());
            return false;
        }

        Eigen::Isometry3d target_adjusted = target_pose;
        target_adjusted.linear() = fk_it->second.linear();

        // ---- Inverse Kinematics ----
        KinGroupIKInput ik_input(target_adjusted, base_link_, ee_link_);
        IKSolutions solutions = tracking_manip_->calcInvKin(ik_input, ik_seed);
        if (solutions.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] IK failed for XYZ=(%.4f, %.4f, %.4f)",
                                   target_adjusted.translation().x(),
                                   target_adjusted.translation().y(),
                                   target_adjusted.translation().z());
            return false;
        }

        // Pick solution closest to seed
        Eigen::VectorXd best_sol = solutions.front();
        double best_dist = std::numeric_limits<double>::max();
        for (const auto &sol : solutions)
        {
            const double d = (sol - ik_seed).squaredNorm();
            if (d < best_dist)
            {
                best_dist = d;
                best_sol = sol;
            }
        }

        // Clamp to joint position limits
        for (Eigen::Index i = 0; i < best_sol.size() && i < tracking_joint_limits_.rows(); ++i)
            best_sol[i] = std::clamp(best_sol[i],
                                     tracking_joint_limits_(i, 0),
                                     tracking_joint_limits_(i, 1));

        // Per-tick step clamp
        Eigen::VectorXd target_joints = best_sol;
        for (Eigen::Index i = 0; i < target_joints.size(); ++i)
        {
            const double delta = best_sol[i] - start_pos[i];
            target_joints[i] = start_pos[i] +
                               std::clamp(delta, -tracking_max_joint_step_, tracking_max_joint_step_);
        }

        // ================================================================
        //  STEP 2: Interpolate N waypoints in joint space
        // ================================================================
        const int N = tracking_num_steps_;
        std::vector<Eigen::VectorXd> waypoints(N);
        for (int s = 0; s < N; ++s)
        {
            const double alpha = static_cast<double>(s) / (N - 1);
            waypoints[s] = (1.0 - alpha) * start_pos + alpha * target_joints;
        }

        // ================================================================
        //  STEP 3 (optional): TrajOpt-ifopt optimization
        // ================================================================
        if (tracking_use_trajopt_ && N >= 3)
        {
            auto nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

            // --- Variables ---
            std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>> vars;
            vars.reserve(N);
            for (int i = 0; i < N; ++i)
            {
                auto var = std::make_shared<trajopt_ifopt::JointPosition>(
                    waypoints[i], joint_names, "JP_" + std::to_string(i));

                if (i == 0)
                {
                    // Fix start position (tight bounds ±1e-6)
                    Eigen::MatrixX2d fixed_bounds(n_dof, 2);
                    fixed_bounds.col(0) = start_pos.array() - 1e-6;
                    fixed_bounds.col(1) = start_pos.array() + 1e-6;
                    var->SetBounds(fixed_bounds);
                }
                else if (i == N - 1)
                {
                    // Fix end position to IK target (tight bounds ±1e-6)
                    Eigen::MatrixX2d fixed_bounds(n_dof, 2);
                    fixed_bounds.col(0) = target_joints.array() - 1e-6;
                    fixed_bounds.col(1) = target_joints.array() + 1e-6;
                    var->SetBounds(fixed_bounds);
                }
                else
                {
                    // Intermediate points: full joint limits
                    var->SetBounds(tracking_joint_limits_);
                }

                vars.push_back(var);
                nlp->addVariableSet(var);
            }

            // --- Velocity limit constraints (hard bounds) ---
            // Each consecutive waypoint difference must respect max velocity * (time step)
            // Assume uniform time distribution: dt = total_time / (N-1)
            // For tracking, use conservative 0.1s per step
            const double dt_trajopt = 0.1;
            const double dt_inv = 1.0 / dt_trajopt;

            // Velocity = (q_{i+1} - q_i) / dt must be <= velocity_limit
            // Therefore: q_{i+1} - q_i <= velocity_limit * dt
            for (int i = 0; i < N - 1; ++i)
            {
                // Get velocity limits (asymmetric, but we use symmetric bounds)
                Eigen::VectorXd vel_ub = Eigen::VectorXd::Zero(n_dof);
                for (int j = 0; j < n_dof; ++j)
                {
                    double lim_neg = std::abs(tracking_velocity_limits_(j, 0));
                    double lim_pos = tracking_velocity_limits_(j, 1);
                    vel_ub[j] = std::min(lim_neg, lim_pos) * dt_trajopt * 0.8; // 80% safety margin
                }

                // q_{i+1} - q_i <= vel_ub (velocity constraint as difference bound)
                auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(
                    vel_ub, vars, Eigen::VectorXd::Ones(n_dof), "VelBound_" + std::to_string(i));
                nlp->addConstraintSet(vel_constraint);
            }

            // --- Smoothing costs ---
            // Velocity smoothing: minimize (q_{i+1} - q_i)^2
            Eigen::VectorXd vel_coeffs = Eigen::VectorXd::Ones(n_dof) * 0.5;
            auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(
                Eigen::VectorXd::Zero(n_dof), vars, vel_coeffs, "VelSmooth");
            nlp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::SQUARED);

            // Acceleration smoothing: minimize (q_{i+2} - 2*q_{i+1} + q_i)^2
            Eigen::VectorXd accel_coeffs = Eigen::VectorXd::Ones(n_dof) * 2.0;
            auto accel_cost = std::make_shared<trajopt_ifopt::JointAccelConstraint>(
                Eigen::VectorXd::Zero(n_dof), vars, accel_coeffs, "AccelSmooth");
            nlp->addCostSet(accel_cost, trajopt_sqp::CostPenaltyType::SQUARED);

            // --- Collision constraints (optional, expensive) ---
            if (tracking_enable_collision_)
            {
                trajopt_common::TrajOptCollisionConfig col_cfg(0.01, 50.0);
                col_cfg.collision_check_config.type =
                    tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
                col_cfg.collision_margin_buffer = 0.005;

                auto col_cache = std::make_shared<trajopt_ifopt::CollisionCache>(N);
                for (int i = 1; i < N; ++i)
                {
                    auto evaluator =
                        std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
                            col_cache, tracking_manip_, env_, col_cfg, true);
                    auto constraint =
                        std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
                            evaluator, vars[i], col_cfg.max_num_cnt, false,
                            "Col_" + std::to_string(i));
                    nlp->addConstraintSet(constraint);
                }
            }

            // --- Solve (few iterations for speed) ---
            nlp->setup();
            auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
            trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
            solver.params.initial_trust_box_size = 0.05;
            solver.params.min_trust_box_size = 1e-4;
            solver.params.min_approx_improve = 1e-3;
            solver.params.max_iterations = tracking_trajopt_max_iter_;
            solver.init(nlp);
            solver.solve(nlp);

            // Extract optimized positions
            for (int i = 0; i < N; ++i)
                waypoints[i] = vars[i]->GetValues();
        }

        // ================================================================
        //  STEP 4: ISP time parameterization → velocities + accelerations
        // ================================================================
        CompositeInstruction ci(
            "DEFAULT",
            tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));

        for (const auto &wp : waypoints)
        {
            ci.push_back(MoveInstruction(
                StateWaypoint(joint_names, wp),
                MoveInstructionType::FREESPACE, "FREESPACE"));
        }

        tesseract_planning::formatProgram(ci, *env_);

        // ISP profile: use URDF velocity limits (scaled down for tracking safety)
        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        auto isp_profile =
            std::make_shared<IterativeSplineParameterizationCompositeProfile>(
                0.65, // max_velocity_scaling_factor  (65% of URDF limits for safe margin)
                0.5); // max_acceleration_scaling_factor (conservative for tracking)
        profiles->addProfile(
            "IterativeSplineParameterization", "DEFAULT", isp_profile);

        auto isp = std::make_unique<tesseract_planning::IterativeSplineParameterization>(
            "IterativeSplineParameterization");

        if (!isp->compute(ci, *env_, *profiles))
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] ISP failed, using raw trajectory with manual timing");
            // Fallback: uniform timing, no velocity data
            tesseract_common::JointTrajectory raw = toJointTrajectory(ci);
            const double dt = 0.04;
            for (std::size_t i = 0; i < raw.size(); ++i)
                raw[i].time = static_cast<double>(i) * dt;
            last_tracking_command_ = target_joints;
            has_last_tracking_command_ = true;
            last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(raw);
            return true;
        }

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        if (trajectory.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] Empty trajectory from ISP");
            return false;
        }

        // ================================================================
        //  VERIFY & CLAMP: Ensure velocities are within URDF limits
        // ================================================================
        bool has_velocity_data = false;
        for (const auto &state : trajectory)
        {
            if (state.velocity.size() > 0 && state.velocity.size() == n_dof)
            {
                has_velocity_data = true;
                break;
            }
        }

        if (!has_velocity_data)
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] ISP produced trajectory without velocity data!");
            // Fallback: compute velocities numerically from positions
            for (std::size_t i = 1; i < trajectory.size(); ++i)
            {
                const double dt = trajectory[i].time - trajectory[i - 1].time;
                if (dt > 1e-6)
                {
                    // Safely compute and assign velocities
                    const int pos_size = static_cast<int>(trajectory[i].position.size());
                    if (pos_size == n_dof)
                    {
                        Eigen::VectorXd vel = Eigen::VectorXd::Zero(n_dof);
                        for (int j = 0; j < n_dof; ++j)
                        {
                            vel[j] = (trajectory[i].position[j] - trajectory[i - 1].position[j]) / dt;
                        }
                        trajectory[i].velocity = vel;
                    }
                }
            }
            if (!trajectory.empty())
                trajectory[0].velocity = Eigen::VectorXd::Zero(n_dof);
        }

        // Clamp all velocities to URDF limits
        for (auto &state : trajectory)
        {
            // Ensure velocity data exists and is sized correctly
            const int vel_size = static_cast<int>(state.velocity.size());
            if (vel_size != n_dof)
            {
                if (vel_size > 0)
                {
                    // Resize to n_dof with zero-padding
                    Eigen::VectorXd temp = Eigen::VectorXd::Zero(n_dof);
                    const int copy_size = (vel_size < n_dof) ? vel_size : n_dof;
                    for (int i = 0; i < copy_size; ++i)
                        temp[i] = state.velocity[i];
                    state.velocity = temp;
                }
                else
                {
                    state.velocity = Eigen::VectorXd::Zero(n_dof);
                }
            }

            for (int j = 0; j < n_dof; ++j)
            {
                if (j >= static_cast<int>(state.velocity.size()))
                    break;

                const double vel_min = tracking_velocity_limits_(j, 0);
                const double vel_max = tracking_velocity_limits_(j, 1);
                state.velocity[j] = std::clamp(state.velocity[j], vel_min, vel_max);
            }
        }

        // DEBUG SUPPRESSED: Velocity enforcement debug spam removed
        // CONSOLE_BRIDGE_logDebug("[Tracking] Velocity enforcement complete: %zu pts with velocities",
        //                         trajectory.size());

        // ---- Persist state for next tick ----
        last_tracking_command_ = target_joints;
        has_last_tracking_command_ = true;
        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        // ---- Debug visualization ----
        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());

        if (toolpath_cb_)
        {
            std::vector<Eigen::Vector3d> ee_path;
            ee_path.reserve(trajectory.size());
            for (const auto &state : trajectory)
                ee_path.push_back(
                    tracking_manip_->calcFwdKin(state.position).at(ee_link_).translation());
            toolpath_cb_(ee_path);
        }

        // DEBUG SUPPRESSED: Trajectory ready debug spam removed
        // CONSOLE_BRIDGE_logDebug("[Tracking] Trajectory ready: %zu pts, %.3f s horizon",
        //                         trajectory.size(),
        //                         trajectory.empty() ? 0.0 : trajectory.back().time);
        return true;
    }

} // namespace Vinhtesseract_examples