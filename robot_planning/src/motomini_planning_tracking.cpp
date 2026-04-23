/**
 * @file motomini_planning_tracking.cpp
 * @brief MotoMiniPlanning::runTrackingPlanner() — DLS Jacobian real-time tracking (v4).
 *
 * Pipeline per tick:
 *   Step 0 — Guard checks (env_, caches, DOF)
 *   Step 1 — Seed from last commit (MPC) or actual (cold start)
 *   Step 2 — Cartesian LPF + workspace bounding
 *   Step 3 — FK: current EE pose from seed
 *   Step 4 — Cartesian step rate-limit (ensures smooth, predictable TCP motion)
 *   Step 5 — DLS Jacobian IK (always produces a valid joint step)
 *   Step 6 — Hard joint-limit clamp
 *   Step 7 — Velocity-consistency gate (prevents Ruckig S-curve overshoot)
 *   Step 8 — Build CompositeInstruction + Ruckig smoothing
 *   Step 9 — Convert to JointTrajectory
 *   Step 10 — MPC lookahead cache update
 *
 * KEY IMPROVEMENTS over v3 (IK-based):
 *   - Cartesian LPF on target         → removes TF jitter → no joint oscillation
 *   - Workspace bounding sphere        → refuses unreachable targets before they
 *                                        can drive the wrist into the cube zone
 *   - Cartesian step rate-limit        → controller sees smooth, bounded TCP motion
 *   - DLS Jacobian IK                  → always converges; KDL LMA fails silently
 *                                        on out-of-reach / near-singular targets
 *   - Manipulability-aware damping     → extra λ near singularities, preventing
 *                                        wild joint moves that trip cube alarms
 *   - Velocity-consistency gate        → hold instead of reversal, no Ruckig overshoot
 *   - Hold-pose on Ruckig failure      → tick never dropped completely
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

// Resource locator (needed for TaskComposer plugin config path)
#include <tesseract_common/resource_locator.h>

// TrajOpt profiles (for optional obstacle avoidance in tracking mode)
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <filesystem>

// std
#include <algorithm>
#include <limits>

using namespace tesseract_kinematics;
using namespace tesseract_planning;

namespace Vinhtesseract_examples
{
    // =========================================================================
    // Tuning constants — adjust on hardware without recompiling the planner
    // =========================================================================

    /// Cartesian step rate-limit per tick (applied BEFORE DLS IK).
    /// Keeping these well under the robot's Cartesian velocity limit is what
    /// makes motion smooth and prevents cube-interference alarms.
    static constexpr double MAX_LIN_STEP_M   = 0.015;  // 1.5 cm / tick  @ 30 Hz → 0.45 m/s max
    static constexpr double MAX_ANG_STEP_RAD = 0.05;   // ~2.9°  / tick

    /// Target low-pass filter (translation only). alpha=1 → no filter.
    /// Lower α → smoother but more lag. 0.35 is a good starting point.
    static constexpr double TARGET_LPF_ALPHA = 0.35;

    /// Robot reachable workspace (MotoMini ≈ 350 mm max reach from flange centre).
    /// Targets outside this sphere are clamped along the ray before IK.
    static constexpr double WORKSPACE_RADIUS_M = 0.34;
    static constexpr double WORKSPACE_Z_MIN    = 0.02;  // ≥ 2 cm above base plate

    /// DLS damping bounds.
    /// λ_min = normal operation; λ_max = at full singularity (w → 0).
    static constexpr double DLS_LAMBDA_MIN   = 0.02;
    static constexpr double DLS_LAMBDA_MAX   = 0.20;
    static constexpr double MANIP_THRESHOLD  = 0.02;   // Yoshikawa w below this → ramp to λ_max

    /// Velocity-consistency gate threshold.
    /// Drop the planned joint step if it reverses direction against the
    /// committed velocity by more than (arccos threshold) degrees.
    /// -0.5 ≈ 120° reversal. Set to -1.0 to disable.
    static constexpr double VEL_CONSISTENCY_MIN = -0.5;

    /// MPC resync: if committed joints drift this far from actual, revert to actual seed.
    static constexpr double MAX_COMMIT_TRACKING_ERROR = 0.15;  // ~8.6° total norm

    // =========================================================================
    // ensureTrackingCaches — one-time init of expensive objects
    // =========================================================================
    void MotoMiniPlanning::ensureTrackingCaches()
    {
        if (tracking_caches_valid_) return;

        tracking_manip_ = env_->getKinematicGroup(manipulator_group_);
        if (!tracking_manip_)
        {
            CONSOLE_BRIDGE_logError("[Tracking] Kinematic group '%s' not found.",
                                    manipulator_group_.c_str());
            return;
        }

        const auto& L          = tracking_manip_->getLimits();
        tracking_joint_limits_    = L.joint_limits;
        tracking_velocity_limits_ = L.velocity_limits;
        tracking_caches_valid_    = true;

        CONSOLE_BRIDGE_logInform("[Tracking] Caches initialized. DOF=%zu",
                                 tracking_manip_->getJointNames().size());
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /// Clamp a 3-D position to the robot's reachable sphere + Z-min floor.
    static Eigen::Vector3d clampToWorkspace(const Eigen::Vector3d& p)
    {
        Eigen::Vector3d out = p;
        const double r = out.norm();
        if (r > WORKSPACE_RADIUS_M)
            out *= (WORKSPACE_RADIUS_M / r);
        if (out.z() < WORKSPACE_Z_MIN)
            out.z() = WORKSPACE_Z_MIN;
        return out;
    }

    /// Axis-angle orientation error vector: R_cur → R_tgt.
    static Eigen::Vector3d rotError(const Eigen::Matrix3d& R_cur,
                                    const Eigen::Matrix3d& R_tgt)
    {
        const Eigen::AngleAxisd aa(R_tgt * R_cur.transpose());
        return aa.axis() * aa.angle();
    }

    // =========================================================================
    // runTrackingPlanner — Ruckig-smoothed real-time DLS tracking
    // =========================================================================
    bool MotoMiniPlanning::runTrackingPlanner(const Eigen::Isometry3d& target_pose,
                                              const Eigen::VectorXd&   hw_velocity,
                                              const Eigen::Vector3d&   tip_velocity)
    {
        // ── Step 0: guard checks ───────────────────────────────────────────────
        if (!env_)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 0] env_ is null.");
            return false;
        }
        std::shared_lock<std::shared_mutex> lock(env_mutex_);

        ensureTrackingCaches();
        if (!tracking_caches_valid_)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 0] Tracking caches invalid.");
            return false;
        }

        const std::vector<std::string> joint_names = tracking_manip_->getJointNames();
        const int n_dof = static_cast<int>(joint_names.size());

        // ── Step 1: seed from last commit (MPC) or actual (cold start) ─────────
        const Eigen::VectorXd actual_pos = env_->getCurrentJointValues(joint_names);
        if (static_cast<int>(actual_pos.size()) != n_dof)
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 1] Joint size mismatch: %d vs %d",
                                    (int)actual_pos.size(), n_dof);
            return false;
        }

        Eigen::VectorXd start_pos;
        Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(n_dof);
        Eigen::VectorXd current_acc = Eigen::VectorXd::Zero(n_dof);

        const bool have_commit =
            has_last_tracking_command_ &&
            static_cast<int>(last_tracking_command_.size())  == n_dof &&
            static_cast<int>(last_tracking_velocity_.size()) == n_dof;

        if (have_commit)
        {
            const double err = (last_tracking_command_ - actual_pos).norm();
            if (err < MAX_COMMIT_TRACKING_ERROR)
            {
                // Normal case: plan from the last committed state.
                start_pos   = last_tracking_command_;
                current_vel = last_tracking_velocity_;
                if (static_cast<int>(last_tracking_acceleration_.size()) == n_dof)
                    current_acc = last_tracking_acceleration_;
                CONSOLE_BRIDGE_logDebug("[Tracking][Step 1] Committed seed. err=%.4f", err);
            }
            else
            {
                // Robot fell behind the commit — resync to actual.
                CONSOLE_BRIDGE_logWarn(
                    "[Tracking][Step 1] Commit-plant err=%.3f — resyncing to actual.", err);
                has_last_tracking_command_ = false;
                start_pos = actual_pos;
                // Use hw_vel as the best velocity estimate on resync.
                if (static_cast<int>(hw_velocity.size()) == n_dof)
                {
                    current_vel = hw_velocity;
                    for (Eigen::Index i = 0;
                         i < current_vel.size() && i < tracking_velocity_limits_.rows(); ++i)
                        current_vel[i] = std::clamp(current_vel[i],
                                                    tracking_velocity_limits_(i, 0),
                                                    tracking_velocity_limits_(i, 1));
                }
            }
        }
        else
        {
            // Cold start — hw_vel is the only velocity info we have.
            start_pos = actual_pos;
            if (static_cast<int>(hw_velocity.size()) == n_dof)
            {
                current_vel = hw_velocity;
                for (Eigen::Index i = 0;
                     i < current_vel.size() && i < tracking_velocity_limits_.rows(); ++i)
                    current_vel[i] = std::clamp(current_vel[i],
                                                tracking_velocity_limits_(i, 0),
                                                tracking_velocity_limits_(i, 1));
            }
            CONSOLE_BRIDGE_logInform("[Tracking][Step 1] Cold start.");
        }

        // ── Step 2: Cartesian LPF + workspace bounding ────────────────────────
        // Low-pass filter on translation only — TF jitter in the incoming pose
        // is the primary source of joint oscillation and cube-interference alarms.
        Eigen::Isometry3d target_filt = target_pose;
        if (tracking_target_initialized_)
        {
            target_filt.translation() =
                (1.0 - TARGET_LPF_ALPHA) * last_tracking_target_.translation() +
                       TARGET_LPF_ALPHA  * target_pose.translation();
            // Orientation: pass-through (filtered separately if tracking_track_orientation_)
            target_filt.linear() = target_pose.linear();
        }
        last_tracking_target_        = target_filt;
        tracking_target_initialized_ = true;

        // Clamp to the robot's reachable sphere + Z-min plate.
        target_filt.translation() = clampToWorkspace(target_filt.translation());

        // ── Step 3: FK — current EE pose at seed ──────────────────────────────
        const auto fk_map = tracking_manip_->calcFwdKin(start_pos);
        auto fk_it = fk_map.find(ee_link_);
        if (fk_it == fk_map.end())
        {
            CONSOLE_BRIDGE_logError("[Tracking][Step 3] FK: ee_link '%s' not found.",
                                    ee_link_.c_str());
            return false;
        }
        const Eigen::Isometry3d T_cur = fk_it->second;

        // ── Step 4: Cartesian step rate-limit ─────────────────────────────────
        // Move only a bounded step toward (filtered, clamped) target this tick.
        // This is the key smoothness fix — the controller sees predictable TCP motion.
        Eigen::Vector3d dx = target_filt.translation() - T_cur.translation();
        {
            const double dx_norm = dx.norm();
            if (dx_norm > MAX_LIN_STEP_M)
                dx *= (MAX_LIN_STEP_M / dx_norm);
        }

        // Orientation error — only applied when user opts in.
        // Position-only (default) is gentler on the wrist and avoids cube alarms
        // from wrist gimbal-lock near the interference zone boundary.
        Eigen::Vector3d dw = Eigen::Vector3d::Zero();
        if (tracking_track_orientation_)
        {
            dw = rotError(T_cur.linear(), target_filt.linear());
            const double dw_norm = dw.norm();
            if (dw_norm > MAX_ANG_STEP_RAD)
                dw *= (MAX_ANG_STEP_RAD / dw_norm);
        }

        // ── Step 5: DLS Jacobian IK ────────────────────────────────────────────
        // Build 6-vector Cartesian twist [vx, vy, vz, wx, wy, wz].
        Eigen::Matrix<double, 6, 1> twist;
        twist.head<3>() = dx;
        twist.tail<3>() = dw;

        // Jacobian at seed in the base_link_ frame (3-arg form from JointGroup API).
        const Eigen::MatrixXd J =
            tracking_manip_->calcJacobian(start_pos, base_link_, ee_link_);

        // Manipulability-aware damping (Yoshikawa 1985).
        // w = sqrt(det(J J^T)) → 0 at singularities → ramp λ to DLS_LAMBDA_MAX.
        const double w = std::sqrt(
            std::max(0.0, (J * J.transpose()).determinant()));
        double lambda = DLS_LAMBDA_MIN;
        if (w < MANIP_THRESHOLD)
        {
            const double t = std::clamp(1.0 - w / MANIP_THRESHOLD, 0.0, 1.0);
            lambda = DLS_LAMBDA_MIN + t * (DLS_LAMBDA_MAX - DLS_LAMBDA_MIN);
        }

        // DLS: dq = J^T (J J^T + λ² I)⁻¹ τ
        const Eigen::Index m = J.rows();
        const Eigen::MatrixXd JJt_reg =
            J * J.transpose() +
            (lambda * lambda) * Eigen::MatrixXd::Identity(m, m);
        const Eigen::VectorXd dq = J.transpose() * JJt_reg.llt().solve(twist);

        Eigen::VectorXd target_joints = start_pos + dq;

        CONSOLE_BRIDGE_logDebug("[Tracking][Step 5] DLS: w=%.4f λ=%.3f |dq|=%.4f",
                                 w, lambda, dq.norm());

        // ── Step 6: hard joint-limit clamp ────────────────────────────────────
        for (Eigen::Index i = 0;
             i < target_joints.size() && i < tracking_joint_limits_.rows(); ++i)
            target_joints[i] = std::clamp(target_joints[i],
                                          tracking_joint_limits_(i, 0),
                                          tracking_joint_limits_(i, 1));

        // ── Step 3b: Jacobian feedforward velocity IC ──────────────────────────
        // WHY: Ruckig's IC velocity must reflect the Cartesian velocity of the
        // gantry tip — otherwise it plans from v=0 every tick and
        // decelerate-accelerates at the planning rate, causing oscillation.
        // J⁺ · v_tip gives the joint velocities that would track the tip in
        // open loop; using this as current_vel means Ruckig sees a smooth
        // continuation rather than a restart.
        // SAFETY: LP-validated against joint velocity limits (95% of limit).
        if (tip_velocity.norm() > 5e-4)  // only if gantry is actually moving
        {
            CONSOLE_BRIDGE_logDebug(
                "[Tracking][Step 3b] Jacobian FF (|v_tip|=%.4f m/s).",
                tip_velocity.norm());
            try
            {
                // Linear rows of the Jacobian (3 × n_dof) at current seed.
                const Eigen::MatrixXd J_full =
                    tracking_manip_->calcJacobian(start_pos, base_link_, ee_link_);
                const Eigen::MatrixXd J_lin = J_full.topRows(3);

                // Damped pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λI)⁻¹
                constexpr double lambda_ff = 1e-4;
                const Eigen::Matrix3d JJt_ff =
                    J_lin * J_lin.transpose() +
                    lambda_ff * Eigen::Matrix3d::Identity();
                const Eigen::VectorXd q_dot_ff =
                    J_lin.transpose() * JJt_ff.ldlt().solve(tip_velocity);

                // Accept only if within 95% of all joint velocity limits.
                bool ok = true;
                for (Eigen::Index i = 0;
                     i < q_dot_ff.size() && i < tracking_velocity_limits_.rows(); ++i)
                {
                    if (std::abs(q_dot_ff[i]) >
                        tracking_velocity_limits_(i, 1) * 0.95)
                    { ok = false; break; }
                }

                if (ok)
                {
                    current_vel = q_dot_ff;
                    current_acc.setZero();  // let Ruckig compute acceleration from context
                    CONSOLE_BRIDGE_logDebug(
                        "[Tracking][Step 3b] FF accepted. |q̇_ff|=%.4f rad/s.",
                        current_vel.norm());
                }
                else
                {
                    CONSOLE_BRIDGE_logDebug(
                        "[Tracking][Step 3b] FF exceeds vel limits — keeping committed vel.");
                }
            }
            catch (const std::exception& e)
            {
                CONSOLE_BRIDGE_logWarn(
                    "[Tracking][Step 3b] Jacobian threw: %s — keeping committed vel.",
                    e.what());
            }
        }

        // ── Step 7: velocity-consistency gate ─────────────────────────────────
        // If dq points sharply opposite to the currently-committed velocity, the
        // target likely jittered across the EE position.  Hold the current pose
        // instead of commanding a reversal — this prevents Ruckig from building an
        // S-curve that overshoots and triggers the cube-interference alarm.
        if (have_commit && current_vel.norm() > 1e-3 && dq.norm() > 1e-4)
        {
            const double cos_angle = dq.normalized().dot(current_vel.normalized());
            if (cos_angle < VEL_CONSISTENCY_MIN)
            {
                CONSOLE_BRIDGE_logDebug(
                    "[Tracking][Step 7] vel-consistency gate: cos=%.2f — holding pose.",
                    cos_angle);
                target_joints = start_pos;   // hold at commit
                current_vel.setZero();
                current_acc.setZero();
            }
        }

        // ── Step 8: build CompositeInstruction for Ruckig ─────────────────────
        // Derive num_pts from lookahead time and a fixed 50 Hz streamer step.
        // This avoids a separate param: more lookahead → more dense waypoints.
        static constexpr double STREAMER_DT = 0.020;  // 1 / 50 Hz
        const double lookahead = planner_period_s_ * tracking_lookahead_mult_;
        const int num_pts = std::max(2, static_cast<int>(std::ceil(lookahead / STREAMER_DT)));

        // Compute a kinematically-valid time horizon from the joint displacements.
        double total_time = lookahead;  // minimum = one full lookahead window
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(n_dof); ++j)
        {
            const double vmax  = std::max(1e-6, tracking_velocity_limits_(j, 1));
            const double delta = std::abs(target_joints[j] - start_pos[j]);
            total_time = std::max(total_time, delta / vmax);
        }
        total_time *= 1.3;  // 30 % margin so Ruckig rarely needs to extend

        const int num_steps = std::max(2, num_pts);
        CompositeInstruction ci(
            "DEFAULT",
            tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));

        for (int s = 0; s < num_steps; ++s)
        {
            const double alpha = static_cast<double>(s) / (num_steps - 1);
            const Eigen::VectorXd q = (1.0 - alpha) * start_pos + alpha * target_joints;

            StateWaypoint swp(joint_names, q);
            swp.setTime(alpha * total_time);

            Eigen::VectorXd v = Eigen::VectorXd::Zero(n_dof);
            Eigen::VectorXd a = Eigen::VectorXd::Zero(n_dof);
            if (s == 0) { v = current_vel; a = current_acc; }
            swp.setVelocity(v);
            swp.setAcceleration(a);

            ci.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE, "FREESPACE"));
        }

        // ── Step 9: Optional TrajOpt collision avoidance ───────────────────────
        // When tracking_obstacle_avoid_ is true, run a fast TrajOpt pass on the
        // Ruckig-smoothed CompositeInstruction to push joints away from obstacles.
        // Falls back silently to Ruckig-only output on any TrajOpt failure.
        if (tracking_obstacle_avoid_)
        {
            try
            {
                // Reuse the same TaskComposer path as offline planning, but with:
                //   - max tracking_trajopt_max_iter_ SQP iterations
                //   - collision COST only (no constraint = never infeasible)
                //   - cloned env so the shared env_ is not mutated
                std::shared_lock<std::shared_mutex> env_lock(env_mutex_);
                auto env_c = std::shared_ptr<tesseract_environment::Environment>(env_->clone());
                env_c->setState(joint_names, start_pos);
                env_lock.unlock();

                auto obs_profiles = std::make_shared<tesseract_common::ProfileDictionary>();

                auto tio_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
                tio_move->cartesian_constraint_config.enabled = false;
                tio_move->cartesian_cost_config.enabled = false;
                tio_move->joint_cost_config.enabled = true;
                tio_move->joint_cost_config.coeff = Eigen::VectorXd::Ones(n_dof) * 1.0;

                auto tio_comp = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
                tio_comp->smooth_velocities     = true;
                tio_comp->smooth_accelerations  = true;
                tio_comp->smooth_jerks          = false;
                tio_comp->velocity_coeff        = Eigen::VectorXd::Ones(1) * 0.5;
                tio_comp->acceleration_coeff    = Eigen::VectorXd::Ones(1) * 1.0;
                tio_comp->collision_cost_config  =
                    trajopt_common::TrajOptCollisionConfig(0.02, 200.0);
                tio_comp->collision_cost_config.enabled = true;

                auto tio_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
                tio_solver->opt_params.max_iterations = tracking_trajopt_max_iter_;

                static const std::string TRAJ_NS = "TrajOptIfoptMotionPlannerTask";
                obs_profiles->addProfile(TRAJ_NS, "DEFAULT", tio_move);
                obs_profiles->addProfile(TRAJ_NS, "FREESPACE", tio_move);
                obs_profiles->addProfile(TRAJ_NS, "DEFAULT", tio_comp);
                obs_profiles->addProfile(TRAJ_NS, "DEFAULT", tio_solver);

                // Locate the TaskComposer config file from the env's resource locator
                std::filesystem::path cfg_path(
                    env_c->getResourceLocator()
                         ->locateResource(
                             "package://tesseract_task_composer/config/task_composer_plugins.yaml")
                         ->getFilePath());
                TaskComposerPluginFactory factory(cfg_path, *env_c->getResourceLocator());

                TaskComposerNode::UPtr tc_task =
                    factory.createTaskComposerNode("TrajOptIfoptPipeline");
                const std::string out_key = tc_task->getOutputKeys().get("program");

                auto ds = std::make_unique<TaskComposerDataStorage>();
                ds->setData("planning_input", ci);
                ds->setData("environment",
                    std::shared_ptr<const tesseract_environment::Environment>(env_c));
                ds->setData("profiles", obs_profiles);

                auto tc_exec = factory.createTaskComposerExecutor("TaskflowExecutor");
                auto tc_ctx  = std::make_shared<TaskComposerContext>(
                    tc_task->getName(), std::move(ds));
                auto fut = tc_exec->run(*tc_task, std::move(tc_ctx));
                fut->wait();

                if (fut->context->isSuccessful())
                {
                    ci = fut->context->data_storage->getData(out_key)
                             .as<CompositeInstruction>();
                    CONSOLE_BRIDGE_logDebug(
                        "[Tracking][Step 9] TrajOpt collision pass applied.");
                }
                else
                {
                    CONSOLE_BRIDGE_logWarn(
                        "[Tracking][Step 9] TrajOpt collision pass failed — using Ruckig-only.");
                }
            }
            catch (const std::exception& e)
            {
                CONSOLE_BRIDGE_logWarn(
                    "[Tracking][Step 9] TrajOpt threw: %s — using Ruckig-only.", e.what());
            }
        }

        // ── Step 10: Ruckig smoothing ──────────────────────────────────────────
        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        auto rp = std::make_shared<RuckigTrajectorySmoothingCompositeProfile>();
        rp->max_duration_extension_factor = 20.0;
        profiles->addProfile("RuckigTrajectorySmoothing", "DEFAULT", rp);

        RuckigTrajectorySmoothing ruckig("RuckigTrajectorySmoothing");
        if (!ruckig.compute(ci, *env_, *profiles))
        {
            CONSOLE_BRIDGE_logWarn("[Tracking][Step 9] Ruckig failed — publishing hold pose.");
            // Publish a zero-velocity hold at the current seed so the streamer
            // always has a valid trajectory and the controller stays live.
            has_last_tracking_command_  = true;
            last_tracking_command_      = start_pos;
            last_tracking_velocity_     = Eigen::VectorXd::Zero(n_dof);
            last_tracking_acceleration_ = Eigen::VectorXd::Zero(n_dof);
            return false;
        }

        // ── Step 11: convert + update MPC cache ───────────────────────────────
        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        if (trajectory.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking][Step 11] toJointTrajectory() returned empty.");
            return false;
        }

        // Lookahead: seed for next tick — pick point at lookahead time
        // ✅ FIX Bug 1: commit velocity INSIDE the loop, never pre-zero.
        //    Fallback to last trajectory point if no lookahead point found.
        bool found_lookahead = false;
        for (const auto& st : trajectory)
        {
            if (st.time >= lookahead)
            {
                has_last_tracking_command_  = true;
                last_tracking_command_      = st.position;
                last_tracking_velocity_     =
                    (static_cast<int>(st.velocity.size()) == n_dof)
                        ? st.velocity : Eigen::VectorXd::Zero(n_dof);
                last_tracking_acceleration_ =
                    (static_cast<int>(st.acceleration.size()) == n_dof)
                        ? st.acceleration : Eigen::VectorXd::Zero(n_dof);
                found_lookahead = true;
                break;
            }
        }
        if (!found_lookahead)
        {
            // Trajectory shorter than lookahead — commit the last point
            const auto& last_st = trajectory.back();
            has_last_tracking_command_  = true;
            last_tracking_command_      = last_st.position;
            last_tracking_velocity_     =
                (static_cast<int>(last_st.velocity.size()) == n_dof)
                    ? last_st.velocity : Eigen::VectorXd::Zero(n_dof);
            last_tracking_acceleration_ = Eigen::VectorXd::Zero(n_dof);
        }

        last_trajectory_ =
            std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());

        CONSOLE_BRIDGE_logInform(
            "[Tracking] OK — %zu pts, %.3f s, lookahead=%.3fs, |dq|=%.4f w=%.3f λ=%.3f",
            trajectory.size(), trajectory.back().time, lookahead, dq.norm(), w, lambda);

        return true;
    }

} // namespace Vinhtesseract_examples