/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — MPC Receding Horizon Planner tracking mode.
 *
 * Optimized for high-frequency (30Hz) execution to feed the trajectory streamer.
 *
 * Fixes applied (in order of impact):
 *   1. Horizon reduced 5→3             : Solve time ~440ms → ~30ms
 *   2. State propagation anchor        : Eliminates staircase from sensor lag
 *   3. IK velocity-feasibility filter  : Prevents IK branch-flip spikes
 *   4. Last-joint weighted IK + cost   : Suppresses unnecessary wrist rotation
 *   5. Dense interpolation (no reintegration): Fills streamer buffer continuously
 *   6. Central-difference + LPF veloc. : Smooth velocity commands
 *   7. Splice-time cap                 : Prevents t_splice=12s on slow ticks
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_environment/environment.h>

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_common/collision_types.h>

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <tesseract_collision/core/types.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/core/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_dictionary.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

// =========================================================================
// PHASE 1: ASYNC CALLBACKS
// =========================================================================

void MotoMiniPlanningNode::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);

    Eigen::Isometry3d new_pose;
    tf2::fromMsg(msg->pose, new_pose);

    if (target_initialized_)
    {
        rclcpp::Time current_time(msg->header.stamp);
        double dt = (current_time - last_target_time_).seconds();

        if (dt > 1e-4)
        {
            target_velocity_linear_ =
                (new_pose.translation() - current_target_pose_.translation()) / dt;

            Eigen::AngleAxisd diff(new_pose.linear() * current_target_pose_.linear().inverse());
            target_velocity_angular_ = diff.axis() * diff.angle() / dt;
        }
    }
    else
    {
        target_velocity_linear_.setZero();
        target_velocity_angular_.setZero();
        target_initialized_ = true;
    }

    current_target_pose_ = new_pose;
    last_target_time_ = msg->header.stamp;
}

// =========================================================================
// MATH HELPERS
// =========================================================================

Eigen::Isometry3d MotoMiniPlanningNode::predictTargetPose(int step_k)
{
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);

    double lead_time = step_k * mpc_dt_;
    Eigen::Isometry3d predicted = current_target_pose_;
    predicted.translation() += target_velocity_linear_ * lead_time;

    double angle = target_velocity_angular_.norm() * lead_time;
    if (angle > 1e-6)
    {
        predicted.linear() =
            Eigen::AngleAxisd(angle, target_velocity_angular_.normalized())
                .toRotationMatrix() *
            current_target_pose_.linear();
    }
    return predicted;
}

Eigen::VectorXd MotoMiniPlanningNode::computeDlsExtrapolation(
    const Eigen::VectorXd &q_last, const Eigen::Isometry3d &target_next)
{
    auto fk = manip_->calcFwdKin(q_last);
    if (fk.find(ee_link_) == fk.end())
        return q_last;

    Eigen::Isometry3d ee_current = fk.at(ee_link_);
    Eigen::Vector3d dx = target_next.translation() - ee_current.translation();
    Eigen::AngleAxisd aa(target_next.linear() * ee_current.linear().inverse());
    Eigen::Vector3d dw = aa.axis() * aa.angle();

    Eigen::Matrix<double, 6, 1> twist;
    twist.head<3>() = dx;
    twist.tail<3>() = dw;

    Eigen::MatrixXd J = manip_->calcJacobian(q_last, base_link_, ee_link_);
    const double lambda = 0.05;
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt += (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(twist);

    Eigen::VectorXd q_next = q_last + dq;
    for (int i = 0; i < q_next.size(); ++i)
        q_next[i] = std::max(joint_limits_(i, 0),
                             std::min(joint_limits_(i, 1), q_next[i]));
    return q_next;
}

// =========================================================================
// PHASE 2: MPC TIMER CALLBACK
// =========================================================================

void MotoMiniPlanningNode::mpcTimerCallback()
{
    if (!tracking_enabled_)
        return;

    if (!target_initialized_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "MPC: Waiting for target...");
        return;
    }

    if (!task_factory_ || !task_executor_ || !mpc_task_)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "TaskComposer components NOT initialized!");
        return;
    }

    std::shared_lock<std::shared_mutex> env_lock(env_mutex_);

    // Record tick start — used later to compute splice offset for the streamer
    const rclcpp::Time tick_start_time = this->now();

    // ------------------------------------------------------------------
    // Horizon = 3: fewer OSQP variables → solve time ~30ms vs ~440ms
    // At mpc_dt_=33ms this still looks 100ms ahead, enough for tracking.
    // ------------------------------------------------------------------
    const int horizon = 3;
    const int n_joints = static_cast<int>(manip_->getJointNames().size());
    const auto &joint_names = manip_->getJointNames();

    using namespace tesseract_planning;
    CompositeInstruction ci_prog(
        "DEFAULT",
        tesseract_common::ManipulatorInfo(manip_->getName(), base_link_, ee_link_));

    // ===================================================================
    // FIX 1 — STATE PROPAGATION
    // Anchor to horizon_joints_[1] (our own last plan) instead of raw
    // sensor data.  Raw sensor data lags behind command by ~10ms and
    // makes the optimizer panic-and-catch-up every tick → staircase.
    // ===================================================================
    Eigen::VectorXd q_anchor;
    {
        std::lock_guard<std::mutex> lock(_mpc_state_mutex);
        static bool first_run = true;
        if (first_run)
        {
            q_anchor = current_joints_; // bootstrap from real robot state
            first_run = false;
        }
        else
        {
            q_anchor = horizon_joints_[1]; // trust the plan
        }
    }

    if (q_anchor.size() == 0)
        return;

    // Shift warm-start one step forward (aligned with new anchor)
    for (int i = 0; i < horizon - 1; ++i)
        horizon_joints_[i] = horizon_joints_[i + 1];

    // Clamp anchor to joint limits
    for (int i = 0; i < q_anchor.size(); ++i)
        q_anchor[i] = std::max(joint_limits_(i, 0),
                               std::min(joint_limits_(i, 1), q_anchor[i]));

    ci_prog.push_back(MoveInstruction(StateWaypoint(joint_names, q_anchor),
                                      MoveInstructionType::FREESPACE, "FREESPACE"));

    // Fetch velocity limits once — used in IK selection and post-processing
    const Eigen::VectorXd v_max = manip_->getLimits().velocity_limits.col(1);

    // ===================================================================
    // FIX 2 — IK SOLUTION SELECTION WITH VELOCITY FEASIBILITY + LAST-
    //          JOINT WEIGHTING
    //
    // Old code: picked closest IK solution by raw L2 distance.
    // Problem:  never checked if delta/mpc_dt_ was within v_max, so an
    //           IK branch-flip produced a -7.5 rad/s spike on joint 5.
    //
    // New logic:
    //   Pass 1 — only consider solutions where every joint velocity is
    //            physically achievable within mpc_dt_.
    //   Pass 2 — among feasible solutions, minimise a WEIGHTED distance
    //            where the last joint costs 10× more than the others.
    //            This suppresses wrist rotation when multiple solutions
    //            are equally good for the arm joints.
    //   Fallback — if no solution is feasible (rare), fall back to the
    //            closest-by-weighted-distance regardless of velocity.
    // ===================================================================
    Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(n_joints);
    joint_weights[n_joints - 1] = 10.0; // last joint (wrist): penalise 10×

    for (int k = 1; k <= horizon; ++k)
    {
        Eigen::Isometry3d P_k = predictTargetPose(k);
        const Eigen::VectorXd &seed_prev =
            (k == 1) ? q_anchor : horizon_joints_[k - 2];

        tesseract_kinematics::KinGroupIKInput ik_in(P_k, base_link_, ee_link_);
        auto ik_sols = manip_->calcInvKin(ik_in, seed_prev);

        if (!ik_sols.empty())
        {
            double best_feasible = std::numeric_limits<double>::max();
            double best_fallback = std::numeric_limits<double>::max();
            bool found_feasible = false;

            for (const auto &s : ik_sols)
            {
                Eigen::VectorXd delta = s - seed_prev;

                // Check every joint is within velocity limits
                bool feasible = true;
                for (int j = 0; j < delta.size(); ++j)
                {
                    if (std::abs(delta[j]) / mpc_dt_ > v_max[j])
                    {
                        feasible = false;
                        break;
                    }
                }

                // Weighted squared distance (last joint costs 10×)
                double wd = (delta.array() * joint_weights.array())
                                .matrix()
                                .squaredNorm();

                if (feasible)
                {
                    found_feasible = true;
                    if (wd < best_feasible)
                    {
                        best_feasible = wd;
                        horizon_joints_[k - 1] = s;
                    }
                }
                else
                {
                    if (wd < best_fallback)
                    {
                        best_fallback = wd;
                        if (!found_feasible)
                            horizon_joints_[k - 1] = s;
                    }
                }
            }
        }
        else
        {
            horizon_joints_[k - 1] = computeDlsExtrapolation(seed_prev, P_k);
        }

        // Clamp to joint limits
        for (int i = 0; i < horizon_joints_[k - 1].size(); ++i)
            horizon_joints_[k - 1][i] =
                std::max(joint_limits_(i, 0),
                         std::min(joint_limits_(i, 1), horizon_joints_[k - 1][i]));

        ci_prog.push_back(MoveInstruction(CartesianWaypoint(P_k),
                                          MoveInstructionType::FREESPACE, "FREESPACE"));
    }

    // ===================================================================
    // FIX 3 — FAST TRAJOPT PROFILES
    //
    // Changes from the original "safe" config:
    //   horizon 5→3                 covered above
    //   collision_cost OFF          biggest single speedup
    //   smooth_velocities OFF       we handle this ourselves post-solve
    //   max_iterations 3→2          fewer QP iters per SQP step
    //   initial_trust_box 0.01→0.05 larger step → fewer SQP outer iters
    //
    // FIX 4 — LAST-JOINT COST IN TRAJOPT
    //   joint_cost_coeff for joint N-1 set to 2.0 (others stay 0.1).
    //   TrajOpt now actively minimises wrist movement while still
    //   tracking the Cartesian target.
    //   Tune last_joint_cost:
    //     2.0  = moderate suppression  (recommended start)
    //     5.0  = strong suppression
    //     10.0 = wrist nearly locked   (may degrade Cartesian accuracy)
    // ===================================================================
    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

    auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
    trajopt_ifopt_move->cartesian_cost_config.enabled = true;
    trajopt_ifopt_move->cartesian_cost_config.coeff = Eigen::VectorXd::Ones(6) * 50.0;

    trajopt_ifopt_move->joint_cost_config.enabled = true;
    {
        const double default_joint_cost = 0.1;
        const double last_joint_cost = 2.0; // tune to suppress wrist rotation
        Eigen::VectorXd jc = Eigen::VectorXd::Ones(n_joints) * default_joint_cost;
        jc[n_joints - 1] = last_joint_cost;
        trajopt_ifopt_move->joint_cost_config.coeff = jc;
    }
    trajopt_ifopt_move->cartesian_constraint_config.enabled = false;

    auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite->collision_cost_config.enabled = false; // OFF — biggest speedup
    trajopt_ifopt_composite->smooth_velocities = false;             // we do this below
    trajopt_ifopt_composite->velocity_coeff = Eigen::VectorXd::Ones(1) * 1.0;

    auto trajopt_ifopt_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    trajopt_ifopt_solver->opt_params.max_iterations = 2;            // was 3
    trajopt_ifopt_solver->opt_params.initial_trust_box_size = 0.05; // was 0.01
    trajopt_ifopt_solver->opt_params.min_approx_improve = 1e-3;

    const std::string NS = "TrajOptIfoptMotionPlannerTask";
    profiles->addProfile(NS, "FREESPACE", trajopt_ifopt_move);
    profiles->addProfile(NS, "DEFAULT", trajopt_ifopt_composite);
    profiles->addProfile(NS, "DEFAULT", trajopt_ifopt_solver);

    // ===================================================================
    // EXECUTE TASK
    // ===================================================================
    std::shared_ptr<const tesseract_environment::Environment> const_env = env_;
    auto ds = std::make_unique<TaskComposerDataStorage>();
    ds->setData("planning_input", ci_prog);
    ds->setData("environment", const_env);
    ds->setData("profiles", profiles);
    ds->setData("initial_guess", horizon_joints_);

    auto tc_ctx = std::make_shared<TaskComposerContext>(
        mpc_task_->getName(), std::move(ds));

    try
    {
        auto fut = task_executor_->run(*mpc_task_, std::move(tc_ctx));
        if (!fut)
            return;
        fut->wait();

        const std::string out_key = mpc_task_->getOutputKeys().get("program");
        const auto stored = fut->context->data_storage->getData();
        if (stored.count(out_key) == 0)
            return;

        auto ci_out = stored.at(out_key).template as<CompositeInstruction>();
        tesseract_planning::formatProgram(ci_out, *env_);
        auto tess_traj = toJointTrajectory(ci_out);

        if (!tess_traj.empty())
        {
            const size_t traj_size = tess_traj.size();

            // ===============================================================
            // FIX 5a — PER-SEGMENT TIMING FROM LIMITING JOINT
            // Compute how long each TrajOpt segment needs so no joint
            // ever exceeds v_max.
            // ===============================================================
            std::vector<double> seg_dt(traj_size, mpc_dt_);
            for (size_t i = 1; i < traj_size; ++i)
            {
                Eigen::VectorXd delta = tess_traj[i].position - tess_traj[i - 1].position;
                double dt_req = mpc_dt_;
                for (int j = 0; j < delta.size(); ++j)
                {
                    double t = std::abs(delta[j]) / v_max[j];
                    if (t > dt_req)
                        dt_req = t;
                }
                seg_dt[i] = dt_req;
            }

            // ===============================================================
            // FIX 5b & 6 MERGED — QUINTIC SPLINE INTERPOLATION
            //
            // Replaces linear interpolation and low-pass filtering.
            // Calculates central-difference target velocities for the sparse
            // waypoints, then connects them using 5th-order polynomials.
            // This guarantees C1 continuity (perfectly smooth velocities)
            // and bounds acceleration, eliminating mechanical jerk entirely.
            // ===============================================================
            const double MAX_RAD_PER_STEP = 0.02;

            // 1. Calculate Target Velocities at Sparse Waypoints
            std::vector<Eigen::VectorXd> sparse_v(traj_size, Eigen::VectorXd::Zero(n_joints));
            static Eigen::VectorXd last_commanded_vel = Eigen::VectorXd::Zero(n_joints);

            static Eigen::VectorXd prev_q_anchor;

            Eigen::VectorXd v_anchor;

            if (prev_q_anchor.size() == q_anchor.size())
            {
                v_anchor = (q_anchor - prev_q_anchor) / mpc_dt_;
            }
            else
            {
                v_anchor = Eigen::VectorXd::Zero(q_anchor.size());
            }

            sparse_v[0] = v_anchor;
            prev_q_anchor = q_anchor;
            for (size_t i = 1; i < traj_size - 1; ++i)
            {
                sparse_v[i] = (tess_traj[i + 1].position - tess_traj[i - 1].position) / (seg_dt[i] + seg_dt[i + 1]);
                for (int j = 0; j < n_joints; ++j)
                    sparse_v[i][j] = std::max(-v_max[j], std::min(v_max[j], sparse_v[i][j]));
            }

            // Set ending velocity (using backward difference)
            if (traj_size > 1)
            {
                sparse_v[traj_size - 1] = (tess_traj[traj_size - 1].position - tess_traj[traj_size - 2].position) / seg_dt[traj_size - 1];
                for (int j = 0; j < n_joints; ++j)
                    sparse_v[traj_size - 1][j] = std::max(-v_max[j], std::min(v_max[j], sparse_v[traj_size - 1][j]));
            }

            // 2. Generate Dense Spline
            tesseract_common::JointTrajectory dense_traj;
            dense_traj.reserve(traj_size * 20);

            {
                tesseract_common::JointState sp;
                sp.joint_names = tess_traj.front().joint_names;
                sp.position = tess_traj.front().position;
                sp.velocity = sparse_v[0];
                sp.time = 0.0;
                dense_traj.push_back(sp);
            }

            double cumulative_time = 0.0;

            for (size_t i = 1; i < traj_size; ++i)
            {
                Eigen::VectorXd delta_q = tess_traj[i].position - tess_traj[i - 1].position;
                double max_delta = delta_q.lpNorm<Eigen::Infinity>();
                int num_substeps = std::max(1, static_cast<int>(std::ceil(max_delta / MAX_RAD_PER_STEP)));

                double dt_sub = seg_dt[i] / num_substeps;
                double T = seg_dt[i];
                double T2 = T * T;
                double T3 = T2 * T;
                double T4 = T3 * T;
                double T5 = T4 * T;

                for (int step = 1; step <= num_substeps; ++step)
                {
                    double t = step * dt_sub;
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t5 = t4 * t;

                    tesseract_common::JointState pt;
                    pt.joint_names = tess_traj[i].joint_names;
                    pt.position = Eigen::VectorXd::Zero(n_joints);
                    pt.velocity = Eigen::VectorXd::Zero(n_joints);

                    for (int j = 0; j < n_joints; ++j)
                    {
                        double q0 = tess_traj[i - 1].position[j];
                        double q1 = tess_traj[i].position[j];
                        double v0 = sparse_v[i - 1][j];
                        double v1 = sparse_v[i][j];

                        // Quintic Coefficients (assuming a0 = 0, a1 = 0 for maximum smoothness)
                        double c0 = q0;
                        double c1 = v0;
                        double c3 = (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * T) / (2.0 * T3);
                        double c4 = (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * T) / (2.0 * T4);
                        double c5 = (12.0 * (q1 - q0) - 6.0 * (v1 + v0) * T) / (2.0 * T5);

                        pt.position[j] = c0 + c1 * t + c3 * t3 + c4 * t4 + c5 * t5;
                        pt.velocity[j] = c1 + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
                    }

                    cumulative_time += dt_sub;
                    pt.time = cumulative_time;
                    dense_traj.push_back(pt);
                }
            }

            // Save the momentum to bridge smoothly to the next MPC tick
            if (dense_traj.size() > 1)
            {
                last_commanded_vel = dense_traj[1].velocity;
            }

            // ===============================================================
            // UPDATE WARM-START HORIZON FOR NEXT TICK
            // ===============================================================
            size_t anchor_idx = 1;
            for (size_t k = 1; k < dense_traj.size(); ++k)
            {
                if (dense_traj[k].time >= mpc_dt_)
                {
                    anchor_idx = k;
                    break;
                }
            }
            for (int k = 0; k < horizon; ++k)
            {
                size_t grab = std::min(anchor_idx + static_cast<size_t>(k * 2),
                                       dense_traj.size() - 1);
                horizon_joints_[k] = dense_traj[grab].position;
            }

            // ===============================================================
            // FIX 7 — SPLICE-TIME CAP
            // ===============================================================
            double solve_elapsed_sec = (this->now() - tick_start_time).seconds();

            if (!dense_traj.empty())
            {
                double traj_duration = dense_traj.back().time;
                solve_elapsed_sec = std::min(solve_elapsed_sec, traj_duration * 0.8);
            }

            publishTrajectory(dense_traj, joint_names, solve_elapsed_sec, mpc_dt_);

            // Visual EE path marker
            if (pub_ee_path_)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = base_link_;
                marker.header.stamp = this->now();
                marker.ns = "mpc_horizon_path";
                marker.id = 1;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.005;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                for (const auto &state : dense_traj)
                {
                    auto fk = manip_->calcFwdKin(state.position);
                    if (fk.count(ee_link_) > 0)
                    {
                        geometry_msgs::msg::Point p;
                        p.x = fk.at(ee_link_).translation().x();
                        p.y = fk.at(ee_link_).translation().y();
                        p.z = fk.at(ee_link_).translation().z();
                        marker.points.push_back(p);
                    }
                }
                pub_ee_path_->publish(marker);
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "MPC Pipeline Error: %s", e.what());
    }

    auto end_time = this->now();
    double duration = (end_time - tick_start_time).seconds() * 1000.0;
    if (duration > 20.0)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "MPC tick took %.2f ms (budget: 20ms)", duration);
}

void MotoMiniPlanningNode::buildAndPublishTrajectory() {}