/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — MPC Receding Horizon Planner tracking mode.
 *
 * Optimized for high-frequency (30Hz) execution to feed the trajectory streamer.
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_environment/environment.h>

// TrajOpt SQP formulation
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_common/collision_types.h>

// TrajOpt Ifopt constraints and costs
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

// Tesseract Planning
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

// TF/Math
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

// =========================================================================
// PHASE 1: ASYNC CALLBACKS
// =========================================================================

void MotoMiniPlanningNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
            // Estimate target velocities via finite differencing
            target_velocity_linear_ = (new_pose.translation() - current_target_pose_.translation()) / dt;
            
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
            Eigen::AngleAxisd(angle, target_velocity_angular_.normalized()).toRotationMatrix() *
            current_target_pose_.linear();
    }
    return predicted;
}


Eigen::VectorXd MotoMiniPlanningNode::computeDlsExtrapolation(const Eigen::VectorXd& q_last,
                                                              const Eigen::Isometry3d& target_next)
{
    auto fk = manip_->calcFwdKin(q_last);
    if (fk.find(ee_link_) == fk.end()) return q_last;

    Eigen::Isometry3d ee_current = fk.at(ee_link_);
    Eigen::Vector3d dx = target_next.translation() - ee_current.translation();
    Eigen::AngleAxisd aa(target_next.linear() * ee_current.linear().inverse());
    Eigen::Vector3d dw = aa.axis() * aa.angle();

    Eigen::Matrix<double, 6, 1> twist;
    twist.head<3>() = dx;
    twist.tail<3>() = dw;

    Eigen::MatrixXd J = manip_->calcJacobian(q_last, base_link_, ee_link_);
    const double lambda = 0.05; // Slightly higher damping for smoothness
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt += (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(twist);

    Eigen::VectorXd q_next = q_last + dq;
    for (int i = 0; i < q_next.size(); ++i)
        q_next[i] = std::max(joint_limits_(i, 0), std::min(joint_limits_(i, 1), q_next[i]));

    return q_next;
}

// =========================================================================
// PHASE 2: MPC TIMER CALLBACK (30 Hz recommended to stay away from streamer 20ms guard)
// =========================================================================

void MotoMiniPlanningNode::mpcTimerCallback()
{
    if (!tracking_enabled_) return;
    
    if (!target_initialized_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "MPC: Waiting for target...");
        return;
    }

    if (!task_factory_ || !task_executor_ || !mpc_task_)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "TaskComposer components NOT initialized!");
        return;
    }

    std::shared_lock<std::shared_mutex> env_lock(env_mutex_);
    auto start_time = this->now();

    // 1. Warm-start shift
    int horizon = 5; 
    for (int i = 0; i < horizon - 1; ++i)
        horizon_joints_[i] = horizon_joints_[i + 1];
    
    const std::vector<std::string> joint_names = manip_->getJointNames();
    
    using namespace tesseract_planning;
    CompositeInstruction ci_prog("DEFAULT", 
        tesseract_common::ManipulatorInfo(manip_->getName(), base_link_, ee_link_));
    
    // 2. Get current joints as anchor
    Eigen::VectorXd q_anchor;
    {
        std::lock_guard<std::mutex> lock(_mpc_state_mutex);
        q_anchor = current_joints_;
    }
    if (q_anchor.size() == 0) return;

    // Clamp anchor
    for (int i = 0; i < q_anchor.size(); ++i)
        q_anchor[i] = std::max(joint_limits_(i, 0), std::min(joint_limits_(i, 1), q_anchor[i]));

    ci_prog.push_back(MoveInstruction(StateWaypoint(joint_names, q_anchor), 
                                      MoveInstructionType::FREESPACE, "FREESPACE"));

    // 3. Build horizon with CartesianWaypoints for precision
    for (int k = 1; k <= horizon; ++k)
    {
        Eigen::Isometry3d P_k = predictTargetPose(k);
        const Eigen::VectorXd& seed_prev = (k == 1) ? q_anchor : horizon_joints_[k-2];
        
        tesseract_kinematics::KinGroupIKInput ik_in(P_k, base_link_, ee_link_);
        auto ik_sols = manip_->calcInvKin(ik_in, seed_prev);

        if (!ik_sols.empty())
        {
            double best_d = std::numeric_limits<double>::max();
            for (const auto& s : ik_sols)
            {
                double d = (s - seed_prev).squaredNorm();
                if (d < best_d) { best_d = d; horizon_joints_[k - 1] = s; }
            }
        }
        else
            horizon_joints_[k - 1] = computeDlsExtrapolation(seed_prev, P_k);

        for (int i = 0; i < horizon_joints_[k-1].size(); ++i)
            horizon_joints_[k-1][i] = std::max(joint_limits_(i, 0), std::min(joint_limits_(i, 1), horizon_joints_[k-1][i]));

        ci_prog.push_back(MoveInstruction(CartesianWaypoint(P_k), 
                                          MoveInstructionType::FREESPACE, "FREESPACE"));
    }

    // 4. Configure Fast Profiles
    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
    
    auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
    trajopt_ifopt_move->cartesian_cost_config.enabled = true;
    trajopt_ifopt_move->cartesian_cost_config.coeff = Eigen::VectorXd::Ones(6) * 50.0;
    trajopt_ifopt_move->joint_cost_config.enabled = true;
    trajopt_ifopt_move->joint_cost_config.coeff = Eigen::VectorXd::Ones(joint_names.size()) * 0.1;
    trajopt_ifopt_move->cartesian_constraint_config.enabled = false;

    auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite->collision_cost_config.enabled = (mpc_coll_type_ != -1);
    trajopt_ifopt_composite->collision_cost_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    trajopt_ifopt_composite->collision_cost_config.collision_margin_buffer = 0.01;
    trajopt_ifopt_composite->smooth_velocities = true;
    trajopt_ifopt_composite->velocity_coeff = Eigen::VectorXd::Ones(1) * 1.0;

    auto trajopt_ifopt_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    trajopt_ifopt_solver->opt_params.max_iterations = 3; 
    trajopt_ifopt_solver->opt_params.initial_trust_box_size = 0.01; 
    trajopt_ifopt_solver->opt_params.min_approx_improve = 1e-3;

    const std::string NS = "TrajOptIfoptMotionPlannerTask";
    profiles->addProfile(NS, "FREESPACE", trajopt_ifopt_move);
    profiles->addProfile(NS, "DEFAULT", trajopt_ifopt_composite);
    profiles->addProfile(NS, "DEFAULT", trajopt_ifopt_solver);

    // 5. Execute Task
    std::shared_ptr<const tesseract_environment::Environment> const_env = env_;
    auto ds = std::make_unique<TaskComposerDataStorage>();
    ds->setData("planning_input", ci_prog);
    ds->setData("environment", const_env);
    ds->setData("profiles", profiles);
    ds->setData("initial_guess", horizon_joints_);

    auto tc_ctx = std::make_shared<TaskComposerContext>(mpc_task_->getName(), std::move(ds));
    
    try {
        auto fut = task_executor_->run(*mpc_task_, std::move(tc_ctx));
        if (!fut) return;
        fut->wait();

        const std::string out_key = mpc_task_->getOutputKeys().get("program");
        const auto stored = fut->context->data_storage->getData();
        if (stored.count(out_key) == 0) return;

        auto ci_out = stored.at(out_key).template as<CompositeInstruction>();
        
        // Convert to joint positions and extract trajectory
        tesseract_planning::formatProgram(ci_out, *env_);
        auto tess_traj = toJointTrajectory(ci_out);

        if (!tess_traj.empty())
        {
            // CRITICAL: Accurate velocities and time stamps for the streamer
            for (size_t i = 0; i < tess_traj.size(); ++i)
            {
                tess_traj[i].time = i * mpc_dt_;
                if (i > 0)
                    tess_traj[i].velocity = (tess_traj[i].position - tess_traj[i-1].position) / mpc_dt_;
                else
                    tess_traj[i].velocity = Eigen::VectorXd::Zero(tess_traj[i].position.size());
            }

            for (size_t k = 0; k < std::min((size_t)horizon, tess_traj.size()); ++k)
                horizon_joints_[k] = tess_traj[k].position;

            // Feed full horizon to streamer on /path_command
            publishTrajectory(tess_traj, joint_names, 0.0, mpc_dt_);
            
            // Visual Marker
            if (pub_ee_path_)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = base_link_;
                marker.header.stamp = this->now();
                marker.ns = "mpc_horizon_path";
                marker.id = 1;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP; marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.005;
                marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
                for (const auto& state : tess_traj)
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
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC Pipeline Error: %s", e.what());
    }

    auto end_time = this->now();
    double duration = (end_time - start_time).seconds() * 1000.0;
    if (duration > 20.0)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC tick took %.2f ms (budget: 20ms)", duration);
}

void MotoMiniPlanningNode::buildAndPublishTrajectory() {}
