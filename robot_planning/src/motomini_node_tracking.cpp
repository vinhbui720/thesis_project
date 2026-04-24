/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — MPC Receding Horizon Planner tracking mode.
 *
 * Replaces the old DLS tracking with the high-performance micro-NLP solver directly
 * inside the node per Phase 1-3 requirements.
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
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>

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
        
        if (dt > 1e-4) // Prevent div by zero
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
    // Step B: Target Projection (constant-velocity model)
    // P_k = P_tgt(t) ⊕ (k · Δt · v_tgt(t))
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);
    
    double lead_time = step_k * mpc_dt_;
    Eigen::Isometry3d predicted = current_target_pose_;
    predicted.translation() += target_velocity_linear_ * lead_time;

    // Orientation integration: exp(w * t)
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
    // Extrapolate using Damped Least Squares
    auto fk = manip_->calcFwdKin(q_last);
    if (fk.find(ee_link_) == fk.end()) return q_last; // Safe fallback

    Eigen::Isometry3d ee_current = fk.at(ee_link_);
    Eigen::Vector3d dx = target_next.translation() - ee_current.translation();

    // Axis-angle rotational error
    Eigen::AngleAxisd aa(target_next.linear() * ee_current.linear().inverse());
    Eigen::Vector3d dw = aa.axis() * aa.angle();

    Eigen::Matrix<double, 6, 1> twist;
    twist.head<3>() = dx;
    twist.tail<3>() = dw;

    Eigen::MatrixXd J = manip_->calcJacobian(q_last, base_link_, ee_link_);
    
    const double lambda = 0.01; // Damping
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt += (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(twist);

    return q_last + dq;
}

// =========================================================================
// PHASE 2: MPC TIMER CALLBACK (50 Hz)
// =========================================================================

void MotoMiniPlanningNode::mpcTimerCallback()
{
    if (!tracking_enabled_)
    {
        return;
    }
    
    if (!target_initialized_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "mpcTimerCallback: skipping, target_initialized_ is false");
        return;
    }

    auto start_time = this->now();

    // ---- Step A: Warm-start shift ----
    for (int i = 0; i < mpc_horizon_n_ - 1; ++i)
    {
        horizon_joints_[i] = horizon_joints_[i + 1];
    }
    
    // Extrapolate final step
    Eigen::Isometry3d P_N = predictTargetPose(mpc_horizon_n_);
    horizon_joints_[mpc_horizon_n_ - 1] = computeDlsExtrapolation(horizon_joints_[mpc_horizon_n_ - 2], P_N);

    // ---- Step C: Build micro-NLP ----
    auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
    const int n_dof = manip_->numJoints();

    // Check if manip is valid
    if (!manip_)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "mpcTimerCallback: manip_ is null!");
        return;
    }

    std::vector<trajopt_ifopt::JointPosition::Ptr> vars;
    Eigen::MatrixX2d joint_limits = manip_->getLimits().joint_limits;
    for (int k = 0; k < mpc_horizon_n_; ++k)
    {
        auto var = std::make_shared<trajopt_ifopt::JointPosition>(
            horizon_joints_[k], manip_->getJointNames(), "step_" + std::to_string(k));
        var->SetBounds(joint_limits);
        vars.push_back(var);
        qp_problem->addVariableSet(var);
    }

    // Anchor q_0 to current joint states
    Eigen::VectorXd q_anchor;
    {
        std::lock_guard<std::mutex> lock(_mpc_state_mutex);
        if (current_joints_.size() == n_dof)
            q_anchor = current_joints_;
        else
            q_anchor = horizon_joints_[0]; // Fallback
    }

    auto q0_bounds = std::vector<ifopt::Bounds>(static_cast<size_t>(n_dof));
    for (int i = 0; i < n_dof; ++i)
        q0_bounds[static_cast<size_t>(i)] = ifopt::Bounds(q_anchor[i], q_anchor[i]);

    auto anchor_cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(
        q0_bounds, 
        std::vector<trajopt_ifopt::JointPosition::ConstPtr>{vars[0]},
        Eigen::VectorXd::Ones(n_dof));
    qp_problem->addConstraintSet(anchor_cnt);

    const auto v_lim = manip_->getLimits().velocity_limits;

    // Apply Costs and Constraints across horizon
    for (int k = 0; k < mpc_horizon_n_; ++k)
    {
        if (k > 0)
        {
            // Cartesian tracking penalty (k > 0)
            Eigen::Isometry3d P_k = predictTargetPose(k);
            trajopt_ifopt::CartPosInfo cp_info(manip_, ee_link_, base_link_, Eigen::Isometry3d::Identity(), P_k);
            auto cart_cnt = std::make_shared<trajopt_ifopt::CartPosConstraint>(
                cp_info, vars[k], Eigen::VectorXd::Ones(6) * mpc_w_cart_);
            qp_problem->addCostSet(cart_cnt, trajopt_sqp::CostPenaltyType::SQUARED);

            // Joint Velocity smoothing cost
            auto vel_cnt = std::make_shared<trajopt_ifopt::JointVelConstraint>(
                Eigen::VectorXd::Zero(n_dof), std::vector<trajopt_ifopt::JointPosition::ConstPtr>{vars[k - 1], vars[k]},
                Eigen::VectorXd::Ones(n_dof) * mpc_w_vel_);
            qp_problem->addCostSet(vel_cnt, trajopt_sqp::CostPenaltyType::SQUARED);


        }

        if (k > 0 && k < mpc_horizon_n_ - 1)
        {
            // Joint Acceleration smoothing cost (commented out because it requires 4 variables)
            // auto acc_cnt = std::make_shared<trajopt_ifopt::JointAccelConstraint>(...);
        }

        // Discrete Collision setup
        auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100ul);
        trajopt_common::TrajOptCollisionConfig collision_config(mpc_d_safe_, 10.0);
        auto collision_evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
            collision_cache, manip_, env_, collision_config);
        auto coll_cnt = std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(collision_evaluator, vars[k], 1);
        qp_problem->addConstraintSet(coll_cnt);
    }

    qp_problem->setup();

    // ---- Step D: Solve ----
    auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
    trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
    solver.params.max_iterations = mpc_max_iter_;
    solver.params.initial_trust_box_size = 0.1;
    solver.params.improve_ratio_threshold = 0.1;

    solver.solve(qp_problem);

    // ---- Step E: Extract results ----
    std::vector<Eigen::Vector3d> horizon_path;
    for (int k = 0; k < mpc_horizon_n_; ++k)
    {
        horizon_joints_[k] = vars[k]->GetValues();

        if (pub_ee_path_)
        {
            auto fk = manip_->calcFwdKin(horizon_joints_[k]);
            if (fk.find(ee_link_) != fk.end())
            {
                horizon_path.push_back(fk.at(ee_link_).translation());
            }
        }
    }

    if (pub_ee_path_ && !horizon_path.empty())
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_link_;
        marker.header.stamp = this->now();
        marker.ns = "mpc_horizon_path";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.005;
        // Orange for the MPC horizon path
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        for (const auto &pt : horizon_path)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            marker.points.push_back(p);
        }
        pub_ee_path_->publish(marker);

        // Also plot the TARGET prediction path
        visualization_msgs::msg::Marker target_marker;
        target_marker.header.frame_id = base_link_;
        target_marker.header.stamp = this->now();
        target_marker.ns = "mpc_target_path";
        target_marker.id = 2;
        target_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        target_marker.action = visualization_msgs::msg::Marker::ADD;
        target_marker.scale.x = 0.003;
        // Cyan for target predictions
        target_marker.color.r = 0.0f;
        target_marker.color.g = 1.0f;
        target_marker.color.b = 1.0f;
        target_marker.color.a = 1.0f;
        for (int k = 0; k < mpc_horizon_n_; ++k)
        {
            Eigen::Isometry3d P_k = predictTargetPose(k);
            geometry_msgs::msg::Point p;
            p.x = P_k.translation().x();
            p.y = P_k.translation().y();
            p.z = P_k.translation().z();
            target_marker.points.push_back(p);
        }
        pub_ee_path_->publish(target_marker);
    }

    buildAndPublishTrajectory();

    auto end_time = this->now();
    double duration = (end_time - start_time).seconds() * 1000.0;
    if (duration > 15.0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "MPC tick took %.2f ms (budget: 15ms)", duration);
    }
}

// =========================================================================
// PHASE 3: BUILD AND PUBLISH
// =========================================================================

void MotoMiniPlanningNode::buildAndPublishTrajectory()
{
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = this->now();
    msg.joint_names = manip_->getJointNames();

    const int n_dof = manip_->numJoints();

    for (int k = 0; k < mpc_horizon_n_; ++k)
    {
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(horizon_joints_[k].data(), horizon_joints_[k].data() + n_dof);
        pt.time_from_start = rclcpp::Duration::from_seconds(k * mpc_dt_);

        // Eq 5: Velocity Extraction
        Eigen::VectorXd vel(n_dof);
        if (k == 0)
            vel = (horizon_joints_[1] - horizon_joints_[0]) / mpc_dt_;
        else if (k == mpc_horizon_n_ - 1)
            vel = (horizon_joints_[mpc_horizon_n_ - 1] - horizon_joints_[mpc_horizon_n_ - 2]) / mpc_dt_;
        else
            vel = (horizon_joints_[k + 1] - horizon_joints_[k - 1]) / (2.0 * mpc_dt_);

        pt.velocities.assign(vel.data(), vel.data() + n_dof);
        msg.points.push_back(pt);
    }

    pub_trajectory_->publish(msg);
}