/**
 * @file motomini_node_callbacks.cpp
 * @brief MotoMiniPlanningNode — all ROS2 subscription callbacks and execution monitor.
 *
 * Callbacks:
 *   jointStateCallback      — cache latest joint states; feed online planner
 *   targetPosesCallback     — accumulate Cartesian waypoints
 *   clearCallback           — flush waypoint buffer, cancel any running planner
 *   startCallback           — validate inputs, run full offline planner, start monitor
 *   trackingControlCallback — enable / disable real-time tracking
 *   monitorExecution        — 10 Hz joint-error check; detects timeout and goal reached
 *   publishWaypointsTFs     — broadcast waypoint debug TF frames
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// jointStateCallback
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    last_joint_state_ = msg;

    // Fast capture for MPC anchor
    {
        std::lock_guard<std::mutex> lock(_mpc_state_mutex);
        if (manip_) {
            const auto& joint_names = manip_->getJointNames();
            if (current_joints_.size() != static_cast<long>(joint_names.size())) {
                current_joints_.resize(static_cast<long>(joint_names.size()));
            }
            for (size_t i = 0; i < joint_names.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
                if (it != msg->name.end()) {
                    size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
                    current_joints_[static_cast<std::ptrdiff_t>(i)] = msg->position[idx];
                }
            }
        }
    }

    // Thread-safe environment update for the online SQP solver
    bool online_mode = this->get_parameter("online_mode").as_bool();
    if (is_executing_ && online_mode)
    {
        Eigen::VectorXd joint_pos(static_cast<Eigen::Index>(msg->position.size()));
        for (size_t i = 0; i < msg->position.size(); ++i)
            joint_pos[static_cast<Eigen::Index>(i)] = msg->position[i];
        planner_->updateEnvironmentState(msg->name, joint_pos);
    }
}

// ---------------------------------------------------------------------------
// targetPosesCallback — accumulates poses into a buffer
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::targetPosesCallback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (msg->poses.empty())
        return;

    accumulated_targets_.reserve(accumulated_targets_.size() + msg->poses.size());
    accumulated_targets_.insert(accumulated_targets_.end(),
                                msg->poses.begin(), msg->poses.end());

    RCLCPP_INFO(this->get_logger(), "Received %zu poses. Total accumulated: %zu",
                msg->poses.size(), accumulated_targets_.size());
    publishStatus("Accumulating Poses: " + std::to_string(accumulated_targets_.size()));

    if (this->get_parameter("debug").as_bool() && tf_broadcaster_ && wp_tf_timer_)
        wp_tf_timer_->reset();
}

// ---------------------------------------------------------------------------
// clearCallback — flush buffer and stop any running planner
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::clearCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        return;

    if (this->get_parameter("debug").as_bool() && wp_tf_timer_)
        wp_tf_timer_->cancel();

    accumulated_targets_.clear();
    is_executing_ = false;
    planner_->stopOnlinePlanner();

    RCLCPP_INFO(this->get_logger(), "Target buffer cleared.");
    publishStatus("Buffer Cleared");
}

// ---------------------------------------------------------------------------
// startCallback — validates inputs and runs the full offline planner
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::startCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        return;

    if (tracking_enabled_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Tracking is active. Send /tracking_control false first.");
        publishStatus("Failed: Tracking active");
        return;
    }

    if (is_executing_)
    {
        RCLCPP_WARN(this->get_logger(), "Already executing. Ignoring start signal.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Start signal received!");

    if (!last_joint_state_)
    {
        RCLCPP_WARN(this->get_logger(), "Aborting: No joint states received yet.");
        publishStatus("Failed: No Joint States");
        return;
    }
    if (accumulated_targets_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Aborting: No targets accumulated.");
        publishStatus("Failed: No Targets");
        return;
    }

    publishStatus("Planning Started...");

    // Sync environment with latest real joint values
    Eigen::VectorXd joint_pos(
        static_cast<Eigen::Index>(last_joint_state_->position.size()));
    for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
        joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[i];
    planner_->updateEnvironmentState(last_joint_state_->name, joint_pos);

    // Convert ROS poses to Eigen
    std::vector<Eigen::Isometry3d> eigen_poses;
    eigen_poses.reserve(accumulated_targets_.size());
    for (const auto &ros_pose : accumulated_targets_)
    {
        Eigen::Quaterniond q(
            ros_pose.orientation.w, ros_pose.orientation.x,
            ros_pose.orientation.y, ros_pose.orientation.z);
        Eigen::Translation3d t(
            ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
        eigen_poses.push_back(t * q);
    }

    planner_->setTargetPoses(eigen_poses);
    const bool success = planner_->run();

    if (success)
    {
        auto traj_ptr = planner_->getTrajectory();
        if (traj_ptr && !traj_ptr->empty())
        {
            // Publish the complete stitched trajectory once.
            // Per-chunk streaming is NOT used: the Motoman controller does not queue
            // trajectories — each new message preempts the current one, so only
            // the last chunk would ever fully execute if we published per-chunk.
            static const std::vector<std::string> planned_joints = {
                "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
            publishTrajectory(*traj_ptr, planned_joints);

            // Set up execution monitor
            target_joint_names_ = last_joint_state_->name;
            Eigen::VectorXd final_pos = traj_ptr->back().position;
            final_joint_target_.assign(final_pos.data(), final_pos.data() + final_pos.size());
            expected_execution_duration_ = traj_ptr->back().time;
            execution_start_time_ = this->now();
            is_executing_ = true;

            bool online_mode = this->get_parameter("online_mode").as_bool();
            publishStatus(online_mode ? "Optimization Success. Executing ONLINE..."
                                      : "Optimization Success. Executing STATIC...");
            RCLCPP_INFO(this->get_logger(),
                        "Full trajectory published: %zu pts, %.2f s. Monitoring joints...",
                        traj_ptr->size(), traj_ptr->back().time);
        }
        else
        {
            publishStatus("Failed: Trajectory Empty");
        }
    }
    else
    {
        publishStatus("Failed: Optimization Error");
    }
}

// ---------------------------------------------------------------------------
// trackingControlCallback — runtime mode switch: true = tracking, false = planning
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingControlCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    tracking_enabled_ = msg->data;
    if (msg->data)
    {
        // Reset target initialization so tracking re-locks onto the current tip position.
        {
            std::lock_guard<std::mutex> lock(_mpc_target_mutex);
            target_initialized_ = false;
        }

        // Initialize MPC horizon with current robot position to avoid jumps
        {
            std::lock_guard<std::mutex> lock(_mpc_state_mutex);
            if (current_joints_.size() > 0) {
                if (horizon_joints_.size() != static_cast<size_t>(mpc_horizon_n_))
                    horizon_joints_.resize(static_cast<size_t>(mpc_horizon_n_));
                for (auto& q : horizon_joints_) q = current_joints_;
                last_tracking_velocity_command_ =
                    Eigen::VectorXd::Zero(current_joints_.size());
            }
        }
        tracking_start_time_ = this->now();
        has_tracking_velocity_command_ = false;
        mode_.store(ControllerMode::TRACKING);
        avoidance_traj_valid_.store(false);
        RCLCPP_INFO(this->get_logger(), "Mode → TRACKING");
        publishStatus("Mode: Tracking");
    }
    else
    {
        has_tracking_velocity_command_ = false;
        RCLCPP_INFO(this->get_logger(), "Mode → PLANNING (use /target_poses + /start)");
        publishStatus("Mode: Planning");
    }
}

// ---------------------------------------------------------------------------
// monitorExecution — 10 Hz joint-error check
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::monitorExecution()
{
    if (!is_executing_ || !last_joint_state_)
        return;

    // Timeout check
    const double elapsed = (this->now() - execution_start_time_).seconds();
    if (elapsed > expected_execution_duration_ + TIMEOUT_BUFFER)
    {
        is_executing_ = false;
        planner_->stopOnlinePlanner();
        publishStatus("Failed: Execution Timeout");
        RCLCPP_ERROR(this->get_logger(), "Robot did not reach target within expected time.");
        return;
    }

    // Joint-error check
    double max_error = 0.0;
    for (size_t i = 0; i < target_joint_names_.size(); ++i)
    {
        auto it = std::find(last_joint_state_->name.begin(),
                            last_joint_state_->name.end(),
                            target_joint_names_[i]);
        if (it == last_joint_state_->name.end())
            continue;
        const size_t idx = static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
        const double error = std::abs(last_joint_state_->position[idx] - final_joint_target_[i]);
        if (error > max_error)
            max_error = error;
    }

    if (max_error < JOINT_TOLERANCE)
    {
        is_executing_ = false;
        planner_->stopOnlinePlanner();
        publishStatus("Success");
        RCLCPP_INFO(this->get_logger(),
                    "Robot reached target! (Max error: %.4f rad)", max_error);
    }
}

// ---------------------------------------------------------------------------
// publishWaypointsTFs — broadcast waypoint debug TF frames
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::publishWaypointsTFs()
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(accumulated_targets_.size());

    for (size_t i = 0; i < accumulated_targets_.size(); ++i)
    {
        const auto &pose = accumulated_targets_[i];
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "wp_" + std::to_string(i);
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;
        transforms.push_back(tf);
    }
    tf_broadcaster_->sendTransform(transforms);
}
