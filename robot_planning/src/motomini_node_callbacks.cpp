/**
 * @file motomini_node_callbacks.cpp
 * @brief MotoMiniPlanningNode — all ROS2 subscription callbacks and execution monitor.
 *
 * Callbacks:
 *   jointStateCallback      — cache latest joint states; feed online planner
 *   targetPosesCallback     — accumulate Cartesian waypoints
 *   clearCallback           — flush waypoint buffer, cancel any running planner
 *   startCallback           — validate inputs, run full offline planner, start monitor
 *   monitorExecution        — clears the busy flag after the published trajectory duration
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
                    "Tracking stream is active. Send /tracking_control false before /start.");
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
            const bool all_waypoints_published =
                publishTrajectory(*traj_ptr, planned_joints);
            if (!all_waypoints_published)
            {
                publishStatus("Failed: Trajectory Publish Error");
                RCLCPP_ERROR(this->get_logger(),
                             "Trajectory publish failed before all planned points were staged.");
                return;
            }

            // Status is publish-based: once all planned points are in the
            // outgoing JointTrajectory, the planner has completed its job.
            expected_execution_duration_ = std::max(0.0, traj_ptr->back().time);
            execution_start_time_ = this->now();
            is_executing_ = true;

            const bool online_mode = this->get_parameter("online_mode").as_bool();
            publishStatus("Success");
            RCLCPP_INFO(this->get_logger(),
                        "Full trajectory published: %zu pts, %.2f s. Status marked Success after all points were staged%s.",
                        traj_ptr->size(), traj_ptr->back().time,
                        online_mode ? " (online mode)" : "");
            if (online_mode)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Online planner remains active until the published trajectory duration completes.");
            }
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
// monitorExecution — keep the node busy until the published trajectory duration
// has elapsed. Status success is decided when the trajectory is published, not
// by checking final joint error against /joint_states.
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::monitorExecution()
{
    if (!is_executing_)
        return;

    const double elapsed = (this->now() - execution_start_time_).seconds();
    if (elapsed >= expected_execution_duration_)
    {
        is_executing_ = false;
        planner_->stopOnlinePlanner();
        RCLCPP_INFO(this->get_logger(),
                    "Published trajectory duration elapsed; ending planner busy state without final-target error check.");
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
