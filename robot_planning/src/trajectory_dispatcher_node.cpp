/**
 * @file trajectory_dispatcher_node.cpp
 * @brief Node to re-time and dispatch trajectories to the controller
 * @details Listens for a planned trajectory and a success signal, then stamps the trajectory with "now" and publishes it.
 * @author Bùi Quang Vinh
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class TrajectoryDispatcherNode : public rclcpp::Node
{
public:
    TrajectoryDispatcherNode() : Node("trajectory_dispatcher_node")
    {
        // --- SUBSCRIBERS ---

        // 1. Input: /joint_states (Used to ensure we are alive and optionally for time ref)
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryDispatcherNode::jointStateCallback, this, std::placeholders::_1));

        // 2. Input: /trajectory (The plan starting at t=0)
        sub_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/trajectory", 10,
            std::bind(&TrajectoryDispatcherNode::trajectoryCallback, this, std::placeholders::_1));

        // 3. Input: /optimization_status (The Trigger)
        sub_status_ = this->create_subscription<std_msgs::msg::String>(
            "/optimization_status", 10,
            std::bind(&TrajectoryDispatcherNode::statusCallback, this, std::placeholders::_1));

        // --- PUBLISHERS ---

        // 4. Output: /joint_path_command (The executable trajectory)
        pub_command_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);

        RCLCPP_INFO(this->get_logger(), "Trajectory Dispatcher Node Ready.");
        RCLCPP_INFO(this->get_logger(), "Waiting for '/optimization_status' = 'Success' to dispatch '/trajectory'...");
    }

private:
    // --- MEMBERS ---

    // Store the latest planned trajectory here
    trajectory_msgs::msg::JointTrajectory::SharedPtr pending_trajectory_;

    // Store the latest joint state (optional, mostly for checking system health)
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_trajectory_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_command_;

    // --- CALLBACKS ---

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        pending_trajectory_ = msg;
        // We do NOT publish here. We wait for the "Success" signal.
        RCLCPP_DEBUG(this->get_logger(), "Received a new trajectory plan (buffered).");
    }

    void statusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 1. Check Signal
        if (msg->data != "Success")
        {
            return; // Ignore "Failed" or "Planning Started" messages
        }

        // 2. Validate Buffer
        if (!pending_trajectory_)
        {
            RCLCPP_WARN(this->get_logger(), "Received Success signal, but no trajectory is buffered!");
            return;
        }

        // 3. Re-Time and Dispatch
        RCLCPP_INFO(this->get_logger(), "Dispatching Trajectory...");
        publishAdjustedTrajectory(*pending_trajectory_);

        // Optional: Clear buffer to prevent double sending
        pending_trajectory_.reset();
    }

    void publishAdjustedTrajectory(const trajectory_msgs::msg::JointTrajectory &input_traj)
    {
        trajectory_msgs::msg::JointTrajectory output_traj = input_traj;

        // --- CRITICAL TIMING FIX ---
        // Controllers need to know WHEN to start the trajectory.
        // If header.stamp is 0, they might ignore it or start immediately (implementation dependent).
        // Best practice: Set header.stamp to NOW() + small buffer.

        rclcpp::Time now = this->now();

        // Add a small delay (e.g., 0.1s) to allow the message to reach the controller
        // before the start time passes. This prevents "dropped old point" errors.
        rclcpp::Duration start_delay = rclcpp::Duration::from_seconds(0.1);

        output_traj.header.stamp = now + start_delay;
        output_traj.header.frame_id = "world"; // Ensure frame matches your robot

        // Note: The input_traj.points[i].time_from_start are DURATIONS.
        // They are relative to header.stamp.
        // Example:
        // Header: 12:00:00
        // Point 1: time_from_start = 0.5s -> execute at 12:00:00.5
        // Point 2: time_from_start = 1.0s -> execute at 12:00:01.0
        //
        // So simply updating header.stamp shifts the entire trajectory to the future.
        // We do not need to modify time_from_start inside the loop unless you want to slow it down.

        pub_command_->publish(output_traj);
        RCLCPP_INFO(this->get_logger(), "Sent trajectory with %zu points to /joint_path_command", output_traj.points.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryDispatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}