#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;

class MotoMiniBridge : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    MotoMiniBridge() : Node("motomini_bridge_node")
    {
        // 1. Subscribe to the GUI sliders
        // We listen to 'gui_commands' instead of 'joint_states' to avoid conflict with the real robot feedback
        sub_gui_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/gui_commands", 10, std::bind(&MotoMiniBridge::gui_callback, this, _1));

        // 2. Create Action Client for the robot
        client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_trajectory_action");

        RCLCPP_INFO(this->get_logger(), "Bridge Initialized: GUI(JointState) -> Hardware(Trajectory)");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gui_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;

    // Store the last goal handle to cancel it if a new command comes fast
    std::shared_future<GoalHandleFJT::SharedPtr> future_goal_handle_;

    void gui_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();

        // Copy Joint Names from the GUI message
        goal_msg.trajectory.joint_names = msg->name;

        // Create the Target Point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = msg->position;

        // We set a small duration (0.3s) to make the movement smooth but responsive
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 300000000; // 300ms

        goal_msg.trajectory.points.push_back(point);

        // Send the goal asynchronously
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniBridge>());
    rclcpp::shutdown();
    return 0;
}