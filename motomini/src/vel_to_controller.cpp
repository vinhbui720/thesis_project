/**
 * @file vel_to_controller.cpp
 * @brief Converts JointTrajectory velocity data to position commands for velocity controller
 *
 * Integrates velocities and publishes to velocity controller in position mode
 * Subscribes to /joint_command (JointTrajectory with velocities)
 * Publishes to /motomini_vel_controller/commands (Float64MultiArray with integrated positions)
 */

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

class VelToController : public rclcpp::Node
{
public:
    VelToController() : Node("vel_to_controller")
    {
        // Subscribe to joint command topic
        sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_command", 10,
            std::bind(&VelToController::trajectoryCallback, this, std::placeholders::_1));

        // Publish to velocity controller (actually sends positions since mock hw doesn't support vel)
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motomini_vel_controller/commands", 10);

        // Initialize tracked positions to zero
        tracked_positions_.resize(6, 0.0);

        RCLCPP_INFO(this->get_logger(),
                    "Velocity converter started (integrating mode): /joint_command -> /motomini_vel_controller/commands");
    }

private:
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Extract positions from the trajectory point
        if (msg->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
            return;
        }

        const auto &point = msg->points[0];

        if (point.positions.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Trajectory point has no positions");
            return;
        }

        // Use positions from trajectory (already integrated by motomini_vel_tracking)
        std_msgs::msg::Float64MultiArray pos_cmd;
        pos_cmd.data = point.positions;
        pub_->publish(pos_cmd);
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    std::vector<double> tracked_positions_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelToController>());
    rclcpp::shutdown();
    return 0;
}
