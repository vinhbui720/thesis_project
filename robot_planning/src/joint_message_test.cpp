#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class JointState : public rclcpp::Node
{
public:
    sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointState::call_back, this, _1));
    RCLCPP_INFO(this->get_logger(), "Test Ros1_bridge");

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState> joint_state_;
    void call_back(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "")
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointState>());
    rclcpp::shutdown();
    return 0;
}