#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ObstacleSimulator : public rclcpp::Node
{
public:
    ObstacleSimulator() : Node("obstacle_simulator"), active_(false)
    {
        this->declare_parameter<double>("active_x", 0.2);
        this->declare_parameter<double>("active_y", 0.0);
        this->declare_parameter<double>("active_z", 0.1);
        this->declare_parameter<double>("hidden_z", -10.0);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_trigger", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                active_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Obstacle trigger: %s", active_ ? "ACTIVE" : "HIDDEN");
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ObstacleSimulator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Simulator Started.");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "obstacle_link";

        if (active_)
        {
            t.transform.translation.x = this->get_parameter("active_x").as_double();
            t.transform.translation.y = this->get_parameter("active_y").as_double();
            t.transform.translation.z = this->get_parameter("active_z").as_double();
        }
        else
        {
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = this->get_parameter("hidden_z").as_double();
        }

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    bool active_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleSimulator>());
    rclcpp::shutdown();
    return 0;
}
