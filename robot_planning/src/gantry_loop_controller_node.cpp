#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

class GantryLoopControllerNode : public rclcpp::Node
{
public:
    GantryLoopControllerNode() : Node("gantry_loop_controller_node")
    {
        this->declare_parameter<double>("x_start", 0.0);
        this->declare_parameter<double>("x_end", -0.28);
        this->declare_parameter<double>("z_start", 0.0);
        this->declare_parameter<double>("z_end", -0.06);
        this->declare_parameter<int>("steps", 28);
        this->declare_parameter<double>("publish_period_sec", 0.3);
        this->declare_parameter<double>("motion_time_sec", 0.3);

        x_start_ = this->get_parameter("x_start").as_double();
        x_end_ = this->get_parameter("x_end").as_double();
        z_start_ = this->get_parameter("z_start").as_double();
        z_end_ = this->get_parameter("z_end").as_double();
        const int requested_steps = static_cast<int>(this->get_parameter("steps").as_int());
        steps_ = std::max(2, requested_steps);
        publish_period_sec_ = std::max(0.05, this->get_parameter("publish_period_sec").as_double());
        motion_time_sec_ = std::max(0.05, this->get_parameter("motion_time_sec").as_double());

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gantry_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period_sec_),
            std::bind(&GantryLoopControllerNode::publishLoopPoint, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Gantry loop mode started: x %.3f->%.3f, z %.3f->%.3f, steps=%d",
            x_start_, x_end_, z_start_, z_end_, steps_);
    }

private:
    void publishLoopPoint()
    {
        const double phase = static_cast<double>(index_) / static_cast<double>(steps_ - 1);

        const double x_pos = x_start_ + phase * (x_end_ - x_start_);
        const double z_pos = z_start_ + phase * (z_end_ - z_start_);

        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = this->now();
        msg.joint_names = {"joint_x", "joint_z"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {x_pos, z_pos};
        point.time_from_start = rclcpp::Duration::from_seconds(motion_time_sec_);

        msg.points.push_back(point);
        publisher_->publish(msg);

        if (forward_)
        {
            if (index_ >= static_cast<std::size_t>(steps_ - 1))
            {
                forward_ = false; // Reach end, start going back
                --index_;
            }
            else
            {
                ++index_;
            }
        }
        else
        {
            if (index_ <= 0)
            {
                forward_ = true; // Reach start, start going forward
                ++index_;
            }
            else
            {
                --index_;
            }
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_start_{0.0};
    double x_end_{-0.28};
    double z_start_{0.0};
    double z_end_{-0.06};
    double publish_period_sec_{0.3};
    double motion_time_sec_{0.3};
    int steps_{28};
    std::size_t index_{0};
    bool forward_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GantryLoopControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
