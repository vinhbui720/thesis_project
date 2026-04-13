#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <string>
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
        this->declare_parameter<double>("loop_period_sec", 0.0);

        x_start_ = this->get_parameter("x_start").as_double();
        x_end_ = this->get_parameter("x_end").as_double();
        z_start_ = this->get_parameter("z_start").as_double();
        z_end_ = this->get_parameter("z_end").as_double();
        const int requested_steps = static_cast<int>(this->get_parameter("steps").as_int());
        steps_ = std::max(2, requested_steps);
        publish_period_sec_ = std::max(0.05, this->get_parameter("publish_period_sec").as_double());
        motion_time_sec_ = std::max(0.05, this->get_parameter("motion_time_sec").as_double());
        loop_period_sec_ = this->get_parameter("loop_period_sec").as_double();

        // Keep backward compatibility: if no explicit loop period is provided,
        // derive it from the old stepped behavior.
        if (loop_period_sec_ <= 0.0)
        {
            loop_period_sec_ = publish_period_sec_ * 2.0 * static_cast<double>(steps_ - 1);
        }
        loop_period_sec_ = std::max(loop_period_sec_, publish_period_sec_ * 4.0);

        // Give the trajectory controller a horizon longer than one publish period
        // so it can interpolate smoothly rather than stop-go each command.
        motion_time_sec_ = std::max(motion_time_sec_, publish_period_sec_ * 1.5);

        start_time_ = this->now();

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gantry_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period_sec_),
            std::bind(&GantryLoopControllerNode::publishLoopPoint, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Gantry loop mode started: x %.3f->%.3f, z %.3f->%.3f, loop_period=%.3fs, publish_period=%.3fs",
            x_start_, x_end_, z_start_, z_end_, loop_period_sec_, publish_period_sec_);
    }

private:
    void publishLoopPoint()
    {
        constexpr double kTwoPi = 6.283185307179586;
        const double elapsed_sec = (this->now() - start_time_).seconds();
        const double cycle_phase = std::fmod(elapsed_sec, loop_period_sec_) / loop_period_sec_;
        const double angle = kTwoPi * cycle_phase;

        const double x_mid = 0.5 * (x_start_ + x_end_);
        const double z_mid = 0.5 * (z_start_ + z_end_);
        const double x_amp = 0.5 * (x_start_ - x_end_);
        const double z_amp = 0.5 * (z_start_ - z_end_);

        const double x_pos = x_mid + x_amp * std::cos(angle);
        const double z_pos = z_mid + z_amp * std::cos(angle);

        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = this->now();
        msg.joint_names = {"joint_x", "joint_z"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {x_pos, z_pos};
        point.time_from_start = rclcpp::Duration::from_seconds(motion_time_sec_);

        msg.points.push_back(point);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_start_{0.0};
    double x_end_{-0.28};
    double z_start_{0.0};
    double z_end_{-0.06};
    double publish_period_sec_{0.3};
    double motion_time_sec_{0.3};
    double loop_period_sec_{0.0};
    int steps_{28};
    rclcpp::Time start_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GantryLoopControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
