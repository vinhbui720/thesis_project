#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class SmoothJoggingNode : public rclcpp::Node
{
public:
    SmoothJoggingNode() : Node("smooth_jogging")
    {
        this->declare_parameter<int>("rate", 40);
        this->declare_parameter<double>("max_translat_acc", 0.015);
        this->declare_parameter<double>("max_rot_acc", 0.05);

        rate_hz_ = std::max(1, static_cast<int>(this->get_parameter("rate").as_int()));
        g_translat_acc_ = this->get_parameter("max_translat_acc").as_double();
        g_rot_acc_ = this->get_parameter("max_rot_acc").as_double();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pose_following/cmd_vel", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/smooth_jogging/cmd_vel", 10,
            std::bind(&SmoothJoggingNode::cmdVelCallback, this, std::placeholders::_1));

        cart_acc_.setZero();
        cart_vel_.setZero();
        vel_err_.setZero();
        dof_.fill(0.0);

        last_tick_time_ = this->now();
        const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / static_cast<double>(rate_hz_)));
        timer_ = this->create_wall_timer(period, std::bind(&SmoothJoggingNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "smooth_jogging node started at %d Hz", rate_hz_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        dof_[0] = msg->linear.x;
        dof_[1] = msg->linear.y;
        dof_[2] = msg->linear.z;
        dof_[3] = msg->angular.x;
        dof_[4] = msg->angular.y;
        dof_[5] = msg->angular.z;
    }

    void tick()
    {
        const auto now = this->now();
        double dt = (now - last_tick_time_).seconds();
        if (dt <= 1e-6)
            dt = 1.0 / static_cast<double>(rate_hz_);
        last_tick_time_ = now;

        Eigen::VectorXd vel_command(6);

        double sum_squares = 0.0;
        for (size_t i = 0; i < 3; ++i)
            sum_squares += dof_[i] * dof_[i];
        double delta_mag = std::sqrt(sum_squares);

        for (size_t i = 0; i < 3; ++i)
            vel_command[static_cast<Eigen::Index>(i)] = (delta_mag > 0.0) ? dof_[i] : 0.0;

        sum_squares = 0.0;
        for (size_t i = 3; i < 6; ++i)
            sum_squares += dof_[i] * dof_[i];
        delta_mag = std::sqrt(sum_squares);

        for (size_t i = 3; i < 6; ++i)
            vel_command[static_cast<Eigen::Index>(i)] = (delta_mag > 0.0) ? dof_[i] : 0.0;

        err_dot_ = (1.0 / dt) * ((vel_command - cart_vel_) - vel_err_);
        vel_err_ = vel_command - cart_vel_;

        const double err_mag = vel_err_.norm();

        if (err_mag <= 0.015)
        {
            cart_vel_ = vel_command;
            cart_acc_.setZero();
        }
        else
        {
            const Eigen::VectorXd vel_dir = vel_err_.normalized();
            for (size_t i = 0; i < 3; ++i)
                cart_acc_[static_cast<Eigen::Index>(i)] = vel_dir[static_cast<Eigen::Index>(i)] * g_translat_acc_ / dt;
            for (size_t i = 3; i < 6; ++i)
                cart_acc_[static_cast<Eigen::Index>(i)] = vel_dir[static_cast<Eigen::Index>(i)] * g_rot_acc_ / dt;

            cart_vel_ = cart_vel_ + dt * cart_acc_;
        }

        if (cart_vel_.norm() == 0.0)
            cart_vel_.setZero();

        geometry_msgs::msg::Twist out;
        out.linear.x = cart_vel_[0];
        out.linear.y = cart_vel_[1];
        out.linear.z = cart_vel_[2];
        out.angular.x = cart_vel_[3];
        out.angular.y = cart_vel_[4];
        out.angular.z = cart_vel_[5];
        cmd_vel_pub_->publish(out);
    }

    int rate_hz_{40};
    double g_translat_acc_{0.015};
    double g_rot_acc_{0.05};

    std::array<double, 6> dof_{};
    Eigen::VectorXd cart_acc_{6};
    Eigen::VectorXd cart_vel_{6};
    Eigen::VectorXd vel_err_{6};
    Eigen::VectorXd err_dot_{6};

    rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmoothJoggingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
