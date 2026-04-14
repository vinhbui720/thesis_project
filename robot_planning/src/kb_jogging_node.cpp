#include <cstdio>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

class KeyboardJoggingNode : public rclcpp::Node
{
public:
    KeyboardJoggingNode() : Node("kb_jogging")
    {
        this->declare_parameter<double>("jogging_velocity", 0.001);
        this->declare_parameter<int>("rate_hz", 40);
        this->declare_parameter<double>("max_trans_vel", 0.5);
        this->declare_parameter<std::string>("frame_id", "imu_angle");

        jogging_velocity_ = this->get_parameter("jogging_velocity").as_double();
        rate_hz_ = std::max(1, static_cast<int>(this->get_parameter("rate_hz").as_int()));
        max_cart_translation_vel_ = this->get_parameter("max_trans_vel").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_following/pose", 10);

        pose_.position.x = 0.185;
        pose_.position.y = 0.0;
        pose_.position.z = 0.228;
        pose_.orientation.x = 1.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 0.0;

        cart_translation_vel_ = std::min(max_cart_translation_vel_, max_cart_translation_vel_);

        configureTerminal();

        const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / static_cast<double>(rate_hz_)));
        timer_ = this->create_wall_timer(period, std::bind(&KeyboardJoggingNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "Keyboard jogging online.");
        RCLCPP_INFO(this->get_logger(), "Translation: +x:1 +y:2 +z:3  -x:q -y:w -z:e");
        RCLCPP_INFO(this->get_logger(), "Velocity: +:0 -:p, Stop:x");
    }

    ~KeyboardJoggingNode() override
    {
        restoreTerminal();
    }

private:
    void configureTerminal()
    {
        if (terminal_configured_)
            return;

        if (tcgetattr(STDIN_FILENO, &old_termios_) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read terminal attributes");
            return;
        }

        termios new_termios = old_termios_;
        new_termios.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set terminal to raw mode");
            return;
        }

        terminal_configured_ = true;
    }

    void restoreTerminal()
    {
        if (!terminal_configured_)
            return;

        (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        terminal_configured_ = false;
    }

    char getchAsync() const
    {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = static_cast<suseconds_t>(1000000 / std::max(1, rate_hz_));

        const int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
        if (rv <= 0)
            return '\0';

        char c = '\0';
        (void)read(STDIN_FILENO, &c, 1);
        return c;
    }

    void tick()
    {
        const char command = getchAsync();

        switch (command)
        {
        case '1':
            pose_.position.x += jogging_velocity_ * cart_translation_vel_;
            break;
        case '2':
            pose_.position.y += jogging_velocity_ * cart_translation_vel_;
            break;
        case '3':
            pose_.position.z += jogging_velocity_ * cart_translation_vel_;
            break;
        case 'q':
            pose_.position.x -= jogging_velocity_ * cart_translation_vel_;
            break;
        case 'w':
            pose_.position.y -= jogging_velocity_ * cart_translation_vel_;
            break;
        case 'e':
            pose_.position.z -= jogging_velocity_ * cart_translation_vel_;
            break;
        case '0':
            jogging_velocity_ += 0.001;
            if (jogging_velocity_ > 0.01)
                jogging_velocity_ = 0.01;
            RCLCPP_INFO(this->get_logger(), "Jogging velocity factor: %.3f", jogging_velocity_);
            break;
        case 'p':
            jogging_velocity_ -= 0.001;
            if (jogging_velocity_ < 0.0)
                jogging_velocity_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Jogging velocity factor: %.3f", jogging_velocity_);
            break;
        case 'x':
            rclcpp::shutdown();
            return;
        default:
            break;
        }

        geometry_msgs::msg::PoseStamped pose_ref;
        pose_ref.header.stamp = this->now();
        pose_ref.header.frame_id = frame_id_;
        pose_ref.pose = pose_;
        pose_pub_->publish(pose_ref);
    }

    double jogging_velocity_{0.001};
    double max_cart_translation_vel_{0.5};
    double cart_translation_vel_{0.5};
    int rate_hz_{40};
    std::string frame_id_{"imu_angle"};

    geometry_msgs::msg::Pose pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool terminal_configured_{false};
    termios old_termios_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardJoggingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
