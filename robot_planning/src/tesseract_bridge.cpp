#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>

class TesseractBridge : public rclcpp::Node
{
public:
    TesseractBridge() : Node("tesseract_bridge_node")
    {

        sub_ = this->create_subscription<tesseract_msgs::msg::Trajectory>(
            "/tesseract/display_tesseract_trajectory",
            10,
            std::bind(&TesseractBridge::topic_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory",
            10);

        RCLCPP_INFO(this->get_logger(), "Bridge Initialized. Waiting for Tesseract plan...");
    }

private:
    void topic_callback(const tesseract_msgs::msg::Trajectory::SharedPtr msg) const
    {
        if (msg->joint_trajectories.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received Tesseract message, but 'joint_trajectories' list is empty.");
            return;
        }

        const auto &tesseract_path = msg->joint_trajectories[0];

        if (tesseract_path.states.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory has no states (waypoints). Ignoring.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received valid plan! Extracting %zu waypoints...", tesseract_path.states.size());


        trajectory_msgs::msg::JointTrajectory robot_msg;
        robot_msg.header.stamp = this->now();
        robot_msg.header.frame_id = "world"; 

        robot_msg.joint_names = tesseract_path.states[0].joint_names;


        for (const auto &state : tesseract_path.states)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;

            point.positions = state.position;

            point.velocities = state.velocity;


            point.accelerations = state.acceleration;

            point.time_from_start = state.time_from_start;

            robot_msg.points.push_back(point);
        }

        pub_->publish(robot_msg);
        RCLCPP_INFO(this->get_logger(), ">>> Sent full trajectory to /joint_trajectory successfully!");
    }

    rclcpp::Subscription<tesseract_msgs::msg::Trajectory>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TesseractBridge>());
    rclcpp::shutdown();
    return 0;
}