// This node subscribes to sensor_msgs/msg/JointState (velocity commands)
// and /joint_states (current positions), integrates velocity to position,
// and publishes a trajectory_msgs/msg/JointTrajectory to /path_command.
// Only active when vel_streaming is true and real_robot is false.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class JointStateToTrajectoryNode : public rclcpp::Node
{
public:
    JointStateToTrajectoryNode() : Node("joint_state_to_trajectory")
    {
        this->declare_parameter<double>("rate_hz", 50.0);
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command", 10,
            std::bind(&JointStateToTrajectoryNode::jointCommandCallback, this, std::placeholders::_1));
        current_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointStateToTrajectoryNode::jointStateCallback, this, std::placeholders::_1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/path_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(dt_)),
            std::bind(&JointStateToTrajectoryNode::tick, this));
    }

private:
    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_cmd_ = *msg;
        has_cmd_ = true;
    }
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_state_ = *msg;
        has_state_ = true;
    }
    void tick()
    {
        if (!has_cmd_ || !has_state_)
            return;
        // Integrate velocity to get next position
        std::vector<double> next_pos = last_state_.position;
        std::vector<double> velocities = last_cmd_.velocity;
        if (next_pos.size() != velocities.size())
            return;
        for (size_t i = 0; i < next_pos.size(); ++i)
        {
            next_pos[i] += velocities[i] * dt_;
        }
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = last_cmd_.name;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = next_pos;
        pt.velocities = velocities;
        pt.time_from_start = rclcpp::Duration::from_seconds(dt_);
        traj_msg.points.push_back(pt);
        traj_pub_->publish(traj_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState last_cmd_;
    sensor_msgs::msg::JointState last_state_;
    bool has_cmd_ = false;
    bool has_state_ = false;
    double rate_hz_;
    double dt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateToTrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
