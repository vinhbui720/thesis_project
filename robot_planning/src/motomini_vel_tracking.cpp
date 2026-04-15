#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_rosutils/utils.h>

class MotoMiniVelTrackingNode : public rclcpp::Node
{
public:
    MotoMiniVelTrackingNode()
        : rclcpp::Node("motomini_vel_tracking")
    {
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("ee_link", "tool0");
        this->declare_parameter<double>("rate_hz", 50.0);
        this->declare_parameter<double>("cmd_vel_timeout", 0.5);
        this->declare_parameter<double>("theta_d_lim", 3.14);
        this->declare_parameter<double>("w0", 0.1);
        this->declare_parameter<double>("k0", 0.001);

        this->get_parameter("robot_description", urdf_xml_);
        this->get_parameter("robot_description_semantic", srdf_xml_);
        manipulator_group_ = this->get_parameter("manipulator_group").as_string();
        base_link_ = this->get_parameter("base_link").as_string();
        ee_link_ = this->get_parameter("ee_link").as_string();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        theta_d_limit_ = this->get_parameter("theta_d_lim").as_double();
        w0_ = this->get_parameter("w0").as_double();
        k0_ = this->get_parameter("k0").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);

        if (!initializeKinematics())
            throw std::runtime_error("Failed to initialize Tesseract kinematics");

        // Publish JointTrajectory directly to the controller topic
        pub_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/pose_following/cmd_vel", 10,
            std::bind(&MotoMiniVelTrackingNode::cmdVelCallback, this, std::placeholders::_1));

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&MotoMiniVelTrackingNode::jointStateCallback, this, std::placeholders::_1));

        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/start",
            std::bind(&MotoMiniVelTrackingNode::startCallback, this, std::placeholders::_1, std::placeholders::_2));

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/stop",
            std::bind(&MotoMiniVelTrackingNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

        state_ = STATE_IDLE;
        latest_cart_vel_.setZero();
        last_cmd_time_ = this->now();
        t_start_ = this->now();

        auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
        timer_ = this->create_wall_timer(
            period_ns,
            std::bind(&MotoMiniVelTrackingNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Group: %s", manipulator_group_.c_str());
    }

private:
    enum State
    {
        STATE_IDLE,
        STATE_POSE_FOLLOW,
        STATE_STOP
    };

    bool initializeKinematics()
    {
        auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        env_ = std::make_shared<tesseract_environment::Environment>();
        if (!env_->init(urdf_xml_, srdf_xml_, locator))
            return false;

        manip_ = env_->getKinematicGroup(manipulator_group_);
        if (!manip_)
            return false;

        joint_names_ = manip_->getJointNames();
        return !joint_names_.empty();
    }

    static Eigen::MatrixXd calcSrInverse(const Eigen::MatrixXd &J, double w, double w0, double k0)
    {
        double k = (w < w0) ? k0 * std::pow(1.0 - w / w0, 2.0) : 0.0;
        const Eigen::Index rows = J.rows();
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(rows, rows);
        return J.transpose() * (J * J.transpose() + k * I).inverse();
    }

    bool currentManipulatorJointVector(Eigen::VectorXd &q) const
    {
        if (!last_joint_state_)
            return false;
        q.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), joint_names_[i]);
            if (it == last_joint_state_->name.end())
                return false;
            q[i] = last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
        }
        return true;
    }

    // Seed tracked_positions_ from current joint states (called once at start)
    bool initTrackedPositions()
    {
        Eigen::VectorXd q;
        if (!currentManipulatorJointVector(q))
            return false;
        tracked_positions_.assign(q.data(), q.data() + q.size());
        return true;
    }

    void publishTrajectory(const std::vector<double> &positions,
                           const std::vector<double> &velocities,
                           const std::vector<double> &next_positions)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;

        // Point 0: current position with computed velocity at t=0 (motion hint)
        trajectory_msgs::msg::JointTrajectoryPoint pt0;
        pt0.positions = positions;
        pt0.velocities = velocities;
        pt0.time_from_start = rclcpp::Duration::from_seconds(0.0);

        // Point 1: next integrated position with zero velocity at t=dt
        // JointTrajectoryController requires zero velocity on the last point
        trajectory_msgs::msg::JointTrajectoryPoint pt1;
        pt1.positions = next_positions;
        pt1.velocities.assign(joint_names_.size(), 0.0);
        pt1.time_from_start = rclcpp::Duration::from_seconds(dt_);

        traj.points.push_back(pt0);
        traj.points.push_back(pt1);
        pub_traj_->publish(traj);
    }

    void publishStop()
    {
        if (tracked_positions_.empty())
            return;
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishTrajectory(tracked_positions_, zero_vel, tracked_positions_);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_cart_vel_ << msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z;
        last_cmd_time_ = this->now();
        if (state_ == STATE_IDLE)
        {
            // Seed positions from real joint states at transition to active
            if (initTrackedPositions())
            {
                t_start_ = this->now();
                state_ = STATE_POSE_FOLLOW;
            }
        }
    }

    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_IDLE;
        tracked_positions_.clear();
        res->success = true;
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_STOP;
        publishStop();
        res->success = true;
    }

    void tick()
    {
        if (state_ == STATE_IDLE)
            return;

        if (state_ == STATE_STOP)
        {
            publishStop();
            return;
        }

        // Timeout: no new cmd_vel → go idle
        if ((this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_)
        {
            state_ = STATE_IDLE;
            publishStop();
            return;
        }

        if (!last_joint_state_)
            return;

        // Seed tracked positions if not yet initialized
        if (tracked_positions_.empty())
        {
            if (!initTrackedPositions())
                return;
        }

        // Compute Jacobian from current real joint positions
        Eigen::VectorXd q;
        if (!currentManipulatorJointVector(q))
            return;

        Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
        double w = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
        Eigen::VectorXd theta_d = calcSrInverse(J, w, w0_, k0_) * latest_cart_vel_;

        // Joint velocity limit check
        for (int i = 0; i < theta_d.size(); ++i)
        {
            if (std::abs(theta_d[i]) > theta_d_limit_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Joint %d velocity %.3f exceeds limit %.3f — stopping.", i, theta_d[i], theta_d_limit_);
                state_ = STATE_STOP;
                publishStop();
                return;
            }
        }

        // Compute next integrated positions (same as ROS1 node)
        std::vector<double> current_positions = tracked_positions_;
        for (size_t i = 0; i < tracked_positions_.size(); ++i)
            tracked_positions_[i] += theta_d[i] * dt_;

        std::vector<double> velocities(theta_d.data(), theta_d.data() + theta_d.size());
        // Publish: pt0=current+vel, pt1=next+zero_vel (satisfies JointTrajectoryController constraint)
        publishTrajectory(current_positions, velocities, tracked_positions_);
    }

    std::string urdf_xml_, srdf_xml_, manipulator_group_, base_link_, ee_link_;
    std::vector<std::string> joint_names_;
    double rate_hz_, cmd_vel_timeout_, theta_d_limit_, w0_, k0_, dt_;
    State state_;
    Eigen::Matrix<double, 6, 1> latest_cart_vel_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time t_start_;
    tesseract_environment::Environment::Ptr env_;
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;

    std::vector<double> tracked_positions_; // integrated joint positions

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniVelTrackingNode>());
    rclcpp::shutdown();
    return 0;
}