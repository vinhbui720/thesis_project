#include <robot_planning/gantry_planning.h>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

#include <Eigen/Geometry>

using namespace Vinhtesseract_examples;

class GantryPlanningNode : public rclcpp::Node
{
public:
    GantryPlanningNode() : Node("gantry_planning_node")
    {
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<std::string>("manipulator_group", "gantry");
        this->declare_parameter<std::string>("base_link", "world");
        this->declare_parameter<std::string>("ee_link", "gantry_tool_link");
        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<bool>("use_obstacles", false);

        const bool debug = this->get_parameter("debug").as_bool();
        const bool use_obstacles = this->get_parameter("use_obstacles").as_bool();
        const std::string manipulator_group = this->get_parameter("manipulator_group").as_string();
        const std::string base_link = this->get_parameter("base_link").as_string();
        const std::string ee_link = this->get_parameter("ee_link").as_string();
        this->get_parameter("robot_description", urdf_xml_);
        this->get_parameter("robot_description_semantic", srdf_xml_);

        if (!initializeEnvironment())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize gantry environment.");
            throw std::runtime_error("Gantry Tesseract init failed");
        }

        planner_ = std::make_shared<GantryPlanning>(env_, plotter_, manipulator_group, base_link, ee_link, debug, use_obstacles);

        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20, std::bind(&GantryPlanningNode::jointStateCallback, this, std::placeholders::_1));

        sub_targets_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gantry/target_poses", 10, std::bind(&GantryPlanningNode::targetPosesCallback, this, std::placeholders::_1));

        sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
            "/gantry/start", 10, std::bind(&GantryPlanningNode::startCallback, this, std::placeholders::_1));

        sub_clear_ = this->create_subscription<std_msgs::msg::Bool>(
            "/gantry/clear_targets", 10, std::bind(&GantryPlanningNode::clearCallback, this, std::placeholders::_1));

        pub_status_ = this->create_publisher<std_msgs::msg::String>("/gantry/optimization_status", 10);
        pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/gantry/joint_path_command", 10);

        RCLCPP_INFO(this->get_logger(), "Gantry planning node ready.");
    }

private:
    bool initializeEnvironment()
    {
        if (urdf_xml_.empty() || srdf_xml_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "URDF/SRDF parameter is empty for gantry planner.");
            return false;
        }

        auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        env_ = std::make_shared<tesseract_environment::Environment>();

        if (!env_->init(urdf_xml_, srdf_xml_, locator))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract environment for gantry planner.");
            return false;
        }

        plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
        return true;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void targetPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;

        accumulated_targets_.insert(accumulated_targets_.end(), msg->poses.begin(), msg->poses.end());

        publishStatus("Gantry accumulating poses: " + std::to_string(accumulated_targets_.size()));
    }

    void clearCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;

        accumulated_targets_.clear();
        publishStatus("Gantry target buffer cleared");
    }

    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;

        if (!last_joint_state_)
        {
            publishStatus("Gantry failed: no joint states");
            RCLCPP_WARN(this->get_logger(), "No /joint_states received yet for gantry planner.");
            return;
        }

        if (accumulated_targets_.empty())
        {
            publishStatus("Gantry failed: no target poses");
            RCLCPP_WARN(this->get_logger(), "No gantry target poses to plan.");
            return;
        }

        std::vector<std::string> joint_names = last_joint_state_->name;
        Eigen::VectorXd joint_pos(last_joint_state_->position.size());
        for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
            joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[i];

        planner_->updateEnvironmentState(joint_names, joint_pos);

        std::vector<Eigen::Isometry3d> eigen_poses;
        eigen_poses.reserve(accumulated_targets_.size());

        for (const auto &ros_pose : accumulated_targets_)
        {
            Eigen::Vector3d t(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
            Eigen::Quaterniond q(ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z);
            Eigen::Isometry3d pose = Eigen::Translation3d(t) * q;
            eigen_poses.push_back(pose);
        }

        planner_->setTargetPoses(eigen_poses);
        publishStatus("Gantry planning started");

        const bool success = planner_->run();
        if (!success)
        {
            publishStatus("Gantry planning failed");
            return;
        }

        auto traj_ptr = planner_->getTrajectory();
        if (!traj_ptr || traj_ptr->empty())
        {
            publishStatus("Gantry planning failed: empty trajectory");
            return;
        }

        publishTrajectory(*traj_ptr);
        publishStatus("Gantry planning success");
    }

    void publishTrajectory(const tesseract_common::JointTrajectory &tess_traj)
    {
        const std::vector<std::string> controlled_joints = {"joint_x", "joint_z"};

        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";
        msg.joint_names = controlled_joints;

        for (const auto &state : tess_traj)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(state.time);

            for (size_t i = 0; i < controlled_joints.size(); ++i)
            {
                if (i < static_cast<size_t>(state.position.size()))
                    point.positions.push_back(state.position[static_cast<Eigen::Index>(i)]);
                if (i < static_cast<size_t>(state.velocity.size()))
                    point.velocities.push_back(state.velocity[static_cast<Eigen::Index>(i)]);
                if (i < static_cast<size_t>(state.acceleration.size()))
                    point.accelerations.push_back(state.acceleration[static_cast<Eigen::Index>(i)]);
            }

            msg.points.push_back(point);
        }

        pub_trajectory_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published gantry trajectory (%zu points).", msg.points.size());
    }

    void publishStatus(const std::string &status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        pub_status_->publish(msg);
    }

    std::string urdf_xml_;
    std::string srdf_xml_;

    std::shared_ptr<tesseract_environment::Environment> env_;
    std::shared_ptr<tesseract_visualization::Visualization> plotter_;
    std::shared_ptr<GantryPlanning> planner_;

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::vector<geometry_msgs::msg::Pose> accumulated_targets_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_targets_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_clear_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GantryPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
