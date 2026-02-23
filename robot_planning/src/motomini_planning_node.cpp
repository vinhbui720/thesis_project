/**
 * @file motomini_planning_node.cpp
 * @brief ROS 2 Node to wrap the MotoMiniPlanning logic (Advanced Version)
 * @details Features: Accumulates poses, supports buffer clearing, optimized logging.
 * @author Bùi Quang Vinh
 */

// 1. Include our Planning Logic
#include <robot_planning/motomini_planning.h>

// Include for data types
#include <tesseract_common/types.h>
#include <tesseract_common/joint_state.h>

// 2. ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// 3. Tesseract Includes
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>

// 4. Utils
#include <Eigen/Geometry>
#include <vector>

using namespace Vinhtesseract_examples;

class MotoMiniPlanningNode : public rclcpp::Node
{
public:
    MotoMiniPlanningNode() : Node("motomini_planning_node")
    {
        // --- PARAMETERS ---
        this->declare_parameter("urdf_path", "package://robot_model/urdf/motoman_motomini.urdf");
        this->declare_parameter("srdf_path", "package://robot_model/urdf/motoman_motomini.srdf");

        // --- INITIALIZE TESSERACT ENVIRONMENT ---
        if (!initializeEnvironment())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract Environment. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // --- INITIALIZE PLANNER ---
        planner_ = std::make_shared<MotoMiniPlanning>(env_, nullptr, true, true);

        // --- SUBSCRIBERS ---

        // 1. Input: /joint_states
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&MotoMiniPlanningNode::jointStateCallback, this, std::placeholders::_1));

        // 2. Input: /target_poses (ACCUMULATIVE)
        // Now appends new poses to the list instead of overwriting
        sub_targets_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/target_poses", 10, std::bind(&MotoMiniPlanningNode::targetPosesCallback, this, std::placeholders::_1));

        // 3. Input: /start (Triggers Planning)
        sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start", 10, std::bind(&MotoMiniPlanningNode::startCallback, this, std::placeholders::_1));

        // 4. Input: /clear_targets (New: Clears the buffer)
        sub_clear_ = this->create_subscription<std_msgs::msg::Bool>(
            "/clear_targets", 10, std::bind(&MotoMiniPlanningNode::clearCallback, this, std::placeholders::_1));

        // --- PUBLISHERS ---
        pub_status_ = this->create_publisher<std_msgs::msg::String>("/optimization_status", 10);
        pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);

        RCLCPP_INFO(this->get_logger(), "MotoMini Planning Node Ready (Advanced).");
        RCLCPP_INFO(this->get_logger(), "Topics: /joint_states, /target_poses (accumulates), /clear_targets, /start");
    }

private:
    // --- MEMBERS ---
    std::shared_ptr<tesseract_environment::Environment> env_;
    std::shared_ptr<MotoMiniPlanning> planner_;

    // Data Cache
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::vector<geometry_msgs::msg::Pose> accumulated_targets_; // OPTIMIZATION: Vector to store all poses

    // ROS Interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_targets_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_clear_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;

    // --- INITIALIZATION HELPER ---
    bool initializeEnvironment()
    {
        auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

        std::string urdf_str = this->get_parameter("urdf_path").as_string();
        std::string srdf_str = this->get_parameter("srdf_path").as_string();

        auto urdf_res = locator->locateResource(urdf_str);
        auto srdf_res = locator->locateResource(srdf_str);

        if (!urdf_res || !srdf_res)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not locate URDF or SRDF resource.");
            return false;
        }

        std::filesystem::path urdf_path = urdf_res->getFilePath();
        std::filesystem::path srdf_path = srdf_res->getFilePath();

        env_ = std::make_shared<tesseract_environment::Environment>();
        if (!env_->init(urdf_path, srdf_path, locator))
            return false;

        return true;
    }

    // --- CALLBACKS ---

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void targetPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;

        // Optimization: Reserve memory if adding a large batch to prevent reallocations
        size_t new_size = accumulated_targets_.size() + msg->poses.size();
        accumulated_targets_.reserve(new_size);

        // Accumulate poses
        accumulated_targets_.insert(accumulated_targets_.end(), msg->poses.begin(), msg->poses.end());

        RCLCPP_INFO(this->get_logger(), "Received %zu poses. Total accumulated: %zu",
                    msg->poses.size(), accumulated_targets_.size());

        publishStatus("Accumulating Poses: " + std::to_string(accumulated_targets_.size()));
    }

    void clearCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            accumulated_targets_.clear();
            RCLCPP_INFO(this->get_logger(), "Target buffer cleared.");
            publishStatus("Buffer Cleared");
        }
    }

    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;

        RCLCPP_INFO(this->get_logger(), "Start signal received!");

        // 1. Validation
        if (!last_joint_state_)
        {
            RCLCPP_WARN(this->get_logger(), "Aborting: No joint states received yet.");
            publishStatus("Failed: No Joint States");
            return;
        }
        if (accumulated_targets_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Aborting: No targets accumulated.");
            publishStatus("Failed: No Targets");
            return;
        }

        publishStatus("Planning Started...");

        // 2. Update Environment
        std::vector<std::string> joint_names = last_joint_state_->name;
        Eigen::VectorXd joint_pos(last_joint_state_->position.size());
        for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
        {
            joint_pos[i] = last_joint_state_->position[i];
        }
        env_->setState(joint_names, joint_pos);

        // 3. Convert Poses (Using accumulated list)
        std::vector<Eigen::Isometry3d> eigen_poses;
        eigen_poses.reserve(accumulated_targets_.size()); // Optimization: Reserve memory

        for (const auto &ros_pose : accumulated_targets_)
        {
            Eigen::Isometry3d eigen_pose;
            Eigen::Vector3d t(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
            Eigen::Quaterniond q(ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z);
            eigen_pose = Eigen::Translation3d(t) * q;
            eigen_poses.push_back(eigen_pose);
        }

        // 4. Run Planning
        planner_->setTargetPoses(eigen_poses);
        bool success = planner_->run();

        // 5. Result
        if (success)
        {
            publishStatus("Success");
            auto traj_ptr = planner_->getTrajectory();
            if (traj_ptr)
            {
                publishTrajectory(*traj_ptr, joint_names);

                RCLCPP_INFO(this->get_logger(), "Planning Done. Buffer still holds %zu poses. Send /clear_targets to reset.", accumulated_targets_.size());
            }
        }
        else
        {
            publishStatus("Failed: Optimization Error");
        }
    }

    void publishStatus(const std::string &status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        pub_status_->publish(msg);
    }

    void publishTrajectory(const tesseract_common::JointTrajectory &tess_traj,
                           const std::vector<std::string> &joint_names)
    {
        trajectory_msgs::msg::JointTrajectory ros_msg;
        ros_msg.header.stamp = this->now();
        ros_msg.header.frame_id = "world";
        ros_msg.joint_names = joint_names;

        for (const auto &state : tess_traj)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(state.time);

            for (double val : state.position)
                point.positions.push_back(val);
            for (double val : state.velocity)
                point.velocities.push_back(val);
            for (double val : state.acceleration)
                point.accelerations.push_back(val);

            ros_msg.points.push_back(point);
        }

        pub_trajectory_->publish(ros_msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory published with %zu points.", ros_msg.points.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotoMiniPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}