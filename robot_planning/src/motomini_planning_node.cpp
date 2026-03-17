/**
 * @file motomini_planning_node.cpp
 * @brief ROS 2 Node to wrap the MotoMiniPlanning logic (Advanced Version)
 * @details Features: Accumulates poses, supports buffer clearing, optimized logging, closed-loop execution monitoring.
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
#include <algorithm>
#include <chrono>
#include <cmath> // Added for std::abs()

using namespace Vinhtesseract_examples;

class MotoMiniPlanningNode : public rclcpp::Node
{
public:
    MotoMiniPlanningNode() : Node("motomini_planning_node")
    {

        // --- PARAMETERS ---
        this->declare_parameter<std::string>("robot_description", "package://robot_planning/urdf/motoman_motomini.urdf");
        this->declare_parameter<std::string>("robot_description_semantic", "package://robot_planning/urdf/motoman_motomini.srdf");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");
        this->declare_parameter<std::string>("base_link", "world");
        this->declare_parameter<std::string>("ee_link", "tool0");
        std::string manipulator_group = this->get_parameter("manipulator_group").as_string();
        std::string base_link = this->get_parameter("base_link").as_string();
        std::string ee_link = this->get_parameter("ee_link").as_string();
        this->get_parameter("robot_description", urdf_xml_);
        this->get_parameter("robot_description_semantic", srdf_xml_);

        // --- INITIALIZE TESSERACT ENVIRONMENT ---
        if (!initializeEnvironment())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize Tesseract Environment.");
            throw std::runtime_error("Tesseract init failed");
        }

        // --- INITIALIZE PLANNER ---
        planner_ = std::make_shared<MotoMiniPlanning>(
            env_,
            nullptr,
            manipulator_group,
            base_link,
            ee_link,
            false,
            true,
            false);

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

        // Timer for closed-loop execution monitoring (10 Hz)
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotoMiniPlanningNode::monitorExecution, this));

        RCLCPP_INFO(this->get_logger(), "MotoMini Planning Node Ready (Advanced).");
        RCLCPP_INFO(this->get_logger(), "Topics: /joint_states, /target_poses (accumulates), /clear_targets, /start");
    }

private:
    std::string urdf_xml_;
    std::string srdf_xml_;

    // --- MEMBERS ---
    std::shared_ptr<tesseract_environment::Environment> env_;
    std::shared_ptr<MotoMiniPlanning> planner_;

    // Data Cache
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::vector<geometry_msgs::msg::Pose> accumulated_targets_;

    // ROS Interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_targets_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_clear_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;

    // --- EXECUTION MONITORING ---
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    bool is_executing_ = false;
    std::vector<double> final_joint_target_;
    std::vector<std::string> target_joint_names_;
    rclcpp::Time execution_start_time_;
    double expected_execution_duration_ = 0.0;

    const double JOINT_TOLERANCE = 0.05; // radians (approx 2.8 degrees tolerance)
    const double TIMEOUT_BUFFER = 5.0;   // seconds

    bool initializeEnvironment()
    {
        if (urdf_xml_.empty() || srdf_xml_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "URDF or SRDF parameter is empty.");
            return false;
        }

        auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

        env_ = std::make_shared<tesseract_environment::Environment>();

        if (!env_->init(urdf_xml_, srdf_xml_, locator))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract environment.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Tesseract environment initialized successfully.");
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
            is_executing_ = false; // Cancel execution monitoring if cleared manually
            RCLCPP_INFO(this->get_logger(), "Target buffer cleared.");
            publishStatus("Buffer Cleared");
        }
    }

    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;

        if (is_executing_)
        {
            RCLCPP_WARN(this->get_logger(), "Already executing a trajectory. Ignoring start signal.");
            return;
        }

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
            auto traj_ptr = planner_->getTrajectory();
            if (traj_ptr && !traj_ptr->empty())
            {
                publishStatus("Optimization Success. Executing...");

                // Publish physical path command to the robot
                publishTrajectory(*traj_ptr, joint_names);

                // Setup variables for Closed-Loop Monitoring
                target_joint_names_ = joint_names;

                // Convert Eigen::VectorXd to std::vector<double>
                Eigen::VectorXd eigen_final_pos = traj_ptr->back().position;
                final_joint_target_.assign(eigen_final_pos.data(), eigen_final_pos.data() + eigen_final_pos.size());

                expected_execution_duration_ = traj_ptr->back().time;
                execution_start_time_ = this->now();
                is_executing_ = true;

                RCLCPP_INFO(this->get_logger(), "Trajectory sent. Expected duration: %.2f seconds. Monitoring joints...", expected_execution_duration_);
            }
            else
            {
                publishStatus("Failed: Trajectory Empty");
            }
        }
        else
        {
            publishStatus("Failed: Optimization Error");
        }
    }

    void monitorExecution()
    {
        if (!is_executing_ || !last_joint_state_)
            return;

        // 1. Check Timeout Condition
        double elapsed = (this->now() - execution_start_time_).seconds();
        if (elapsed > (expected_execution_duration_ + TIMEOUT_BUFFER))
        {
            is_executing_ = false;
            publishStatus("Failed: Execution Timeout");
            RCLCPP_ERROR(this->get_logger(), "Robot did not reach target within expected time + buffer.");
            return;
        }

        // 2. Check Joint Errors (Compare current to target)
        double max_error = 0.0;
        for (size_t i = 0; i < target_joint_names_.size(); ++i)
        {
            // Find index of the joint in the last_joint_state_ message safely
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), target_joint_names_[i]);
            if (it != last_joint_state_->name.end())
            {
                size_t idx = std::distance(last_joint_state_->name.begin(), it);
                double current_pos = last_joint_state_->position[idx];
                double error = std::abs(current_pos - final_joint_target_[i]);
                if (error > max_error)
                {
                    max_error = error;
                }
            }
        }

        // 3. Verify if within tolerance threshold
        if (max_error < JOINT_TOLERANCE)
        {
            is_executing_ = false;
            publishStatus("Success"); // This finally releases your main_command_node to do the next task!
            RCLCPP_INFO(this->get_logger(), "Robot successfully reached physical target! (Max Error: %.4f rad)", max_error);
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