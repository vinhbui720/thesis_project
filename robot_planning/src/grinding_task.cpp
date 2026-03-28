/**
 * @file grinding_task.cpp
 * @brief Grinding Task Node — transforms trajectory points to tool frame and publishes target poses.
 *
 * Responsibilities:
 *   - Subscribe to trajectory points from external source (/trajectory_points)
 *   - Get current EE pose from Tesseract environment (T_world^ee)
 *   - Transform points: T_world^tool = T_world^ee × T_ee^points
 *   - Publish transformed poses to /target_poses for motion planning
 *
 * Formula: Each point in the trajectory is transformed from EE frame to world frame
 * using the current end-effector pose.
 *
 * @author Bùi Quang Vinh
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/kinematic_group.h>

#include <Eigen/Geometry>
#include <memory>

class GrindingTaskNode : public rclcpp::Node
{
public:
    GrindingTaskNode() : Node("grinding_task_node")
    {
        // ---- Declare Parameters ----
        this->declare_parameter<std::string>("robot_description",
                                             "package://robot_planning/urdf/motoman_motomini.urdf");
        this->declare_parameter<std::string>("robot_description_semantic",
                                             "package://robot_planning/urdf/motoman_motomini.srdf");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");
        this->declare_parameter<std::string>("base_link", "world");
        this->declare_parameter<std::string>("ee_link", "tool0");
        this->declare_parameter<std::string>("trajectory_points_topic", "/trajectory_points");
        this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
        this->declare_parameter<bool>("debug", false);

        // ---- Get Parameters ----
        std::string urdf_xml, srdf_xml, manipulator_group, base_link, ee_link;
        std::string trajectory_points_topic, joint_states_topic;
        bool debug;

        this->get_parameter("robot_description", urdf_xml);
        this->get_parameter("robot_description_semantic", srdf_xml);
        this->get_parameter("manipulator_group", manipulator_group);
        this->get_parameter("base_link", base_link);
        this->get_parameter("ee_link", ee_link);
        this->get_parameter("trajectory_points_topic", trajectory_points_topic);
        this->get_parameter("joint_states_topic", joint_states_topic);
        this->get_parameter("debug", debug);

        manipulator_group_ = manipulator_group;
        base_link_ = base_link;
        ee_link_ = ee_link;
        debug_ = debug;

        // ---- Initialize Tesseract Environment ----
        if (!initializeEnvironment(urdf_xml, srdf_xml))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize Tesseract Environment.");
            throw std::runtime_error("Tesseract init failed");
        }

        // ---- Create Subscribers ----
        sub_trajectory_points_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            trajectory_points_topic, 10,
            std::bind(&GrindingTaskNode::trajectoryPointsCallback, this, std::placeholders::_1));

        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 10,
            std::bind(&GrindingTaskNode::jointStateCallback, this, std::placeholders::_1));

        // ---- Create Publishers ----
        pub_target_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/target_poses", 10);
        pub_status_ = this->create_publisher<std_msgs::msg::String>("/grinding_task/status", 10);

        RCLCPP_INFO(this->get_logger(),
                    "Grinding Task Node initialized.");
        RCLCPP_INFO(this->get_logger(),
                    "Listening to: %s | Publishing to: /target_poses",
                    trajectory_points_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Transform: T_world^tool = T_world^ee × T_ee^points");
        RCLCPP_INFO(this->get_logger(),
                    "EE link: %s | Base link: %s | Manipulator group: %s",
                    ee_link_.c_str(), base_link_.c_str(), manipulator_group_.c_str());
    }

    ~GrindingTaskNode() {}

private:
    // ---- Member Variables ----
    std::shared_ptr<tesseract_environment::Environment> env_;
    std::string manipulator_group_, base_link_, ee_link_;
    bool debug_;

    // Latest joint states
    sensor_msgs::msg::JointState latest_joint_state_;
    bool joint_state_received_ = false;

    // Subscribers & Publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_trajectory_points_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_target_poses_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

    // ---- Initialize Tesseract Environment ----
    bool initializeEnvironment(const std::string &urdf_xml, const std::string &srdf_xml)
    {
        if (urdf_xml.empty() || srdf_xml.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "URDF or SRDF parameter is empty.");
            return false;
        }

        auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        env_ = std::make_shared<tesseract_environment::Environment>();

        if (!env_->init(urdf_xml, srdf_xml, locator))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract environment.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Tesseract environment initialized successfully.");
        return true;
    }

    // ---- Joint State Callback ----
    void jointStateCallback(const sensor_msgs::msg::JointState &msg)
    {
        latest_joint_state_ = msg;
        joint_state_received_ = true;
    }

    // ---- Trajectory Points Callback ----
    void trajectoryPointsCallback(const geometry_msgs::msg::PoseArray &msg)
    {
        if (!joint_state_received_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Trajectory points received but joint states not yet available. Skipping.");
            return;
        }

        // Get current EE pose (T_world^ee)
        Eigen::Isometry3d T_world_ee = getCurrentEEPose();
        if (!T_world_ee.matrix().allFinite())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute current EE pose.");
            return;
        }

        if (debug_)
        {
            RCLCPP_INFO(this->get_logger(), "Current EE pose (T_world^ee):");
            printTransform(T_world_ee);
        }

        // Transform trajectory points to world frame
        geometry_msgs::msg::PoseArray target_poses;
        target_poses.header.frame_id = msg.header.frame_id; // Use same frame as input
        target_poses.header.stamp = this->now();

        int num_points = msg.poses.size();
        for (size_t i = 0; i < msg.poses.size(); ++i)
        {
            // Convert input pose to Eigen (assumed to be T_ee^points)
            Eigen::Isometry3d T_ee_points = poseToEigen(msg.poses[i]);

            // Compute T_world^tool = T_world^ee × T_ee^points
            Eigen::Isometry3d T_world_tool = T_world_ee * T_ee_points;

            // Convert back to geometry_msgs
            geometry_msgs::msg::Pose tool_pose = eigenToPose(T_world_tool);
            target_poses.poses.push_back(tool_pose);

            if (debug_ && i == 0)
            {
                RCLCPP_INFO(this->get_logger(), "First transformed point:");
                RCLCPP_INFO(this->get_logger(),
                            "  Position: (%.4f, %.4f, %.4f)",
                            tool_pose.position.x,
                            tool_pose.position.y,
                            tool_pose.position.z);
            }
        }

        // Publish transformed poses
        pub_target_poses_->publish(target_poses);

        // Publish status
        std_msgs::msg::String status_msg;
        status_msg.data = "Transformed " + std::to_string(num_points) + " trajectory points";
        pub_status_->publish(status_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published %zu transformed points to /target_poses",
                    target_poses.poses.size());
    }

    // ---- Get Current EE Pose ----
    Eigen::Isometry3d getCurrentEEPose()
    {
        try
        {
            // Update environment state with latest joint values
            std::vector<std::string> joint_names = latest_joint_state_.name;
            Eigen::VectorXd q(latest_joint_state_.position.size());

            for (size_t i = 0; i < latest_joint_state_.position.size(); ++i)
            {
                q(i) = latest_joint_state_.position[i];
            }

            // Update the environment state
            env_->setState(joint_names, q);

            // Get the manipulator
            auto manip = env_->getKinematicGroup(manipulator_group_);
            if (!manip)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to get kinematic group: %s",
                             manipulator_group_.c_str());
                return Eigen::Isometry3d::Identity();
            }

            // Calculate forward kinematics
            auto fk_results = manip->calcFwdKin(q);

            if (fk_results.find(ee_link_) == fk_results.end())
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to get transform for EE link: %s",
                             ee_link_.c_str());
                return Eigen::Isometry3d::Identity();
            }

            return fk_results.at(ee_link_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Exception computing EE pose: %s",
                         e.what());
            return Eigen::Isometry3d::Identity();
        }
    }

    // ---- Convert geometry_msgs::Pose to Eigen::Isometry3d ----
    Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

        // Position
        transform.translation() << pose.position.x, pose.position.y, pose.position.z;

        // Orientation (quaternion)
        Eigen::Quaterniond quat(pose.orientation.w,
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z);
        transform.linear() = quat.toRotationMatrix();

        return transform;
    }

    // ---- Convert Eigen::Isometry3d to geometry_msgs::Pose ----
    geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d &transform)
    {
        geometry_msgs::msg::Pose pose;

        // Position
        pose.position.x = transform.translation()(0);
        pose.position.y = transform.translation()(1);
        pose.position.z = transform.translation()(2);

        // Orientation (quaternion)
        Eigen::Quaterniond quat(transform.linear());
        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();

        return pose;
    }

    // ---- Debug: Print Transform Matrix ----
    void printTransform(const Eigen::Isometry3d &T)
    {
        if (!debug_)
            return;

        RCLCPP_DEBUG(this->get_logger(), "Transform matrix:");
        for (int i = 0; i < 4; ++i)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "  [%.6f, %.6f, %.6f, %.6f]",
                         T.matrix()(i, 0), T.matrix()(i, 1),
                         T.matrix()(i, 2), T.matrix()(i, 3));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GrindingTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
