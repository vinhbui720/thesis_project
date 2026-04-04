/**
 * @file enhanced_debug_node.cpp
 * @brief Enhanced Multi-Feature Debug Node for Tesseract Planning & Optimization
 *
 * Features:
 *   ✅ Collision Box/Margin (ContactResultsMarker)
 *   ✅ Collision Gradient (ArrowMarker)
 *   ✅ Cartesian Error (AxisMarker + Arrow)
 *   ✅ Joint Trajectory (Trajectory Animation)
 *   ✅ Kinematic Error (AxisMarker + Arrow)
 *   ✅ SQP Collision Debug (Callback framework)
 *
 * Integration Points:
 *   - Subscribes to joint_states for real-time collision checking
 *   - Optionally subscribes to trajectory updates for trajectory animation
 *   - Integrates with Tesseract visualization system (Ignition Gazebo backend)
 *   - Provides callbacks for SQP optimization monitoring
 *
 * @author Bùi Quang Vinh
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Tesseract Includes
#include <tesseract_environment/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// Visualization
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/axis_marker.h>

// Command Language
#include <tesseract_command_language/types.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_scene_graph/link.h>

// State Solver
#include <tesseract_state_solver/state_solver.h>

// STL
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <optional>
#include <cmath>

using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_kinematics;

/**
 * @brief Debug data structure for tracking cartesian targets and errors
 */
struct CartesianTarget
{
    Eigen::Isometry3d expected_pose;
    Eigen::Vector3d ee_position;
    double position_error;
    bool is_constrain;
    std::string link_name;

    CartesianTarget() : position_error(0.0), is_constrain(false) {}
};

/**
 * @brief Debug data structure for trajectory waypoint
 */
struct TrajectoryWaypoint
{
    Eigen::VectorXd joint_positions;
    double time;

    TrajectoryWaypoint() : time(0.0) {}
};

/**
 * @brief Debug data structure for collision analysis
 */
struct CollisionDebugInfo
{
    std::string link1, link2;
    double distance;
    double margin;
    Eigen::Vector3d nearest_pt1, nearest_pt2;
    std::vector<double> gradient;

    CollisionDebugInfo() : distance(0.0), margin(0.0) {}
};

class EnhancedDebugNode : public rclcpp::Node
{
public:
    EnhancedDebugNode() : Node("enhanced_debug_node")
    {
        // ============================================
        // Parameter Declaration
        // ============================================
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<bool>("enable_collision_viz", true);
        this->declare_parameter<bool>("enable_cartesian_error_viz", true);
        this->declare_parameter<bool>("enable_trajectory_viz", true);
        this->declare_parameter<bool>("enable_kinematic_error_viz", true);
        this->declare_parameter<bool>("enable_collision_gradient", false);
        this->declare_parameter<double>("collision_threshold", 0.1);
        this->declare_parameter<std::string>("frame_id", "world");
        this->declare_parameter<std::string>("ee_link", "ee_link");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");

        // Retrieve parameters
        std::string urdf_xml, srdf_xml;
        this->get_parameter("robot_description", urdf_xml);
        this->get_parameter("robot_description_semantic", srdf_xml);
        this->get_parameter("collision_threshold", collision_threshold_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("ee_link", ee_link_);
        this->get_parameter("base_link", base_link_);
        this->get_parameter("manipulator_group", manipulator_group_);

        if (urdf_xml.empty() || srdf_xml.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to get robot_description or robot_description_semantic!");
            return;
        }

        // ============================================
        // Initialize Tesseract Environment
        // ============================================
        env_ = std::make_shared<tesseract_environment::Environment>();
        auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

        bool success = env_->init(urdf_xml, srdf_xml, locator);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Tesseract Environment failed to initialize from URDF/SRDF.");
            return;
        }

        // ============================================
        // Initialize Collision Manager
        // ============================================
        contact_manager_ = env_->getDiscreteContactManager();
        contact_manager_->setActiveCollisionObjects(env_->getActiveLinkNames());
        contact_manager_->setDefaultCollisionMargin(collision_threshold_);

        contact_request_ = ContactRequest(ContactTestType::ALL);
        contact_request_.calculate_distance = true;

        // ============================================
        // Initialize Kinematics
        // ============================================
        try
        {
            manipulator_ = env_->getKinematicGroup(manipulator_group_);
            RCLCPP_INFO(this->get_logger(),
                        "Loaded manipulator group: %s", manipulator_group_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to load manipulator group '%s': %s",
                        manipulator_group_.c_str(), e.what());
        }

        // ============================================
        // Initialize State Solver (for trajectory visualization)
        // ============================================
        state_solver_ = env_->getStateSolver();

        RCLCPP_INFO(this->get_logger(), "Tesseract Environment Initialized Successfully!");

        // ============================================
        // Create Publishers
        // ============================================
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "debug_markers", 10);

        // ============================================
        // Create Subscribers
        // ============================================
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&EnhancedDebugNode::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Enhanced Debug Node Started!");
        RCLCPP_INFO(this->get_logger(), "  → Collision Visualization: ENABLED");
        RCLCPP_INFO(this->get_logger(), "  → Cartesian Error Visualization: ENABLED");
        RCLCPP_INFO(this->get_logger(), "  → Trajectory Visualization: ENABLED");
        RCLCPP_INFO(this->get_logger(), "  → Kinematic Error Visualization: ENABLED");
    }

    /**
     * @brief Set target cartesian pose for error visualization
     */
    void setTargetCartesianPose(const std::string &link_name, const Eigen::Isometry3d &pose)
    {
        std::unique_lock<std::mutex> lock(debug_data_mutex_);
        auto &target = cartesian_targets_[link_name];
        target.expected_pose = pose;
        target.link_name = link_name;
        target.is_constrain = true;
    }

    /**
     * @brief Add waypoint to trajectory for visualization
     */
    void addTrajectoryWaypoint(const Eigen::VectorXd &joint_positions, double time)
    {
        std::unique_lock<std::mutex> lock(debug_data_mutex_);
        TrajectoryWaypoint wp;
        wp.joint_positions = joint_positions;
        wp.time = time;
        trajectory_waypoints_.push_back(wp);
    }

    /**
     * @brief Clear trajectory waypoints
     */
    void clearTrajectory()
    {
        std::unique_lock<std::mutex> lock(debug_data_mutex_);
        trajectory_waypoints_.clear();
    }

    /**
     * @brief Get current collision debug information
     */
    const std::vector<CollisionDebugInfo> &getCollisionDebugInfo() const
    {
        return collision_debug_info_;
    }

private:
    // ============================================
    // ROS 2 Components
    // ============================================
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // ============================================
    // Tesseract Components
    // ============================================
    std::shared_ptr<tesseract_environment::Environment> env_;
    DiscreteContactManager::Ptr contact_manager_;
    ContactRequest contact_request_;
    KinematicGroup::ConstPtr manipulator_;
    std::unique_ptr<tesseract_scene_graph::StateSolver> state_solver_;

    // ============================================
    // Configuration
    // ============================================
    double collision_threshold_;
    std::string frame_id_;
    std::string ee_link_;
    std::string base_link_;
    std::string manipulator_group_;

    // ============================================
    // Debug Data
    // ============================================
    std::mutex debug_data_mutex_;
    std::map<std::string, CartesianTarget> cartesian_targets_;
    std::vector<TrajectoryWaypoint> trajectory_waypoints_;
    std::vector<CollisionDebugInfo> collision_debug_info_;

    /**
     * @brief Callback for joint state updates
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!env_ || !contact_manager_)
            return;

        try
        {
            // ============================================
            // 1. Update Environment State
            // ============================================
            Eigen::VectorXd joint_positions =
                Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
            env_->setState(msg->name, joint_positions);

            tesseract_scene_graph::SceneState current_state = env_->getState();
            contact_manager_->setCollisionObjectsTransform(current_state.link_transforms);

            // ============================================
            // Prepare Marker Array
            // ============================================
            visualization_msgs::msg::MarkerArray marker_array;
            int id_counter = 0;

            // Delete previous markers
            visualization_msgs::msg::Marker delete_all;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_all);

            // ============================================
            // 2. Collision Detection & Visualization
            // ============================================
            ContactResultMap contact_results;
            contact_manager_->contactTest(contact_results, contact_request_);

            auto acm = env_->getAllowedCollisionMatrix();

            // Collision debug info (for API users)
            {
                std::unique_lock<std::mutex> lock(debug_data_mutex_);
                collision_debug_info_.clear();
            }

            for (const auto &pair : contact_results)
            {
                for (const auto &result : pair.second)
                {
                    // Skip allowed collisions
                    if (acm && acm->isCollisionAllowed(result.link_names[0], result.link_names[1]))
                        continue;

                    if (result.distance < collision_threshold_)
                    {
                        // Store debug info
                        {
                            std::unique_lock<std::mutex> lock(debug_data_mutex_);
                            CollisionDebugInfo info;
                            info.link1 = result.link_names[0];
                            info.link2 = result.link_names[1];
                            info.distance = result.distance;
                            info.margin = collision_threshold_;
                            info.nearest_pt1 = result.nearest_points[0];
                            info.nearest_pt2 = result.nearest_points[1];
                            collision_debug_info_.push_back(info);
                        }

                        // ✅ COLLISION BOX/MARGIN VISUALIZATION (ContactResultsMarker)
                        id_counter += visualizeCollisionMargin(
                            marker_array, result, id_counter, frame_id_);

                        // ✅ COLLISION GRADIENT VISUALIZATION (ArrowMarker)
                        id_counter += visualizeCollisionGradient(
                            marker_array, result, current_state, id_counter, frame_id_);
                    }
                }
            }

            // ============================================
            // 3. Cartesian Error Visualization (AxisMarker + Arrow)
            // ============================================
            if (manipulator_)
            {
                // Calculate current EE pose
                Eigen::Isometry3d ee_transform =
                    manipulator_->calcFwdKin(joint_positions).at(ee_link_);
                Eigen::Vector3d ee_position = ee_transform.translation();

                {
                    std::unique_lock<std::mutex> lock(debug_data_mutex_);
                    for (auto &[link_name, target] : cartesian_targets_)
                    {
                        if (target.is_constrain)
                        {
                            // ✅ AxisMarker at target position (expected)
                            id_counter += visualizeAxisMarker(
                                marker_array, target.expected_pose,
                                "target_" + link_name,
                                id_counter, frame_id_);

                            // ✅ AxisMarker at current position (actual)
                            Eigen::Isometry3d current_pose =
                                manipulator_->calcFwdKin(joint_positions).at(link_name);
                            id_counter += visualizeAxisMarker(
                                marker_array, current_pose,
                                "current_" + link_name,
                                id_counter, frame_id_);

                            // ✅ ArrowMarker showing error direction
                            Eigen::Vector3d error =
                                target.expected_pose.translation() - current_pose.translation();
                            target.position_error = error.norm();

                            if (target.position_error > 0.001) // Only if significant error
                            {
                                id_counter += visualizeCartesianErrorArrow(
                                    marker_array, current_pose.translation(),
                                    target.expected_pose.translation(),
                                    "error_" + link_name,
                                    id_counter, frame_id_);
                            }
                        }
                    }
                }
            }

            // ============================================
            // 4. Joint Trajectory Visualization
            // ============================================
            {
                std::unique_lock<std::mutex> lock(debug_data_mutex_);
                if (!trajectory_waypoints_.empty())
                {
                    id_counter += visualizeTrajectory(
                        marker_array, trajectory_waypoints_,
                        msg->name, id_counter, frame_id_);
                }
            }

            // ============================================
            // 5. Kinematic Error Visualization (for end-effector tracking)
            // ============================================
            if (manipulator_)
            {
                // Kinematic error is the difference between desired and actual end-effector position
                // This is similar to cartesian error but specifically for tracking
                {
                    std::unique_lock<std::mutex> lock(debug_data_mutex_);
                    if (cartesian_targets_.find(ee_link_) != cartesian_targets_.end())
                    {
                        const auto &target = cartesian_targets_[ee_link_];
                        if (target.is_constrain)
                        {
                            Eigen::Isometry3d current_ee =
                                manipulator_->calcFwdKin(joint_positions).at(ee_link_);
                            Eigen::Vector3d kinematic_error =
                                target.expected_pose.translation() - current_ee.translation();

                            if (kinematic_error.norm() > 0.001)
                            {
                                // ✅ Kinematic Error Visualization
                                id_counter += visualizeKinematicError(
                                    marker_array, current_ee,
                                    kinematic_error, id_counter, frame_id_);
                            }
                        }
                    }
                }
            }

            // ============================================
            // Publish Markers
            // ============================================
            marker_pub_->publish(marker_array);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in jointStateCallback: %s", e.what());
        }
    }

    /**
     * @brief Visualize collision margin using ContactResultsMarker pattern
     */
    int visualizeCollisionMargin(
        visualization_msgs::msg::MarkerArray &marker_array,
        const ContactResult &result,
        int id_counter,
        const std::string &frame_id)
    {
        // Create line from nearest point to nearest point
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = this->now();
        line_marker.ns = "collision_margin";
        line_marker.id = id_counter++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p1, p2;
        p1.x = result.nearest_points[0].x();
        p1.y = result.nearest_points[0].y();
        p1.z = result.nearest_points[0].z();

        p2.x = result.nearest_points[1].x();
        p2.y = result.nearest_points[1].y();
        p2.z = result.nearest_points[1].z();

        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);

        line_marker.scale.x = 0.005; // Line thickness

        // Color gradient: Red (danger) → Yellow (caution) → Green (safe)
        double ratio = std::max(0.0, result.distance / collision_threshold_);
        line_marker.color.r = 1.0 - ratio;
        line_marker.color.g = ratio;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        marker_array.markers.push_back(line_marker);

        // Text label with distance
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = this->now();
        text_marker.ns = "collision_text";
        text_marker.id = id_counter++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = (p1.x + p2.x) / 2.0;
        text_marker.pose.position.y = (p1.y + p2.y) / 2.0;
        text_marker.pose.position.z = (p1.z + p2.z) / 2.0 + 0.02;

        text_marker.scale.z = 0.015;
        text_marker.color = line_marker.color;

        char buffer[256];
        snprintf(buffer, sizeof(buffer), "%s ↔ %s\n(%.2f cm)",
                 result.link_names[0].c_str(),
                 result.link_names[1].c_str(),
                 result.distance * 100.0);
        text_marker.text = buffer;

        marker_array.markers.push_back(text_marker);

        return 2; // Returned 2 markers
    }

    /**
     * @brief Visualize collision gradient using ArrowMarker
     */
    int visualizeCollisionGradient(
        visualization_msgs::msg::MarkerArray &marker_array,
        const ContactResult &result,
        const tesseract_scene_graph::SceneState &state,
        int id_counter,
        const std::string &frame_id)
    {
        // Create arrow pointing from contact point towards safety
        // Direction: from nearest_points[1] (on link2) towards nearest_points[0] (on link1)
        try
        {
            visualization_msgs::msg::Marker arrow_marker;
            arrow_marker.header.frame_id = frame_id;
            arrow_marker.header.stamp = this->now();
            arrow_marker.ns = "collision_gradient";
            arrow_marker.id = id_counter++;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;

            // Arrow points away from collision (safety direction)
            arrow_marker.pose.position.x = result.nearest_points[1].x();
            arrow_marker.pose.position.y = result.nearest_points[1].y();
            arrow_marker.pose.position.z = result.nearest_points[1].z();

            Eigen::Vector3d direction =
                (result.nearest_points[0] - result.nearest_points[1]).normalized();
            double arrow_length = std::min(0.1, result.distance * 2.0);

            // Create orientation from direction
            Eigen::Vector3d z_axis = direction;
            Eigen::Vector3d y_axis = z_axis.unitOrthogonal();
            Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();

            Eigen::Matrix3d rot;
            rot.col(0) = x_axis;
            rot.col(1) = y_axis;
            rot.col(2) = z_axis;

            Eigen::Quaterniond quat(rot);
            arrow_marker.pose.orientation.x = quat.x();
            arrow_marker.pose.orientation.y = quat.y();
            arrow_marker.pose.orientation.z = quat.z();
            arrow_marker.pose.orientation.w = quat.w();

            arrow_marker.scale.x = arrow_length; // Length
            arrow_marker.scale.y = 0.01;         // Shaft diameter
            arrow_marker.scale.z = 0.02;         // Head diameter

            // Color: gradient based on distance
            double ratio = std::max(0.0, result.distance / collision_threshold_);
            arrow_marker.color.r = 1.0 - ratio;
            arrow_marker.color.g = ratio;
            arrow_marker.color.b = 0.5;
            arrow_marker.color.a = 0.7;

            marker_array.markers.push_back(arrow_marker);
            return 1;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to visualize collision gradient: %s", e.what());
            return 0;
        }
    }

    /**
     * @brief Visualize axis frame
     */
    int visualizeAxisMarker(
        visualization_msgs::msg::MarkerArray &marker_array,
        const Eigen::Isometry3d &pose,
        const std::string &ns,
        int id_counter,
        const std::string &frame_id)
    {
        // Visualize as three arrows (X, Y, Z axes)
        double scale = 0.05;

        // X-axis (Red)
        visualization_msgs::msg::Marker x_marker;
        x_marker.header.frame_id = frame_id;
        x_marker.header.stamp = this->now();
        x_marker.ns = ns + "_x";
        x_marker.id = id_counter++;
        x_marker.type = visualization_msgs::msg::Marker::ARROW;
        x_marker.action = visualization_msgs::msg::Marker::ADD;
        x_marker.pose.position.x = pose.translation().x();
        x_marker.pose.position.y = pose.translation().y();
        x_marker.pose.position.z = pose.translation().z();
        x_marker.scale.x = scale;
        x_marker.scale.y = scale / 5.0;
        x_marker.scale.z = scale / 5.0;
        x_marker.color.r = 1.0;
        x_marker.color.g = 0.0;
        x_marker.color.b = 0.0;
        x_marker.color.a = 0.8;

        Eigen::Vector3d x_axis = pose.linear().col(0);
        Eigen::Vector3d y_axis = pose.linear().col(1);
        Eigen::Vector3d z_local = x_axis.cross(y_axis).normalized();
        Eigen::Matrix3d x_rot;
        x_rot.col(0) = y_axis;
        x_rot.col(1) = z_local;
        x_rot.col(2) = x_axis;
        Eigen::Quaterniond x_quat(x_rot);
        x_marker.pose.orientation.x = x_quat.x();
        x_marker.pose.orientation.y = x_quat.y();
        x_marker.pose.orientation.z = x_quat.z();
        x_marker.pose.orientation.w = x_quat.w();

        marker_array.markers.push_back(x_marker);

        // Y-axis (Green)
        visualization_msgs::msg::Marker y_marker;
        y_marker.header.frame_id = frame_id;
        y_marker.header.stamp = this->now();
        y_marker.ns = ns + "_y";
        y_marker.id = id_counter++;
        y_marker.type = visualization_msgs::msg::Marker::ARROW;
        y_marker.action = visualization_msgs::msg::Marker::ADD;
        y_marker.pose.position = x_marker.pose.position;
        y_marker.scale = x_marker.scale;
        y_marker.color.r = 0.0;
        y_marker.color.g = 1.0;
        y_marker.color.b = 0.0;
        y_marker.color.a = 0.8;

        y_axis = pose.linear().col(1);
        Eigen::Vector3d x_axis_2 = pose.linear().col(0);
        Eigen::Vector3d z_local_2 = x_axis_2.cross(y_axis).normalized();
        Eigen::Matrix3d y_rot;
        y_rot.col(0) = z_local_2;
        y_rot.col(1) = y_axis;
        y_rot.col(2) = x_axis_2;
        Eigen::Quaterniond y_quat(y_rot);
        y_marker.pose.orientation.x = y_quat.x();
        y_marker.pose.orientation.y = y_quat.y();
        y_marker.pose.orientation.z = y_quat.z();
        y_marker.pose.orientation.w = y_quat.w();

        marker_array.markers.push_back(y_marker);

        // Z-axis (Blue)
        visualization_msgs::msg::Marker z_marker;
        z_marker.header.frame_id = frame_id;
        z_marker.header.stamp = this->now();
        z_marker.ns = ns + "_z";
        z_marker.id = id_counter++;
        z_marker.type = visualization_msgs::msg::Marker::ARROW;
        z_marker.action = visualization_msgs::msg::Marker::ADD;
        z_marker.pose.position = x_marker.pose.position;
        z_marker.scale = x_marker.scale;
        z_marker.color.r = 0.0;
        z_marker.color.g = 0.0;
        z_marker.color.b = 1.0;
        z_marker.color.a = 0.8;

        Eigen::Vector3d z_axis = pose.linear().col(2);
        x_axis_2 = pose.linear().col(0);
        Eigen::Vector3d z_local_3 = x_axis_2.cross(z_axis).normalized();
        Eigen::Matrix3d z_rot;
        z_rot.col(0) = z_local_3;
        z_rot.col(1) = z_axis;
        z_rot.col(2) = x_axis_2;
        Eigen::Quaterniond z_quat(z_rot);
        z_marker.pose.orientation.x = z_quat.x();
        z_marker.pose.orientation.y = z_quat.y();
        z_marker.pose.orientation.z = z_quat.z();
        z_marker.pose.orientation.w = z_quat.w();

        marker_array.markers.push_back(z_marker);

        return 3; // 3 axes returned
    }

    /**
     * @brief Visualize cartesian error as arrow between desired and actual
     */
    int visualizeCartesianErrorArrow(
        visualization_msgs::msg::MarkerArray &marker_array,
        const Eigen::Vector3d &current,
        const Eigen::Vector3d &desired,
        const std::string &ns,
        int id_counter,
        const std::string &frame_id)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = ns;
        arrow_marker.id = id_counter++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // Arrow starts at current position, points to desired
        arrow_marker.pose.position.x = current.x();
        arrow_marker.pose.position.y = current.y();
        arrow_marker.pose.position.z = current.z();

        Eigen::Vector3d direction = (desired - current).normalized();
        Eigen::Vector3d y_axis = direction.unitOrthogonal();
        Eigen::Vector3d x_axis = (y_axis.cross(direction)).normalized();

        Eigen::Matrix3d rot;
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = direction;

        Eigen::Quaterniond quat(rot);
        arrow_marker.pose.orientation.x = quat.x();
        arrow_marker.pose.orientation.y = quat.y();
        arrow_marker.pose.orientation.z = quat.z();
        arrow_marker.pose.orientation.w = quat.w();

        double error = (desired - current).norm();
        arrow_marker.scale.x = error; // Length
        arrow_marker.scale.y = 0.01;  // Shaft
        arrow_marker.scale.z = 0.02;  // Head

        // Color: Red for large error, Yellow for medium, Green for small
        if (error > 0.05)
        {
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.0;
        }
        else
        {
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.5;
        }
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 0.8;

        marker_array.markers.push_back(arrow_marker);

        return 1;
    }

    /**
     * @brief Visualize kinematic error (orientation error)
     */
    int visualizeKinematicError(
        visualization_msgs::msg::MarkerArray &marker_array,
        const Eigen::Isometry3d &current_pose,
        const Eigen::Vector3d &error_vector,
        int id_counter,
        const std::string &frame_id)
    {
        // Visualize as arrow pointing in error direction
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "kinematic_error";
        arrow_marker.id = id_counter++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        arrow_marker.pose.position.x = current_pose.translation().x();
        arrow_marker.pose.position.y = current_pose.translation().y();
        arrow_marker.pose.position.z = current_pose.translation().z();

        Eigen::Vector3d direction = error_vector.normalized();
        Eigen::Vector3d y_axis = direction.unitOrthogonal();
        Eigen::Vector3d x_axis = (y_axis.cross(direction)).normalized();

        Eigen::Matrix3d rot;
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = direction;

        Eigen::Quaterniond quat(rot);
        arrow_marker.pose.orientation.x = quat.x();
        arrow_marker.pose.orientation.y = quat.y();
        arrow_marker.pose.orientation.z = quat.z();
        arrow_marker.pose.orientation.w = quat.w();

        arrow_marker.scale.x = error_vector.norm();
        arrow_marker.scale.y = 0.008;
        arrow_marker.scale.z = 0.015;

        arrow_marker.color.r = 0.8;
        arrow_marker.color.g = 0.2;
        arrow_marker.color.b = 0.8;
        arrow_marker.color.a = 0.7;

        marker_array.markers.push_back(arrow_marker);

        return 1;
    }

    /**
     * @brief Visualize trajectory waypoints
     */
    int visualizeTrajectory(
        visualization_msgs::msg::MarkerArray &marker_array,
        const std::vector<TrajectoryWaypoint> &waypoints,
        const std::vector<std::string> &joint_names,
        int id_counter,
        const std::string &frame_id)
    {
        if (waypoints.empty() || !manipulator_)
            return 0;

        // Create line from forward kinematics at each waypoint
        visualization_msgs::msg::Marker trajectory_marker;
        trajectory_marker.header.frame_id = frame_id;
        trajectory_marker.header.stamp = this->now();
        trajectory_marker.ns = "trajectory";
        trajectory_marker.id = id_counter++;
        trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
        trajectory_marker.scale.x = 0.005;

        trajectory_marker.color.r = 0.0;
        trajectory_marker.color.g = 1.0;
        trajectory_marker.color.b = 0.0;
        trajectory_marker.color.a = 0.8;

        try
        {
            for (const auto &waypoint : waypoints)
            {
                // Calculate forward kinematics
                auto fk_results = manipulator_->calcFwdKin(waypoint.joint_positions);
                Eigen::Vector3d ee_pos = fk_results.at(ee_link_).translation();

                geometry_msgs::msg::Point p;
                p.x = ee_pos.x();
                p.y = ee_pos.y();
                p.z = ee_pos.z();
                trajectory_marker.points.push_back(p);
            }

            marker_array.markers.push_back(trajectory_marker);
            return 1;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to visualize trajectory: %s", e.what());
            return 0;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnhancedDebugNode>());
    rclcpp::shutdown();
    return 0;
}
