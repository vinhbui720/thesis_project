#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Tesseract Includes
#include <tesseract_environment/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_scene_graph/scene_state.h>

class OnlineCollisionDebugger : public rclcpp::Node
{
public:
    OnlineCollisionDebugger() : Node("online_collision_debugger")
    {
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");

        std::string urdf_xml, srdf_xml;
        this->get_parameter("robot_description", urdf_xml);
        this->get_parameter("robot_description_semantic", srdf_xml);

        if (urdf_xml.empty() || srdf_xml.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot_description or robot_description_semantic! Cannot start.");
            return;
        }

        env_ = std::make_shared<tesseract_environment::Environment>();
        auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

        bool success = env_->init(urdf_xml, srdf_xml, locator);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Tesseract Environment failed to initialize from URDF/SRDF.");
            return;
        }

        threshold_ = 0.1; // 10 cm threshold

        // ==========================================
        // OPTIMIZATION: Initialize Heavy Objects Once
        // ==========================================
        contact_manager_ = env_->getDiscreteContactManager();
        contact_manager_->setActiveCollisionObjects(env_->getActiveLinkNames());
        contact_manager_->setDefaultCollisionMargin(threshold_);

        contact_request_ = tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::ALL);
        contact_request_.calculate_distance = true;
        // ==========================================

        RCLCPP_INFO(this->get_logger(), "Tesseract Environment Initialized Successfully!");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&OnlineCollisionDebugger::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Online Collision Debugger Started!");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!env_ || !contact_manager_)
            return;

        // 1. Update the environment's internal mathematical state with new joint angles
        Eigen::VectorXd joint_positions = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
        env_->setState(msg->name, joint_positions);

        // 2. Extract the SceneState snapshot and update transforms
        tesseract_scene_graph::SceneState current_state = env_->getState();
        contact_manager_->setCollisionObjectsTransform(current_state.link_transforms);

        // 3. Run the math
        tesseract_collision::ContactResultMap contact_results;
        contact_manager_->contactTest(contact_results, contact_request_);

        // 4. Prepare the Marker Array
        visualization_msgs::msg::MarkerArray marker_array;

        // Pre-allocate memory (we multiply by 2 because we add a Line AND Text for each collision)
        marker_array.markers.reserve((contact_results.size() * 2) + 1);

        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        int id_counter = 0;

        // Extract the Allowed Collision Matrix (SRDF Rules) from the environment
        auto acm = env_->getAllowedCollisionMatrix();

        // 5. Create markers for all collisions found
        for (const auto &pair : contact_results)
        {
            for (const auto &result : pair.second)
            {
                // --- FIX: Check if SRDF explicitly disabled collisions for this pair ---
                if (acm && acm->isCollisionAllowed(result.link_names[0], result.link_names[1]))
                {
                    continue; // Skip this pair! SRDF says they are allowed to touch.
                }

                if (result.distance < threshold_)
                {
                    // --- MATH: Calculate Color based on distance ---
                    // Ratio: 1.0 = at threshold (Safe-ish), 0.0 = hitting object (Danger!)
                    double ratio = std::max(0.0, result.distance / threshold_);

                    // --- MARKER 1: THE LINE ---
                    visualization_msgs::msg::Marker line_marker;
                    line_marker.header.frame_id = "world";
                    line_marker.header.stamp = this->now();
                    line_marker.ns = "collision_distances";
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

                    // Add the two points to create a single line segment
                    line_marker.points.push_back(p1);
                    line_marker.points.push_back(p2);

                    // For LINE_STRIP, only scale.x is used (it defines the line width/thickness)
                    line_marker.scale.x = 0.005; // 5mm thick line

                    // Apply the LERP Color Math
                    line_marker.color.r = 1.0 - ratio; // Gets redder as it gets closer
                    line_marker.color.g = ratio;       // Gets greener as it gets further
                    line_marker.color.b = 0.0;
                    line_marker.color.a = 1.0;

                    marker_array.markers.push_back(line_marker);

                    // --- MARKER 2: THE TEXT DISTANCE ---
                    visualization_msgs::msg::Marker text_marker;
                    text_marker.header.frame_id = "world";
                    text_marker.header.stamp = this->now();
                    text_marker.ns = "collision_text";
                    text_marker.id = id_counter++;
                    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    text_marker.action = visualization_msgs::msg::Marker::ADD;

                    // Position text exactly in the middle of the line, slightly raised
                    text_marker.pose.position.x = (p1.x + p2.x) / 2.0;
                    text_marker.pose.position.y = (p1.y + p2.y) / 2.0;
                    text_marker.pose.position.z = (p1.z + p2.z) / 2.0 + 0.02;

                    text_marker.scale.z = 0.015;           // Text height in meters
                    text_marker.color = line_marker.color; // Match the text color to the line color

                    // --- FIX: Format the text to exact user specification ---
                    char text_buffer[256];
                    snprintf(text_buffer, sizeof(text_buffer), "%s <-> %s\n(%.2f cm)",
                             result.link_names[0].c_str(),
                             result.link_names[1].c_str(),
                             result.distance * 100.0);

                    text_marker.text = text_buffer;

                    marker_array.markers.push_back(text_marker);
                }
            }
        }

        // 6. Publish to RViz
        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tesseract_environment::Environment> env_;

    tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
    tesseract_collision::ContactRequest contact_request_;
    double threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCollisionDebugger>());
    rclcpp::shutdown();
    return 0;
}