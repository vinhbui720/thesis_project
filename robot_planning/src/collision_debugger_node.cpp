#include <algorithm>
#include <cmath>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
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
        this->declare_parameter<double>("collision_threshold", 0.1);
        this->declare_parameter<double>("repulsive_force_gain", 1.0);
        this->declare_parameter<double>("repulsive_force_max", 5.0);
        this->declare_parameter<double>("force_arrow_scale", 0.05);
        this->declare_parameter<double>("max_force_arrow_length", 0.15);
        this->declare_parameter<double>("contact_point_radius", 0.008);
        this->declare_parameter<std::string>("marker_frame", "world");
        this->declare_parameter<bool>("show_pair_forces", true);
        this->declare_parameter<bool>("publish_contact_debug_text", true);

        std::string urdf_xml, srdf_xml;
        this->get_parameter("robot_description", urdf_xml);
        this->get_parameter("robot_description_semantic", srdf_xml);
        threshold_ = this->get_parameter("collision_threshold").as_double();
        repulsive_force_gain_ = this->get_parameter("repulsive_force_gain").as_double();
        repulsive_force_max_ = this->get_parameter("repulsive_force_max").as_double();
        force_arrow_scale_ = this->get_parameter("force_arrow_scale").as_double();
        max_force_arrow_length_ = this->get_parameter("max_force_arrow_length").as_double();
        contact_point_radius_ = this->get_parameter("contact_point_radius").as_double();
        marker_frame_ = this->get_parameter("marker_frame").as_string();
        show_pair_forces_ = this->get_parameter("show_pair_forces").as_bool();
        publish_contact_debug_text_ = this->get_parameter("publish_contact_debug_text").as_bool();

        threshold_ = std::max(1e-6, threshold_);
        repulsive_force_gain_ = std::max(0.0, repulsive_force_gain_);
        repulsive_force_max_ = std::max(0.0, repulsive_force_max_);
        force_arrow_scale_ = std::max(1e-6, force_arrow_scale_);
        max_force_arrow_length_ = std::max(1e-6, max_force_arrow_length_);
        contact_point_radius_ = std::max(1e-4, contact_point_radius_);

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

        // ==========================================
        // OPTIMIZATION: Initialize Heavy Objects Once
        // ==========================================
        contact_manager_ = env_->getDiscreteContactManager();
        contact_manager_->setActiveCollisionObjects(env_->getActiveLinkNames());
        contact_manager_->setDefaultCollisionMargin(threshold_);

        contact_request_ = tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::ALL);
        contact_request_.calculate_distance = true;
        contact_request_.calculate_penetration = true;
        // ==========================================

        RCLCPP_INFO(this->get_logger(), "Tesseract Environment Initialized Successfully!");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 10);
        contact_debug_pub_ = this->create_publisher<std_msgs::msg::String>("collision_debug_contacts", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&OnlineCollisionDebugger::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Online Collision Debugger Started. threshold=%.3f m, force_gain=%.3f, markers=%s",
                    threshold_, repulsive_force_gain_, marker_pub_->get_topic_name());
    }

private:
    static geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &v)
    {
        geometry_msgs::msg::Point p;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    }

    static std::string vecToString(const Eigen::Vector3d &v)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(5)
           << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
        return ss.str();
    }

    double computeRepulsiveForceMagnitude(double distance) const
    {
        // Debug-only virtual force: zero at the margin, larger as clearance
        // shrinks, and stronger for penetration where distance is negative.
        const double penetration_or_margin = std::max(0.0, threshold_ - distance);
        const double raw_force =
            repulsive_force_gain_ * penetration_or_margin / std::max(1e-9, threshold_);
        return std::clamp(raw_force, 0.0, repulsive_force_max_);
    }

    Eigen::Vector3d contactNormal0To1(const tesseract_collision::ContactResult &result) const
    {
        // Tesseract normal points from link_names[0] to link_names[1], and is
        // the direction to move link_names[1] away from link_names[0].
        Eigen::Vector3d normal = result.normal;
        if (!normal.allFinite() || normal.norm() < 1e-9)
            normal = result.nearest_points[1] - result.nearest_points[0];
        if (!normal.allFinite() || normal.norm() < 1e-9)
            normal = Eigen::Vector3d::UnitZ();
        return normal.normalized();
    }

    bool isToolCollisionPairIgnored(const std::string &link0, const std::string &link1) const
    {
        // The tool on this setup is custom, and its collision relationships are
        // not always fully mirrored in the SRDF loaded by this node. Keep this
        // list narrow and debug-only.
        static const std::pair<const char *, const char *> ignored_pairs[] = {
            {"link_4_r", "magnetic_link"},
            {"link_5_b", "magnetic_link"},
            {"link_6_t", "magnetic_link"},
            {"tool0", "magnetic_link"},
        };

        for (const auto &pair : ignored_pairs)
        {
            const bool direct = (link0 == pair.first && link1 == pair.second);
            const bool reverse = (link0 == pair.second && link1 == pair.first);
            if (direct || reverse)
                return true;
        }
        return false;
    }

    void applyDistanceColor(visualization_msgs::msg::Marker &marker, double distance, double alpha) const
    {
        const double ratio = std::clamp(distance / threshold_, 0.0, 1.0);
        marker.color.r = 1.0 - ratio;
        marker.color.g = ratio;
        marker.color.b = 0.0;
        marker.color.a = alpha;
    }

    void addDistanceLine(visualization_msgs::msg::MarkerArray &marker_array,
                         const tesseract_collision::ContactResult &result,
                         int &id_counter)
    {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = marker_frame_;
        line_marker.header.stamp = this->now();
        line_marker.ns = "collision_distances";
        line_marker.id = id_counter++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.points.push_back(toPoint(result.nearest_points[0]));
        line_marker.points.push_back(toPoint(result.nearest_points[1]));
        line_marker.scale.x = 0.005;
        applyDistanceColor(line_marker, result.distance, 1.0);
        marker_array.markers.push_back(line_marker);
    }

    void addContactPoint(visualization_msgs::msg::MarkerArray &marker_array,
                         const tesseract_collision::ContactResult &result,
                         int link_index,
                         int &id_counter)
    {
        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = marker_frame_;
        point_marker.header.stamp = this->now();
        point_marker.ns = (link_index == 0) ? "collision_point_link0" : "collision_point_link1";
        point_marker.id = id_counter++;
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.pose.position = toPoint(result.nearest_points[static_cast<size_t>(link_index)]);
        point_marker.pose.orientation.w = 1.0;
        point_marker.scale.x = contact_point_radius_;
        point_marker.scale.y = contact_point_radius_;
        point_marker.scale.z = contact_point_radius_;
        if (link_index == 0)
        {
            point_marker.color.r = 0.1;
            point_marker.color.g = 0.4;
            point_marker.color.b = 1.0;
        }
        else
        {
            point_marker.color.r = 1.0;
            point_marker.color.g = 0.4;
            point_marker.color.b = 0.1;
        }
        point_marker.color.a = 0.9;
        marker_array.markers.push_back(point_marker);
    }

    void addForceArrow(visualization_msgs::msg::MarkerArray &marker_array,
                       const Eigen::Vector3d &start,
                       const Eigen::Vector3d &force,
                       const std::string &ns,
                       int &id_counter)
    {
        if (!force.allFinite() || force.norm() < 1e-9)
            return;

        const double arrow_length =
            std::min(max_force_arrow_length_, std::max(0.01, force.norm() * force_arrow_scale_));
        const Eigen::Vector3d direction = force.normalized();
        const Eigen::Vector3d end = start + direction * arrow_length;

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = marker_frame_;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = ns;
        arrow_marker.id = id_counter++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.points.push_back(toPoint(start));
        arrow_marker.points.push_back(toPoint(end));
        arrow_marker.scale.x = 0.01;  // shaft diameter
        arrow_marker.scale.y = 0.025; // head diameter
        arrow_marker.scale.z = 0.04;  // head length
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.65;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.85;
        marker_array.markers.push_back(arrow_marker);
    }

    void addVectorToObstacleArrow(visualization_msgs::msg::MarkerArray &marker_array,
                                  const tesseract_collision::ContactResult &result,
                                  const Eigen::Vector3d &normal_0_to_1,
                                  int &id_counter)
    {
        Eigen::Vector3d start = result.nearest_points[0];
        Eigen::Vector3d end = result.nearest_points[1];

        // If the nearest points collapse during penetration, still draw a
        // visible normal arrow from link0 toward link1 / the obstacle side.
        if (!start.allFinite() || !end.allFinite())
            return;
        if ((end - start).norm() < 1e-6)
        {
            const double fallback_len = std::min(max_force_arrow_length_, std::max(0.02, 0.5 * threshold_));
            end = start + normal_0_to_1 * fallback_len;
        }

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = marker_frame_;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "collision_vector_to_obstacle";
        arrow_marker.id = id_counter++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.points.push_back(toPoint(start));
        arrow_marker.points.push_back(toPoint(end));
        arrow_marker.scale.x = 0.006;
        arrow_marker.scale.y = 0.018;
        arrow_marker.scale.z = 0.03;
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.35;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 0.9;
        marker_array.markers.push_back(arrow_marker);
    }

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
        std_msgs::msg::String debug_msg;
        std::ostringstream debug_stream;

        marker_array.markers.reserve((contact_results.size() * 6) + 1);

        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        int id_counter = 0;
        size_t contact_counter = 0;

        // Extract the Allowed Collision Matrix (SRDF Rules) from the environment
        auto acm = env_->getAllowedCollisionMatrix();

        debug_stream << std::fixed << std::setprecision(6);
        debug_stream << "threshold_m=" << threshold_;

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

                if (isToolCollisionPairIgnored(result.link_names[0], result.link_names[1]))
                {
                    continue; // Custom tool attachment: ignore its known nearby-link contacts in debug.
                }

                if (result.distance >= threshold_)
                    continue;

                const Eigen::Vector3d normal_0_to_1 = contactNormal0To1(result);
                const Eigen::Vector3d clearance_vector =
                    result.nearest_points[1] - result.nearest_points[0];
                const double force_mag = computeRepulsiveForceMagnitude(result.distance);

                // Push-away convention:
                //   normal points link0 -> link1.
                //   link1 force is +normal; link0 force is -normal.
                const Eigen::Vector3d force_on_link0 = -normal_0_to_1 * force_mag;
                const Eigen::Vector3d force_on_link1 = normal_0_to_1 * force_mag;

                addDistanceLine(marker_array, result, id_counter);
                addContactPoint(marker_array, result, 0, id_counter);
                addContactPoint(marker_array, result, 1, id_counter);
                addVectorToObstacleArrow(marker_array, result, normal_0_to_1, id_counter);

                if (show_pair_forces_)
                    addForceArrow(marker_array, result.nearest_points[0], force_on_link0, "collision_push_force_link0", id_counter);
                addForceArrow(marker_array, result.nearest_points[1], force_on_link1, "collision_push_force_link1", id_counter);

                debug_stream << "\ncontact[" << contact_counter++ << "]"
                             << " link0=" << result.link_names[0]
                             << " shape0=" << result.shape_id[0]
                             << " sub0=" << result.subshape_id[0]
                             << " link1=" << result.link_names[1]
                             << " shape1=" << result.shape_id[1]
                             << " sub1=" << result.subshape_id[1]
                             << " dist_m=" << result.distance
                             << " p0=" << vecToString(result.nearest_points[0])
                             << " p1=" << vecToString(result.nearest_points[1])
                             << " v01=" << vecToString(clearance_vector)
                             << " n01=" << vecToString(normal_0_to_1)
                             << " fmag=" << force_mag
                             << " f0=" << vecToString(force_on_link0)
                             << " f1=" << vecToString(force_on_link1);
            }
        }

        debug_stream << "\ncontact_count=" << contact_counter;

        // 6. Publish to RViz
        marker_pub_->publish(marker_array);
        if (publish_contact_debug_text_)
        {
            debug_msg.data = debug_stream.str();
            contact_debug_pub_->publish(debug_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_debug_pub_;
    std::shared_ptr<tesseract_environment::Environment> env_;

    tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
    tesseract_collision::ContactRequest contact_request_;
    double threshold_;
    double repulsive_force_gain_;
    double repulsive_force_max_;
    double force_arrow_scale_;
    double max_force_arrow_length_;
    double contact_point_radius_;
    std::string marker_frame_;
    bool show_pair_forces_;
    bool publish_contact_debug_text_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCollisionDebugger>());
    rclcpp::shutdown();
    return 0;
}
