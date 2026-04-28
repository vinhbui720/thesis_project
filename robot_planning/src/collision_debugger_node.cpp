#include <algorithm>
#include <cmath>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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

        // Contact detection / framing
        this->declare_parameter<double>("collision_threshold", 0.1);
        this->declare_parameter<std::string>("marker_frame", "world");
        this->declare_parameter<double>("contact_point_radius", 0.008);

        // Outputs
        this->declare_parameter<bool>("debug_markers_enabled", true);
        this->declare_parameter<bool>("publish_collision_wrench", true);
        this->declare_parameter<bool>("publish_contact_debug_text", true);

        // Wrench framing
        this->declare_parameter<std::string>("wrench_frame", "world");
        this->declare_parameter<std::string>("wrench_reference_link", "tool0");

        // Force model
        this->declare_parameter<double>("collision_influence_distance", 0.10);
        this->declare_parameter<double>("collision_safe_distance", 0.04);
        this->declare_parameter<double>("collision_k_rep", 0.0005);
        this->declare_parameter<double>("collision_k_hold", 50.0);
        this->declare_parameter<double>("collision_force_max_per_contact", 10.0);
        this->declare_parameter<double>("collision_force_max_total", 25.0);

        // Force-arrow shaping
        this->declare_parameter<double>("force_arrow_min_length", 0.01);
        this->declare_parameter<double>("force_arrow_max_length", 0.25);
        this->declare_parameter<double>("force_arrow_length_gain", 0.04);

        std::string urdf_xml, srdf_xml;
        this->get_parameter("robot_description", urdf_xml);
        this->get_parameter("robot_description_semantic", srdf_xml);

        threshold_ = this->get_parameter("collision_threshold").as_double();
        marker_frame_ = this->get_parameter("marker_frame").as_string();
        contact_point_radius_ = this->get_parameter("contact_point_radius").as_double();

        debug_markers_enabled_ = this->get_parameter("debug_markers_enabled").as_bool();
        publish_collision_wrench_ = this->get_parameter("publish_collision_wrench").as_bool();
        publish_contact_debug_text_ = this->get_parameter("publish_contact_debug_text").as_bool();

        wrench_frame_ = this->get_parameter("wrench_frame").as_string();
        wrench_reference_link_ = this->get_parameter("wrench_reference_link").as_string();

        collision_influence_distance_ = this->get_parameter("collision_influence_distance").as_double();
        collision_safe_distance_ = this->get_parameter("collision_safe_distance").as_double();
        collision_k_rep_ = this->get_parameter("collision_k_rep").as_double();
        collision_k_hold_ = this->get_parameter("collision_k_hold").as_double();
        collision_force_max_per_contact_ = this->get_parameter("collision_force_max_per_contact").as_double();
        collision_force_max_total_ = this->get_parameter("collision_force_max_total").as_double();

        force_arrow_min_length_ = this->get_parameter("force_arrow_min_length").as_double();
        force_arrow_max_length_ = this->get_parameter("force_arrow_max_length").as_double();
        force_arrow_length_gain_ = this->get_parameter("force_arrow_length_gain").as_double();

        threshold_ = std::max(1e-6, threshold_);
        contact_point_radius_ = std::max(1e-4, contact_point_radius_);
        collision_influence_distance_ = std::max(1e-6, collision_influence_distance_);
        collision_safe_distance_ = std::max(0.0, collision_safe_distance_);
        collision_k_rep_ = std::max(0.0, collision_k_rep_);
        collision_k_hold_ = std::max(0.0, collision_k_hold_);
        collision_force_max_per_contact_ = std::max(0.0, collision_force_max_per_contact_);
        collision_force_max_total_ = std::max(0.0, collision_force_max_total_);
        force_arrow_min_length_ = std::max(1e-4, force_arrow_min_length_);
        force_arrow_max_length_ = std::max(force_arrow_min_length_, force_arrow_max_length_);

        // The contact manager margin must cover the force influence distance,
        // otherwise far-but-relevant contacts are filtered before we see them.
        contact_margin_ = std::max(threshold_, collision_influence_distance_);

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

        contact_manager_ = env_->getDiscreteContactManager();
        contact_manager_->setActiveCollisionObjects(env_->getActiveLinkNames());
        contact_manager_->setDefaultCollisionMargin(contact_margin_);

        contact_request_ = tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::ALL);
        contact_request_.calculate_distance = true;
        contact_request_.calculate_penetration = true;

        RCLCPP_INFO(this->get_logger(), "Tesseract Environment Initialized Successfully!");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 10);
        contact_debug_pub_ = this->create_publisher<std_msgs::msg::String>("collision_debug_contacts", 10);
        collision_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/motomini/collision_wrench", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&OnlineCollisionDebugger::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Online Collision Debugger started. threshold=%.3f m, influence=%.3f m, safe=%.3f m, "
                    "markers=%s, wrench=%s",
                    threshold_, collision_influence_distance_, collision_safe_distance_,
                    debug_markers_enabled_ ? "on" : "off",
                    publish_collision_wrench_ ? "on" : "off");
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

    static Eigen::Vector3d clampNorm(const Eigen::Vector3d &v, double max_norm)
    {
        if (!v.allFinite())
            return Eigen::Vector3d::Zero();

        const double n = v.norm();
        if (max_norm <= 0.0 || n <= max_norm || n < 1e-9)
            return v;

        return v * (max_norm / n);
    }

    Eigen::Vector3d computeCollisionForce(double distance,
                                          const Eigen::Vector3d &push_dir) const
    {
        const double d0 = std::max(1e-6, collision_influence_distance_);
        const double d_safe = std::max(0.0, collision_safe_distance_);

        if (distance >= d0)
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d n = push_dir;
        if (!n.allFinite() || n.norm() < 1e-9)
            return Eigen::Vector3d::Zero();
        n.normalize();

        // Floor distance to avoid singularities when the robot has already
        // penetrated. The hold term picks up the slack in that regime.
        const double d = std::max(distance, 1e-4);

        const double rep_mag =
            collision_k_rep_ * (1.0 / d - 1.0 / d0) / (d * d);

        double hold_mag = 0.0;
        if (distance < d_safe)
            hold_mag = collision_k_hold_ * (d_safe - distance);

        const Eigen::Vector3d f = (rep_mag + hold_mag) * n;
        return clampNorm(f, collision_force_max_per_contact_);
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

        const double force_ratio =
            std::clamp(force.norm() / std::max(1e-9, collision_force_max_per_contact_),
                       0.0, 1.0);

        const double arrow_length =
            force_arrow_min_length_ +
            force_ratio * (force_arrow_max_length_ - force_arrow_min_length_);

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
        arrow_marker.scale.x = 0.01;
        arrow_marker.scale.y = 0.025;
        arrow_marker.scale.z = 0.04;
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.85;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.9;
        marker_array.markers.push_back(arrow_marker);
    }

    void addTotalForceArrow(visualization_msgs::msg::MarkerArray &marker_array,
                            const Eigen::Vector3d &start,
                            const Eigen::Vector3d &force,
                            int &id_counter)
    {
        if (!start.allFinite() || !force.allFinite() || force.norm() < 1e-9)
            return;

        const double force_ratio =
            std::clamp(force.norm() / std::max(1e-9, collision_force_max_total_),
                       0.0, 1.0);

        const double arrow_length =
            force_arrow_min_length_ +
            force_ratio * (force_arrow_max_length_ - force_arrow_min_length_);

        const Eigen::Vector3d end = start + force.normalized() * arrow_length;

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = marker_frame_;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "collision_total_force";
        arrow_marker.id = id_counter++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.points.push_back(toPoint(start));
        arrow_marker.points.push_back(toPoint(end));
        arrow_marker.scale.x = 0.014;
        arrow_marker.scale.y = 0.032;
        arrow_marker.scale.z = 0.05;
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 1.0;
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
            const double fallback_len = std::min(force_arrow_max_length_, std::max(0.02, 0.5 * threshold_));
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

    void publishCollisionWrench(const Eigen::Vector3d &force,
                                const Eigen::Vector3d &torque)
    {
        if (!publish_collision_wrench_)
            return;

        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = wrench_frame_;
        msg.wrench.force.x = force.x();
        msg.wrench.force.y = force.y();
        msg.wrench.force.z = force.z();
        msg.wrench.torque.x = torque.x();
        msg.wrench.torque.y = torque.y();
        msg.wrench.torque.z = torque.z();
        collision_wrench_pub_->publish(msg);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!env_ || !contact_manager_)
            return;

        // 1. Update environment state
        Eigen::VectorXd joint_positions = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
        env_->setState(msg->name, joint_positions);

        // 2. Push transforms into the contact manager
        tesseract_scene_graph::SceneState current_state = env_->getState();
        contact_manager_->setCollisionObjectsTransform(current_state.link_transforms);

        // 3. Wrench reference point (defaults to origin if link not found)
        Eigen::Vector3d wrench_ref = Eigen::Vector3d::Zero();
        auto it_ref = current_state.link_transforms.find(wrench_reference_link_);
        if (it_ref != current_state.link_transforms.end())
            wrench_ref = it_ref->second.translation();

        // 4. Run collision check
        tesseract_collision::ContactResultMap contact_results;
        contact_manager_->contactTest(contact_results, contact_request_);

        // 5. Prepare outputs
        visualization_msgs::msg::MarkerArray marker_array;
        std_msgs::msg::String debug_msg;
        std::ostringstream debug_stream;

        if (debug_markers_enabled_)
        {
            marker_array.markers.reserve((contact_results.size() * 6) + 1);
            visualization_msgs::msg::Marker delete_all;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_all);
        }

        int id_counter = 0;
        size_t contact_counter = 0;
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
        Eigen::Vector3d total_torque = Eigen::Vector3d::Zero();

        auto acm = env_->getAllowedCollisionMatrix();

        debug_stream << std::fixed << std::setprecision(6);
        debug_stream << "threshold_m=" << threshold_
                     << " influence_m=" << collision_influence_distance_
                     << " safe_m=" << collision_safe_distance_;

        for (const auto &pair : contact_results)
        {
            for (const auto &result : pair.second)
            {
                if (acm && acm->isCollisionAllowed(result.link_names[0], result.link_names[1]))
                    continue;

                if (isToolCollisionPairIgnored(result.link_names[0], result.link_names[1]))
                    continue;

                if (result.distance >= contact_margin_)
                    continue;

                const Eigen::Vector3d normal_0_to_1 = contactNormal0To1(result);
                const Eigen::Vector3d clearance_vector =
                    result.nearest_points[1] - result.nearest_points[0];

                // Push-away convention from the upgrade plan:
                //   normal_0_to_1 pushes link1 away from link0,
                //   so the force on link0 uses -normal, link1 uses +normal.
                const Eigen::Vector3d force_on_link0 =
                    computeCollisionForce(result.distance, -normal_0_to_1);
                const Eigen::Vector3d force_on_link1 =
                    computeCollisionForce(result.distance, normal_0_to_1);

                // Robot-side selection: pick the larger of the two as the
                // controller-facing force. For external obstacles only one
                // side is active; for self-collision both are equal-and-
                // opposite and this picks one consistently.
                Eigen::Vector3d force_for_controller = force_on_link0;
                if (force_on_link1.norm() > force_on_link0.norm())
                    force_for_controller = force_on_link1;

                const Eigen::Vector3d contact_point =
                    0.5 * (result.nearest_points[0] + result.nearest_points[1]);

                total_force += force_for_controller;
                total_torque += (contact_point - wrench_ref).cross(force_for_controller);

                if (debug_markers_enabled_)
                {
                    addDistanceLine(marker_array, result, id_counter);
                    addContactPoint(marker_array, result, 0, id_counter);
                    addContactPoint(marker_array, result, 1, id_counter);
                    addVectorToObstacleArrow(marker_array, result, normal_0_to_1, id_counter);
                    addForceArrow(marker_array, result.nearest_points[0], force_on_link0,
                                  "collision_push_force_link0", id_counter);
                    addForceArrow(marker_array, result.nearest_points[1], force_on_link1,
                                  "collision_push_force_link1", id_counter);
                    addForceArrow(marker_array, contact_point, force_for_controller,
                                  "collision_force_for_controller", id_counter);
                }

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
                             << " f0=" << vecToString(force_on_link0)
                             << " f1=" << vecToString(force_on_link1)
                             << " f_ctrl=" << vecToString(force_for_controller);
            }
        }

        total_force = clampNorm(total_force, collision_force_max_total_);

        debug_stream << "\ncontact_count=" << contact_counter
                     << " total_force=" << vecToString(total_force)
                     << " total_torque=" << vecToString(total_torque);

        publishCollisionWrench(total_force, total_torque);

        if (debug_markers_enabled_)
        {
            addTotalForceArrow(marker_array, wrench_ref, total_force, id_counter);
            marker_pub_->publish(marker_array);
        }

        if (publish_contact_debug_text_)
        {
            debug_msg.data = debug_stream.str();
            contact_debug_pub_->publish(debug_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr collision_wrench_pub_;
    std::shared_ptr<tesseract_environment::Environment> env_;

    tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
    tesseract_collision::ContactRequest contact_request_;

    double threshold_;
    double contact_margin_;
    double contact_point_radius_;
    std::string marker_frame_;

    bool debug_markers_enabled_;
    bool publish_collision_wrench_;
    bool publish_contact_debug_text_;

    std::string wrench_frame_;
    std::string wrench_reference_link_;

    double collision_influence_distance_;
    double collision_safe_distance_;
    double collision_k_rep_;
    double collision_k_hold_;
    double collision_force_max_per_contact_;
    double collision_force_max_total_;

    double force_arrow_min_length_;
    double force_arrow_max_length_;
    double force_arrow_length_gain_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCollisionDebugger>());
    rclcpp::shutdown();
    return 0;
}
