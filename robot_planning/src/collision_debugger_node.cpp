#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

        // Safety zones consumed by the controller (close-work design)
        this->declare_parameter<double>("collision_guard_distance", 0.03);
        this->declare_parameter<double>("collision_task_distance", 0.005);
        this->declare_parameter<double>("collision_stop_distance", 0.001);

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

        collision_guard_distance_ = this->get_parameter("collision_guard_distance").as_double();
        collision_task_distance_ = this->get_parameter("collision_task_distance").as_double();
        collision_stop_distance_ = this->get_parameter("collision_stop_distance").as_double();

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

        collision_stop_distance_ = std::max(0.0, collision_stop_distance_);
        collision_task_distance_ = std::max(collision_stop_distance_, collision_task_distance_);
        collision_guard_distance_ = std::max(collision_task_distance_, collision_guard_distance_);

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
        collision_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motomini/collision_distance", 10);
        collision_normal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/motomini/collision_normal", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&OnlineCollisionDebugger::jointStateCallback, this, std::placeholders::_1));
        target_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/motomini/target_vel", 10,
            std::bind(&OnlineCollisionDebugger::targetVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Online Collision Debugger started. threshold=%.3f m, influence=%.3f m, safe=%.3f m, "
                    "guard=%.3f m, task=%.3f m, stop=%.3f m, markers=%s, wrench=%s",
                    threshold_, collision_influence_distance_, collision_safe_distance_,
                    collision_guard_distance_, collision_task_distance_, collision_stop_distance_,
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

    static double lowPassAlpha(double cutoff_hz, double dt)
    {
        if (dt <= 0.0)
            return 1.0;
        return std::clamp(1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt), 0.0, 1.0);
    }

    std::string classifyZone(double d) const
    {
        if (d <= collision_stop_distance_)
            return "STOP";
        if (d <= collision_task_distance_)
            return "TASK";
        if (d <= collision_guard_distance_)
            return "GUARD";
        if (d <= collision_influence_distance_)
            return "INFLUENCE";
        return "FREE";
    }

    double computeProjectionGamma(double d) const
    {
        const double span =
            std::max(1e-6, collision_guard_distance_ - collision_task_distance_);
        return 1.0 - std::clamp((d - collision_task_distance_) / span, 0.0, 1.0);
    }

    double targetVelocityTimeout() const
    {
        return std::max(0.05, 0.75 * collision_influence_distance_);
    }

    double tangentialVelocityDeadband() const
    {
        const double span = std::max(1e-4, collision_guard_distance_ - collision_task_distance_);
        return std::max(0.002, 0.2 * span);
    }

    double tangentialProjectionEpsilon() const
    {
        const double span = std::max(1e-6, collision_guard_distance_ - collision_task_distance_);
        return std::max(1e-6, 0.05 * span);
    }

    double tangentialSpeedScale() const
    {
        const double span = std::max(1e-4, collision_guard_distance_ - collision_task_distance_);
        return std::max(0.01, span);
    }

    double tangentialForceLimit() const
    {
        return std::max(0.0, 0.35 * collision_force_max_total_);
    }

    double tangentialForceGain() const
    {
        return tangentialForceLimit();
    }

    double collisionForceAttackHz() const
    {
        const double tau = std::max(0.02, 0.25 * std::max(1e-3, collision_influence_distance_));
        return 1.0 / tau;
    }

    double collisionForceReleaseHz() const
    {
        return std::max(1.0, 0.35 * collisionForceAttackHz());
    }

    Eigen::Vector3d fallbackTangent(const Eigen::Vector3d &n_away) const
    {
        Eigen::Vector3d tangent = n_away.cross(Eigen::Vector3d::UnitZ());
        if (!tangent.allFinite() || tangent.norm() < tangentialProjectionEpsilon())
            tangent = n_away.cross(Eigen::Vector3d::UnitX());
        if (!tangent.allFinite() || tangent.norm() < tangentialProjectionEpsilon())
            tangent = Eigen::Vector3d::UnitY();
        return tangent.normalized();
    }

    Eigen::Vector3d computeTangentialForce(double distance,
                                           const Eigen::Vector3d &n_away) 
    {
        const double target_age = (this->now() - t_last_target_vel_cb_).seconds();
        const double v_deadband = tangentialVelocityDeadband();
        if (target_age > targetVelocityTimeout())
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d v_goal = latest_target_vel_linear_;
        if (!v_goal.allFinite() || v_goal.norm() <= v_deadband)
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d n = n_away;
        if (!n.allFinite() || n.norm() < 1e-9)
            return Eigen::Vector3d::Zero();
        n.normalize();

        const double v_into = v_goal.dot(n);
        if (v_into >= -v_deadband)
            return Eigen::Vector3d::Zero();

        const Eigen::Vector3d v_proj = v_goal - v_into * n;
        Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
        if (v_proj.norm() >= tangentialProjectionEpsilon())
            tangent = v_proj.normalized();
        else if (last_tangent_dir_.allFinite() && last_tangent_dir_.norm() >= 1e-9)
            tangent = last_tangent_dir_.normalized();
        else
            tangent = fallbackTangent(n);

        if (last_tangent_dir_.allFinite() &&
            last_tangent_dir_.norm() >= 1e-9 &&
            tangent.dot(last_tangent_dir_) < 0.0)
            tangent = -tangent;
        last_tangent_dir_ = tangent;

        const double gamma = computeProjectionGamma(distance);
        if (gamma <= 0.0)
            return Eigen::Vector3d::Zero();

        const double approach_ratio =
            std::clamp((-v_into - v_deadband) / tangentialSpeedScale(),
                       0.0, 1.0);
        const double tangential_mag =
            tangentialForceGain() * gamma * gamma * approach_ratio;
        return clampNorm(tangential_mag * tangent, tangentialForceLimit());
    }

    Eigen::Vector3d smoothPublishedForce(const Eigen::Vector3d &raw_force)
    {
        const rclcpp::Time now = this->now();
        double dt = 0.0;
        if (have_force_filter_state_)
            dt = (now - t_last_force_filter_update_).seconds();
        t_last_force_filter_update_ = now;

        if (!have_force_filter_state_ || dt <= 0.0 || dt > 1.0)
        {
            filtered_total_force_ = raw_force;
            have_force_filter_state_ = true;
            return filtered_total_force_;
        }

        const double cutoff_hz =
            (raw_force.norm() >= filtered_total_force_.norm())
                ? collisionForceAttackHz()
                : collisionForceReleaseHz();
        const double alpha = lowPassAlpha(cutoff_hz, dt);
        filtered_total_force_ += alpha * (raw_force - filtered_total_force_);

        if (filtered_total_force_.norm() < 1e-6 && raw_force.norm() < 1e-6)
            filtered_total_force_.setZero();

        return filtered_total_force_;
    }

    Eigen::Vector3d computeCollisionForce(double distance,
                                          const Eigen::Vector3d &push_dir) const
    {
        const double d0 = std::max(1e-6, collision_influence_distance_);

        if (distance >= d0)
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d n = push_dir;
        if (!n.allFinite() || n.norm() < 1e-9)
            return Eigen::Vector3d::Zero();
        n.normalize();

        const double d = std::max(distance, 1e-4);

        // gamma = 0 far from wall, gamma = 1 at/inside task wall.
        const double wall_gamma = computeProjectionGamma(distance);

        // Normal spring fades out as the controller wall activates.
        // This prevents spring/string oscillation at the edge.
        const double spring_scale = 1.0 - wall_gamma;

        const double rep_mag =
            spring_scale *
            collision_k_rep_ *
            (1.0 / d - 1.0 / d0) /
            (d * d);

        // Emergency-only hold.
        // Do not use hold at the normal safe wall, because that causes bounce.
        double hold_mag = 0.0;
        if (distance < collision_stop_distance_)
        {
            hold_mag =
                collision_k_hold_ *
                (collision_stop_distance_ - distance);
        }

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

    void targetVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_target_vel_linear_ <<
            msg->linear.x, msg->linear.y, msg->linear.z;
        t_last_target_vel_cb_ = this->now();
    }

    // Publish closest-contact constraint data so the controller can run
    // velocity projection and goal-force suppression. When no contact is
    // active, advertise a large distance and zero normal so the controller
    // disables the projection.
    void publishCollisionConstraint(bool has_contact,
                                    double distance,
                                    const Eigen::Vector3d &n_away)
    {
        std_msgs::msg::Float64 dist_msg;
        dist_msg.data = has_contact
                            ? distance
                            : std::max(collision_influence_distance_,
                                       contact_margin_) +
                                  1.0;
        collision_distance_pub_->publish(dist_msg);

        geometry_msgs::msg::Vector3Stamped n_msg;
        n_msg.header.stamp = this->now();
        n_msg.header.frame_id = wrench_frame_;
        if (has_contact && n_away.allFinite() && n_away.norm() > 1e-9)
        {
            const Eigen::Vector3d n = n_away.normalized();
            n_msg.vector.x = n.x();
            n_msg.vector.y = n.y();
            n_msg.vector.z = n.z();
        }
        else
        {
            n_msg.vector.x = 0.0;
            n_msg.vector.y = 0.0;
            n_msg.vector.z = 0.0;
        }
        collision_normal_pub_->publish(n_msg);
    }

    void addInfoTextMarker(visualization_msgs::msg::MarkerArray &marker_array,
                           const Eigen::Vector3d &anchor,
                           double d_min,
                           const std::string &zone,
                           double force_norm,
                           double gamma,
                           const Eigen::Vector3d &n_away,
                           int &id_counter)
    {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = marker_frame_;
        text_marker.header.stamp = this->now();
        text_marker.ns = "collision_info";
        text_marker.id = id_counter++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = toPoint(anchor + Eigen::Vector3d(0.0, 0.0, 0.08));
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.022;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        std::ostringstream ss;
        ss << std::fixed
           << "d_min: " << std::setprecision(4) << d_min << " m\n"
           << "zone: " << zone << "\n"
           << "|F|: " << std::setprecision(2) << force_norm << "\n"
           << "gamma: " << std::setprecision(2) << gamma << "\n"
           << "n_away: " << vecToString(n_away);
        text_marker.text = ss.str();
        marker_array.markers.push_back(text_marker);
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

        // Closest-contact tracking for the controller-facing constraint topic.
        double closest_distance = std::numeric_limits<double>::infinity();
        Eigen::Vector3d closest_n_away = Eigen::Vector3d::Zero();
        Eigen::Vector3d closest_contact_point = Eigen::Vector3d::Zero();
        bool has_closest = false;

        auto acm = env_->getAllowedCollisionMatrix();

        debug_stream << std::fixed << std::setprecision(6);
        debug_stream << "threshold_m=" << threshold_
                     << " influence_m=" << collision_influence_distance_
                     << " safe_m=" << collision_safe_distance_
                     << " guard_m=" << collision_guard_distance_
                     << " task_m=" << collision_task_distance_
                     << " stop_m=" << collision_stop_distance_;

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

                // n_away MUST track the wrench's controller-facing direction.
                // The two per-link forces have identical magnitude (same d,
                // same model), so the wrench's tie-breaker always keeps
                // force_on_link0; deriving n_away from force_for_controller
                // guarantees the same sign and avoids a flipped-normal bug
                // where the controller would project away the safe motion.
                Eigen::Vector3d n_away_for_contact = Eigen::Vector3d::Zero();
                if (force_for_controller.allFinite() &&
                    force_for_controller.norm() > 1e-9)
                    n_away_for_contact = force_for_controller.normalized();

                if (result.distance < closest_distance)
                {
                    closest_distance = result.distance;
                    closest_n_away = n_away_for_contact;
                    closest_contact_point = contact_point;
                    has_closest = true;
                }

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

        Eigen::Vector3d tangential_force = Eigen::Vector3d::Zero();
        if (has_closest)
            tangential_force = computeTangentialForce(closest_distance, closest_n_away);

        total_force += tangential_force;
        total_force = clampNorm(total_force, collision_force_max_total_);
        total_force = smoothPublishedForce(total_force);

        const std::string zone =
            has_closest ? classifyZone(closest_distance) : std::string("FREE");
        const double gamma_dbg =
            has_closest ? computeProjectionGamma(closest_distance) : 0.0;

        debug_stream << "\ncontact_count=" << contact_counter
                     << " total_force=" << vecToString(total_force)
                     << " total_torque=" << vecToString(total_torque)
                     << " closest_d=" << closest_distance
                     << " zone=" << zone
                     << " gamma=" << gamma_dbg
                     << " n_away=" << vecToString(closest_n_away)
                     << " f_tan=" << vecToString(tangential_force);

        publishCollisionWrench(total_force, total_torque);
        publishCollisionConstraint(has_closest, closest_distance, closest_n_away);

        if (debug_markers_enabled_)
        {
            addTotalForceArrow(marker_array, wrench_ref, total_force, id_counter);
            const Eigen::Vector3d text_anchor =
                has_closest ? closest_contact_point : wrench_ref;
            addInfoTextMarker(marker_array, text_anchor,
                              has_closest ? closest_distance
                                          : std::numeric_limits<double>::infinity(),
                              zone, total_force.norm(), gamma_dbg,
                              closest_n_away, id_counter);
            marker_pub_->publish(marker_array);
        }

        if (publish_contact_debug_text_)
        {
            debug_msg.data = debug_stream.str();
            contact_debug_pub_->publish(debug_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_vel_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr collision_wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr collision_distance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr collision_normal_pub_;
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

    double collision_guard_distance_;
    double collision_task_distance_;
    double collision_stop_distance_;

    double force_arrow_min_length_;
    double force_arrow_max_length_;
    double force_arrow_length_gain_;
    Eigen::Vector3d latest_target_vel_linear_{Eigen::Vector3d::Zero()};
    rclcpp::Time t_last_target_vel_cb_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d last_tangent_dir_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_total_force_{Eigen::Vector3d::Zero()};
    rclcpp::Time t_last_force_filter_update_{0, 0, RCL_ROS_TIME};
    bool have_force_filter_state_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCollisionDebugger>());
    rclcpp::shutdown();
    return 0;
}
