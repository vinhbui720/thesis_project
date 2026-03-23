/**
 * @file motomini_node_setup.cpp
 * @brief MotoMiniPlanningNode — constructor, postInit, and environment initialization.
 *
 * Responsibilities:
 *   - Declare/read ROS2 parameters
 *   - Initialize Tesseract environment and plotter
 *   - Create all subscribers, publishers, and timers
 *   - Register planner callbacks (toolpath viz, online command)
 *   - Start environment monitor and capture initial robot pose
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_kinematics/core/kinematic_group.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>
#include <chrono>

using namespace Vinhtesseract_examples;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MotoMiniPlanningNode::MotoMiniPlanningNode() : Node("motomini_planning_node")
{
    // ---- Parameters ----
    this->declare_parameter<std::string>("robot_description",
                                         "package://robot_planning/urdf/motoman_motomini.urdf");
    this->declare_parameter<std::string>("robot_description_semantic",
                                         "package://robot_planning/urdf/motoman_motomini.srdf");
    this->declare_parameter<std::string>("manipulator_group", "manipulator");
    this->declare_parameter<std::string>("base_link", "world");
    this->declare_parameter<std::string>("ee_link", "tool0");
    this->declare_parameter<bool>("online_mode", false);
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("use_ompl", false);
    this->declare_parameter<bool>("tracking_mode", false);
    this->declare_parameter<double>("tracking_rate_hz", 5.0);

    tracking_mode_ = this->get_parameter("tracking_mode").as_bool();
    tracking_rate_hz_ = this->get_parameter("tracking_rate_hz").as_double();
    bool online_mode = this->get_parameter("online_mode").as_bool();
    bool debug = this->get_parameter("debug").as_bool();
    bool use_ompl = this->get_parameter("use_ompl").as_bool();
    std::string manipulator_group = this->get_parameter("manipulator_group").as_string();
    std::string base_link = this->get_parameter("base_link").as_string();
    std::string ee_link = this->get_parameter("ee_link").as_string();
    this->get_parameter("robot_description", urdf_xml_);
    this->get_parameter("robot_description_semantic", srdf_xml_);

    // ---- Environment ----
    if (!initializeEnvironment())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize Tesseract Environment.");
        throw std::runtime_error("Tesseract init failed");
    }

    // ---- Planner ----
    planner_ = std::make_shared<MotoMiniPlanning>(
        env_, plotter_,
        manipulator_group, base_link, ee_link,
        debug, /*ifopt=*/true, use_ompl, online_mode);

    // ---- Subscribers ----
    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MotoMiniPlanningNode::jointStateCallback, this, std::placeholders::_1));

    sub_targets_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/target_poses", 10,
        std::bind(&MotoMiniPlanningNode::targetPosesCallback, this, std::placeholders::_1));

    sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start", 10,
        std::bind(&MotoMiniPlanningNode::startCallback, this, std::placeholders::_1));

    sub_clear_ = this->create_subscription<std_msgs::msg::Bool>(
        "/clear_targets", 10,
        std::bind(&MotoMiniPlanningNode::clearCallback, this, std::placeholders::_1));

    sub_tracking_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/tracking_control", 10,
        std::bind(&MotoMiniPlanningNode::trackingControlCallback, this, std::placeholders::_1));

    // ---- Publishers ----
    pub_status_ = this->create_publisher<std_msgs::msg::String>("/optimization_status", 10);
    pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
    pub_online_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motomini/online_joint_command", 100);

    // Online command callback: forwards each SQP step to the low-level controller
    planner_->setCommandCallback([this](const Eigen::VectorXd &cmd)
                                 {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(cmd.data(), cmd.data() + cmd.size());
        pub_online_cmd_->publish(msg); });

    // ---- Debug-only setup ----
    if (debug)
    {
        pub_ee_path_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/motomini/ee_dynamic_path", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Republish waypoint TF frames at 5 Hz so Rviz keeps them alive
        wp_tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]()
            {
                if (!accumulated_targets_.empty())
                    publishWaypointsTFs();
            });
        wp_tf_timer_->cancel(); // activated on first pose received

        // Toolpath marker: bright green LINE_STRIP of EE positions
        planner_->setToolpathCallback(
            [this, base_link](const std::vector<Eigen::Vector3d> &path)
            {
                if (path.empty())
                    return;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = base_link;
                marker.header.stamp = this->now();
                marker.ns = "online_planner_path";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.005;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;

                for (const auto &pt : path)
                {
                    geometry_msgs::msg::Point p;
                    p.x = pt.x();
                    p.y = pt.y();
                    p.z = pt.z();
                    marker.points.push_back(p);
                }
                pub_ee_path_->publish(marker);
            });
    }
    else
    {
        planner_->setToolpathCallback({});
    }

    // ---- Execution monitor timer (10 Hz) ----
    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotoMiniPlanningNode::monitorExecution, this));

    // ---- TF listener ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Tracking timer ----
    if (tracking_mode_)
    {
        const double hz = std::max(0.1, tracking_rate_hz_);
        const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));

        tracking_timer_ = this->create_wall_timer(
            period,
            std::bind(&MotoMiniPlanningNode::trackingTick, this));

        RCLCPP_INFO(this->get_logger(),
                    "Tracking mode enabled: %.2f Hz, world='%s', base='%s', tip='%s'",
                    hz,
                    tracking_world_frame_.c_str(),
                    tracking_gantry_base_frame_.c_str(),
                    tracking_tip_frame_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "MotoMini Planning Node Ready.");
    RCLCPP_INFO(this->get_logger(),
                "Topics: /joint_states, /target_poses, /clear_targets, /start, /tracking_control");
}

// ---------------------------------------------------------------------------
// postInit — called after shared_from_this() is valid
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::postInit()
{
    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(
        shared_from_this(), env_, "tesseract");
    monitor_->startPublishingEnvironment();
    monitor_->startStateMonitor("/joint_states");

    // Capture initial EE pose so tracking-disabled mode can return to it
    const std::vector<std::string> joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
    auto manip = env_->getKinematicGroup("manipulator");
    if (manip)
    {
        Eigen::VectorXd q = env_->getCurrentJointValues(joint_names);
        auto fk = manip->calcFwdKin(q);
        if (!fk.empty())
        {
            initial_robot_pose_ = fk.at("tool0");
            RCLCPP_INFO(this->get_logger(), "Initial pose stored for tracking mode reset.");
        }
    }

    RCLCPP_INFO(this->get_logger(), "Environment monitor started.");
}

// ---------------------------------------------------------------------------
// initializeEnvironment
// ---------------------------------------------------------------------------
bool MotoMiniPlanningNode::initializeEnvironment()
{
    if (urdf_xml_.empty() || srdf_xml_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "URDF or SRDF parameter is empty.");
        return false;
    }

    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    env_ = std::make_shared<tesseract_environment::Environment>();

    if (!env_->init(urdf_xml_, srdf_xml_, locator))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract environment.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Tesseract environment initialized successfully.");
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(
        env_->getSceneGraph()->getRoot());
    return true;
}
