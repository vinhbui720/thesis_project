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
#include <tesseract_common/resource_locator.h>

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
    this->declare_parameter<std::string>("tool_param", ""); // Alias for ee_link
    this->declare_parameter<bool>("online_mode", false);
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("use_ompl", false);
    this->declare_parameter<double>("tf_poll_rate_hz", 200.0);
    this->declare_parameter<int>("planning_chunk_size", 20);
    this->declare_parameter<int>("planning_parallel_chunks", 2);

    // Embedded feedback-stream controller parameters. Names intentionally
    // match motomini_feedback_stream.cpp so the same tuning values can be used.
    this->declare_parameter<double>("rate_hz", 50.0);
    this->declare_parameter<double>("w0", 0.01);
    this->declare_parameter<double>("k0", 0.01);
    this->declare_parameter<double>("theta_d_lim", 3.14);
    this->declare_parameter<bool>("enable_seed", false);
    this->declare_parameter<double>("kp_max", 3.5);
    this->declare_parameter<double>("ko_max", 2.5);
    this->declare_parameter<double>("kdp", 0.25);
    this->declare_parameter<double>("kdo", 0.25);
    this->declare_parameter<double>("m_pos_min", 0.5);
    this->declare_parameter<double>("m_pos_max", 5.0);
    this->declare_parameter<double>("k_pos_min", 5.0);
    this->declare_parameter<double>("k_pos_max", 50.0);
    this->declare_parameter<double>("zeta_pos", 0.9);
    this->declare_parameter<double>("m_ori_min", 0.2);
    this->declare_parameter<double>("m_ori_max", 2.0);
    this->declare_parameter<double>("k_ori_min", 2.0);
    this->declare_parameter<double>("k_ori_max", 20.0);
    this->declare_parameter<double>("zeta_ori", 0.9);
    this->declare_parameter<double>("adaptive_lambda", 1.0);
    this->declare_parameter<double>("adaptive_alpha_pos", 30.0);
    this->declare_parameter<double>("adaptive_alpha_ori", 6.0);
    this->declare_parameter<double>("max_cart_linear_vel", 0.5);
    this->declare_parameter<double>("max_cart_angular_vel", 1.5);
    this->declare_parameter<double>("max_cart_linear_acc", 0.8);
    this->declare_parameter<double>("max_cart_angular_acc", 2.5);
    this->declare_parameter<double>("velocity_filter_cutoff_hz", 15.0);
    this->declare_parameter<double>("measured_cart_linear_vel_limit", 1.0);
    this->declare_parameter<double>("measured_cart_angular_vel_limit", 3.0);
    this->declare_parameter<double>("collision_wrench_timeout_sec", 0.2);
    this->declare_parameter<bool>("enable_collision_projection", true);
    this->declare_parameter<bool>("collision_goal_suppression", true);
    this->declare_parameter<double>("collision_guard_distance", 0.03);
    this->declare_parameter<double>("collision_task_distance", 0.005);
    this->declare_parameter<double>("collision_stop_distance", 0.001);
    this->declare_parameter<double>("collision_projection_max_gamma", 1.0);
    this->declare_parameter<double>("collision_constraint_timeout_sec", 0.2);
    this->declare_parameter<double>("collision_force_scale", 1.0);
    this->declare_parameter<double>("collision_force_max", 5.0);
    this->declare_parameter<bool>("real_robot", true);

    this->declare_parameter<std::string>("move_instruction_type", "FREESPACE");
    this->declare_parameter<double>("ompl_planning_time", 10.0);
    this->declare_parameter<int>("ompl_max_solutions", 5);
    this->declare_parameter<bool>("ompl_simplify", true);
    this->declare_parameter<double>("ompl_longest_valid_segment", 0.005);
    this->declare_parameter<double>("ompl_rrt_range_1", 0.05);
    this->declare_parameter<double>("ompl_rrt_range_2", 0.10);

    // TrajOptIfopt Defaults (matched to struct and working YAML)
    this->declare_parameter<double>("ifopt_cart_coeff_x", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_y", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_z", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_rx", 0.0); // 0.0 = Free rotation by default
    this->declare_parameter<double>("ifopt_cart_coeff_ry", 0.0);
    this->declare_parameter<double>("ifopt_cart_coeff_rz", 0.0);
    this->declare_parameter<double>("ifopt_joint_cost_coeff", 5.0);
    this->declare_parameter<double>("ifopt_coll_cost_margin", 0.02);
    this->declare_parameter<double>("ifopt_coll_cost_coeff", 500.0);
    this->declare_parameter<double>("ifopt_coll_margin_buffer", 0.02);
    this->declare_parameter<int>("ifopt_coll_eval_type", 2);
    this->declare_parameter<double>("ifopt_coll_lvs_length", 0.005);
    this->declare_parameter<double>("ifopt_smooth_vel_coeff", 0.1);
    this->declare_parameter<double>("ifopt_smooth_acc_coeff", 1.0);
    this->declare_parameter<double>("ifopt_smooth_jerk_coeff", 1.0);
    this->declare_parameter<int>("ifopt_max_iter", 300);
    this->declare_parameter<double>("ifopt_min_approx_improve", 1e-6);
    this->declare_parameter<double>("ifopt_min_trust_box_size", 1e-5);
    this->declare_parameter<double>("ifopt_initial_trust_box_size", 0.5);
    this->declare_parameter<bool>("ifopt_joint_cost_enable", true);
    this->declare_parameter<bool>("ifopt_cart_constraint_enable", true);
    this->declare_parameter<bool>("ifopt_cart_cost_enable", false);
    this->declare_parameter<bool>("ifopt_coll_constraint_enable", false);
    this->declare_parameter<bool>("ifopt_coll_cost_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_vel_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_acc_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_jerk_enable", true);

    rate_hz_ = std::max(1.0, this->get_parameter("rate_hz").as_double());
    w0_ = this->get_parameter("w0").as_double();
    k0_ = this->get_parameter("k0").as_double();
    enable_seed_ = this->get_parameter("enable_seed").as_bool();
    m_pos_min_ = this->get_parameter("m_pos_min").as_double();
    m_pos_max_ = this->get_parameter("m_pos_max").as_double();
    k_pos_min_ = this->get_parameter("k_pos_min").as_double();
    k_pos_max_ = this->get_parameter("k_pos_max").as_double();
    zeta_pos_ = this->get_parameter("zeta_pos").as_double();
    m_ori_min_ = this->get_parameter("m_ori_min").as_double();
    m_ori_max_ = this->get_parameter("m_ori_max").as_double();
    k_ori_min_ = this->get_parameter("k_ori_min").as_double();
    k_ori_max_ = this->get_parameter("k_ori_max").as_double();
    zeta_ori_ = this->get_parameter("zeta_ori").as_double();
    adaptive_lambda_ = this->get_parameter("adaptive_lambda").as_double();
    adaptive_alpha_pos_ = this->get_parameter("adaptive_alpha_pos").as_double();
    adaptive_alpha_ori_ = this->get_parameter("adaptive_alpha_ori").as_double();
    max_cart_linear_vel_ = this->get_parameter("max_cart_linear_vel").as_double();
    max_cart_angular_vel_ = this->get_parameter("max_cart_angular_vel").as_double();
    max_cart_linear_acc_ = this->get_parameter("max_cart_linear_acc").as_double();
    max_cart_angular_acc_ = this->get_parameter("max_cart_angular_acc").as_double();
    velocity_filter_cutoff_hz_ =
        this->get_parameter("velocity_filter_cutoff_hz").as_double();
    measured_cart_linear_vel_limit_ =
        this->get_parameter("measured_cart_linear_vel_limit").as_double();
    measured_cart_angular_vel_limit_ =
        this->get_parameter("measured_cart_angular_vel_limit").as_double();
    collision_wrench_timeout_sec_ =
        this->get_parameter("collision_wrench_timeout_sec").as_double();
    enable_collision_projection_ =
        this->get_parameter("enable_collision_projection").as_bool();
    collision_goal_suppression_ =
        this->get_parameter("collision_goal_suppression").as_bool();
    collision_guard_distance_ =
        this->get_parameter("collision_guard_distance").as_double();
    collision_task_distance_ =
        this->get_parameter("collision_task_distance").as_double();
    collision_stop_distance_ =
        this->get_parameter("collision_stop_distance").as_double();
    collision_projection_max_gamma_ =
        this->get_parameter("collision_projection_max_gamma").as_double();
    collision_constraint_timeout_sec_ =
        this->get_parameter("collision_constraint_timeout_sec").as_double();
    collision_force_scale_ =
        this->get_parameter("collision_force_scale").as_double();
    collision_force_max_ =
        this->get_parameter("collision_force_max").as_double();
    real_robot_ = this->get_parameter("real_robot").as_bool();

    const auto &overrides =
        this->get_node_parameters_interface()->get_parameter_overrides();
    const bool legacy_kp_override = overrides.find("kp_max") != overrides.end();
    const bool legacy_ko_override = overrides.find("ko_max") != overrides.end();
    const bool new_k_pos_max_override = overrides.find("k_pos_max") != overrides.end();
    const bool new_k_ori_max_override = overrides.find("k_ori_max") != overrides.end();
    if (legacy_kp_override && !new_k_pos_max_override)
    {
        const double legacy_kp = this->get_parameter("kp_max").as_double();
        k_pos_max_ = 50.0 * std::max(0.0, legacy_kp) / 3.5;
    }
    if (legacy_ko_override && !new_k_ori_max_override)
    {
        const double legacy_ko = this->get_parameter("ko_max").as_double();
        k_ori_max_ = 20.0 * std::max(0.0, legacy_ko) / 2.5;
    }
    sanitizeTrackingParameters();

    bool online_mode = this->get_parameter("online_mode").as_bool();
    bool debug = this->get_parameter("debug").as_bool();
    bool use_ompl = this->get_parameter("use_ompl").as_bool();
    std::string manipulator_group = this->get_parameter("manipulator_group").as_string();
    base_link_ = this->get_parameter("base_link").as_string();
    
    // Priority: tool_param > ee_link
    std::string tool_p = this->get_parameter("tool_param").as_string();
    if (!tool_p.empty())
        ee_link_ = tool_p;
    else
        ee_link_ = this->get_parameter("ee_link").as_string();

    this->get_parameter("robot_description", urdf_xml_);
    this->get_parameter("robot_description_semantic", srdf_xml_);

    // ---- Environment ----
    if (!initializeEnvironment())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize Tesseract Environment.");
        throw std::runtime_error("Tesseract init failed");
    }

    // ---- Planner ----
    planner_ = std::make_shared<Vinhtesseract_examples::MotoMiniPlanning>(
        env_, plotter_,
        manipulator_group, base_link_, ee_link_,
        debug, /*ifopt=*/true, use_ompl, online_mode);

    // ---- Cache the kinematic group for FK / Jacobian when publishing feedback ----
    manip_ = env_->getKinematicGroup(manipulator_group);
    if (!manip_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to find KinematicGroup %s — feedback topics will stay idle.",
                    manipulator_group.c_str());
    }
    else
    {
        joint_names_ = manip_->getJointNames();
        joint_limits_ = manip_->getLimits().joint_limits;
        velocity_limits_ = manip_->getLimits().velocity_limits;
        qdot_filtered_ =
            Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
    }

    // ---- Offline chunked planning parameters ----
    planner_->configureChunking(
        this->get_parameter("planning_chunk_size").as_int(),
        this->get_parameter("planning_parallel_chunks").as_int());

    // ---- Runtime-tunable planning hyperparameters (from planning_params.yaml) ----
    {
        Vinhtesseract_examples::MotoMiniPlanning::PlanningConfig cfg;
        const std::string instr = this->get_parameter("move_instruction_type").as_string();
        cfg.use_linear = (instr == "LINEAR");
        cfg.ompl_planning_time = this->get_parameter("ompl_planning_time").as_double();
        cfg.ompl_max_solutions = this->get_parameter("ompl_max_solutions").as_int();
        cfg.ompl_simplify = this->get_parameter("ompl_simplify").as_bool();
        cfg.ompl_longest_valid_segment = this->get_parameter("ompl_longest_valid_segment").as_double();
        cfg.ompl_rrt_range_1 = this->get_parameter("ompl_rrt_range_1").as_double();
        cfg.ompl_rrt_range_2 = this->get_parameter("ompl_rrt_range_2").as_double();
        cfg.use_ompl_runtime = this->get_parameter("use_ompl").as_bool();
        
        cfg.ifopt_cart_coeff_x = this->get_parameter("ifopt_cart_coeff_x").as_double();
        cfg.ifopt_cart_coeff_y = this->get_parameter("ifopt_cart_coeff_y").as_double();
        cfg.ifopt_cart_coeff_z = this->get_parameter("ifopt_cart_coeff_z").as_double();
        cfg.ifopt_cart_coeff_rx = this->get_parameter("ifopt_cart_coeff_rx").as_double();
        cfg.ifopt_cart_coeff_ry = this->get_parameter("ifopt_cart_coeff_ry").as_double();
        cfg.ifopt_cart_coeff_rz = this->get_parameter("ifopt_cart_coeff_rz").as_double();
        
        cfg.ifopt_joint_cost_coeff = this->get_parameter("ifopt_joint_cost_coeff").as_double();
        cfg.ifopt_coll_cost_margin = this->get_parameter("ifopt_coll_cost_margin").as_double();
        cfg.ifopt_coll_cost_coeff = this->get_parameter("ifopt_coll_cost_coeff").as_double();
        cfg.ifopt_coll_margin_buffer = this->get_parameter("ifopt_coll_margin_buffer").as_double();
        cfg.ifopt_coll_eval_type = this->get_parameter("ifopt_coll_eval_type").as_int();
        cfg.ifopt_coll_lvs_length = this->get_parameter("ifopt_coll_lvs_length").as_double();
        
        cfg.ifopt_smooth_vel = this->get_parameter("ifopt_smooth_vel_coeff").as_double();
        cfg.ifopt_smooth_acc = this->get_parameter("ifopt_smooth_acc_coeff").as_double();
        cfg.ifopt_smooth_jerk = this->get_parameter("ifopt_smooth_jerk_coeff").as_double();
        
        cfg.ifopt_max_iter = this->get_parameter("ifopt_max_iter").as_int();
        cfg.ifopt_min_approx_improve = this->get_parameter("ifopt_min_approx_improve").as_double();
        cfg.ifopt_min_trust_box_size = this->get_parameter("ifopt_min_trust_box_size").as_double();
        cfg.ifopt_initial_trust_box_size = this->get_parameter("ifopt_initial_trust_box_size").as_double();
        
        cfg.ifopt_joint_cost_enable = this->get_parameter("ifopt_joint_cost_enable").as_bool();
        cfg.ifopt_cart_constraint_enable = this->get_parameter("ifopt_cart_constraint_enable").as_bool();
        cfg.ifopt_cart_cost_enable = this->get_parameter("ifopt_cart_cost_enable").as_bool();
        cfg.ifopt_coll_constraint_enable = this->get_parameter("ifopt_coll_constraint_enable").as_bool();
        cfg.ifopt_coll_cost_enable = this->get_parameter("ifopt_coll_cost_enable").as_bool();
        cfg.ifopt_smooth_vel_enable = this->get_parameter("ifopt_smooth_vel_enable").as_bool();
        cfg.ifopt_smooth_acc_enable = this->get_parameter("ifopt_smooth_acc_enable").as_bool();
        cfg.ifopt_smooth_jerk_enable = this->get_parameter("ifopt_smooth_jerk_enable").as_bool();

        planner_->configurePlanningParams(cfg);

        RCLCPP_INFO(this->get_logger(),
                    "Planning config: mode=%s  ompl=%s  chunk=%d  parallel=%d  "
                    "cart=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]  coll_margin=%.3f  eval=%d",
                    instr.c_str(),
                    cfg.use_ompl_runtime ? "ON" : "OFF",
                    static_cast<int>(this->get_parameter("planning_chunk_size").as_int()),
                    static_cast<int>(this->get_parameter("planning_parallel_chunks").as_int()),
                    cfg.ifopt_cart_coeff_x, cfg.ifopt_cart_coeff_y, cfg.ifopt_cart_coeff_z,
                    cfg.ifopt_cart_coeff_rx, cfg.ifopt_cart_coeff_ry, cfg.ifopt_cart_coeff_rz,
                    cfg.ifopt_coll_cost_margin, cfg.ifopt_coll_eval_type);
    }

    planner_->setChunkReadyCallback(
        [this](const tesseract_common::JointTrajectory &chunk_traj,
               const std::vector<std::string> & /*jnames*/,
               bool is_last)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Chunk solved: %zu points%s",
                        chunk_traj.size(),
                        is_last ? " — last chunk, run() returning." : "...");
        });

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

    sub_desired_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motomini/target_pose", 1,
        std::bind(&MotoMiniPlanningNode::desiredPoseCallback, this, std::placeholders::_1));

    sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose_following/init_pose", 1,
        std::bind(&MotoMiniPlanningNode::initPoseCallback, this, std::placeholders::_1));

    sub_target_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/motomini/target_vel", 10,
        std::bind(&MotoMiniPlanningNode::targetVelCallback, this, std::placeholders::_1));

    sub_collision_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/motomini/collision_wrench", 10,
        std::bind(&MotoMiniPlanningNode::collisionWrenchCallback, this, std::placeholders::_1));

    sub_collision_distance_ = this->create_subscription<std_msgs::msg::Float64>(
        "/motomini/collision_distance", 10,
        std::bind(&MotoMiniPlanningNode::collisionDistanceCallback, this, std::placeholders::_1));

    sub_collision_normal_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/motomini/collision_normal", 10,
        std::bind(&MotoMiniPlanningNode::collisionNormalCallback, this, std::placeholders::_1));

    pub_status_ = this->create_publisher<std_msgs::msg::String>("/optimization_status", 10);
    pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/path_command", 10);
    pub_stream_path_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_path_command", 10);
    pub_stream_joint_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_command", 10);
    pub_tracked_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/motomini/tracked_tip_pose", 10);
    pub_online_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motomini/online_joint_command", 100);

    // Always-on Cartesian feedback (mirrors motomini_feedback_stream).
    pub_feedback_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/motomini/feedback", 10);
    pub_feedback_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/motomini/feedback_vel", 10);

    traj_stream_start_client_ =
        this->create_client<std_srvs::srv::Trigger>("/pose_following/start");
    traj_stream_stop_client_ =
        this->create_client<std_srvs::srv::Trigger>("/pose_following/stop");

    planner_->setCommandCallback([this](const Eigen::VectorXd &cmd)
                                 {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(cmd.data(), cmd.data() + cmd.size());
        pub_online_cmd_->publish(msg); });

    if (debug)
    {
        pub_ee_path_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/motomini/ee_dynamic_path", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        wp_tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]()
            {
                if (!accumulated_targets_.empty())
                    publishWaypointsTFs();
            });
        wp_tf_timer_->cancel();

        planner_->setToolpathCallback(
            [this](const std::vector<Eigen::Vector3d> &path)
            {
                if (path.empty())
                    return;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = base_link_;
                marker.header.stamp = this->now();
                marker.ns = "online_planner_path";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.005;
                marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;

                for (const auto &pt : path)
                {
                    geometry_msgs::msg::Point p;
                    p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
                    marker.points.push_back(p);
                }
                pub_ee_path_->publish(marker);
            });
    }

    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotoMiniPlanningNode::monitorExecution, this));

    const auto feedback_period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / rate_hz_));
    feedback_timer_ = this->create_wall_timer(
        feedback_period,
        std::bind(&MotoMiniPlanningNode::feedbackTimerCallback, this));

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MotoMiniPlanningNode::onParameterChange, this, std::placeholders::_1));
}

MotoMiniPlanningNode::~MotoMiniPlanningNode() = default;

void MotoMiniPlanningNode::postInit()
{
    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(
        shared_from_this(), env_, "tesseract");
    monitor_->startPublishingEnvironment();
    monitor_->startStateMonitor("/joint_states");
}

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
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
    return true;
}

rcl_interfaces::msg::SetParametersResult
MotoMiniPlanningNode::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    Vinhtesseract_examples::MotoMiniPlanning::PlanningConfig cfg = planner_->getPlanningConfig();
    int new_chunk_size = this->get_parameter("planning_chunk_size").as_int();
    int new_parallel = this->get_parameter("planning_parallel_chunks").as_int();
    bool chunk_changed = false;

    for (const auto &p : params)
    {
        const std::string &n = p.get_name();
        try
        {
            if (n == "move_instruction_type") cfg.use_linear = (p.as_string() == "LINEAR");
            else if (n == "use_ompl") cfg.use_ompl_runtime = p.as_bool();
            else if (n == "ifopt_cart_coeff_x") cfg.ifopt_cart_coeff_x = p.as_double();
            else if (n == "ifopt_cart_coeff_y") cfg.ifopt_cart_coeff_y = p.as_double();
            else if (n == "ifopt_cart_coeff_z") cfg.ifopt_cart_coeff_z = p.as_double();
            else if (n == "ifopt_cart_coeff_rx") cfg.ifopt_cart_coeff_rx = p.as_double();
            else if (n == "ifopt_cart_coeff_ry") cfg.ifopt_cart_coeff_ry = p.as_double();
            else if (n == "ifopt_cart_coeff_rz") cfg.ifopt_cart_coeff_rz = p.as_double();
            else if (n == "ifopt_coll_eval_type") cfg.ifopt_coll_eval_type = static_cast<int>(p.as_int());
            else if (n == "ifopt_coll_lvs_length") cfg.ifopt_coll_lvs_length = p.as_double();
            else if (n == "ifopt_joint_cost_coeff") cfg.ifopt_joint_cost_coeff = p.as_double();
            else if (n == "ifopt_coll_cost_margin") cfg.ifopt_coll_cost_margin = p.as_double();
            else if (n == "ifopt_coll_cost_coeff") cfg.ifopt_coll_cost_coeff = p.as_double();
            else if (n == "ifopt_coll_margin_buffer") cfg.ifopt_coll_margin_buffer = p.as_double();
            else if (n == "ifopt_smooth_vel_coeff") cfg.ifopt_smooth_vel = p.as_double();
            else if (n == "ifopt_smooth_acc_coeff") cfg.ifopt_smooth_acc = p.as_double();
            else if (n == "ifopt_smooth_jerk_coeff") cfg.ifopt_smooth_jerk = p.as_double();
            else if (n == "ifopt_max_iter") cfg.ifopt_max_iter = static_cast<int>(p.as_int());
            else if (n == "ifopt_min_approx_improve") cfg.ifopt_min_approx_improve = p.as_double();
            else if (n == "ifopt_min_trust_box_size") cfg.ifopt_min_trust_box_size = p.as_double();
            else if (n == "ifopt_initial_trust_box_size") cfg.ifopt_initial_trust_box_size = p.as_double();
            else if (n == "ifopt_joint_cost_enable") cfg.ifopt_joint_cost_enable = p.as_bool();
            else if (n == "ifopt_cart_constraint_enable") cfg.ifopt_cart_constraint_enable = p.as_bool();
            else if (n == "ifopt_cart_cost_enable") cfg.ifopt_cart_cost_enable = p.as_bool();
            else if (n == "ifopt_coll_constraint_enable") cfg.ifopt_coll_constraint_enable = p.as_bool();
            else if (n == "ifopt_coll_cost_enable") cfg.ifopt_coll_cost_enable = p.as_bool();
            else if (n == "ifopt_smooth_vel_enable") cfg.ifopt_smooth_vel_enable = p.as_bool();
            else if (n == "ifopt_smooth_acc_enable") cfg.ifopt_smooth_acc_enable = p.as_bool();
            else if (n == "ifopt_smooth_jerk_enable") cfg.ifopt_smooth_jerk_enable = p.as_bool();
            else if (n == "ompl_planning_time") cfg.ompl_planning_time = p.as_double();
            else if (n == "ompl_max_solutions") cfg.ompl_max_solutions = static_cast<int>(p.as_int());
            else if (n == "ompl_simplify") cfg.ompl_simplify = p.as_bool();
            else if (n == "ompl_rrt_range_1") cfg.ompl_rrt_range_1 = p.as_double();
            else if (n == "ompl_rrt_range_2") cfg.ompl_rrt_range_2 = p.as_double();
            else if (n == "planning_chunk_size") { new_chunk_size = static_cast<int>(p.as_int()); chunk_changed = true; }
            else if (n == "planning_parallel_chunks") { new_parallel = static_cast<int>(p.as_int()); chunk_changed = true; }
            else if (n == "real_robot") { real_robot_ = p.as_bool(); first_velocity_read_ = true; qdot_filtered_.resize(0); have_velocity_filter_update_ = false; }
            else if (n == "velocity_filter_cutoff_hz") { velocity_filter_cutoff_hz_ = p.as_double(); sanitizeTrackingParameters(); }
            else if (n == "max_cart_linear_acc") { max_cart_linear_acc_ = p.as_double(); sanitizeTrackingParameters(); }
            else if (n == "max_cart_angular_acc") { max_cart_angular_acc_ = p.as_double(); sanitizeTrackingParameters(); }
            else if (n == "measured_cart_linear_vel_limit") { measured_cart_linear_vel_limit_ = p.as_double(); sanitizeTrackingParameters(); }
            else if (n == "measured_cart_angular_vel_limit") { measured_cart_angular_vel_limit_ = p.as_double(); sanitizeTrackingParameters(); }
            else if (n == "ee_link" || n == "tool_param")
            {
                ee_link_ = p.as_string();
                planner_->setEndEffector(ee_link_);
                RCLCPP_INFO(this->get_logger(), "EE Link updated to: %s", ee_link_.c_str());
            }
        }
        catch (const std::exception &e)
        {
            result.successful = false;
            result.reason = "Failed to apply " + n + ": " + e.what();
            return result;
        }
    }

    planner_->configurePlanningParams(cfg);
    if (chunk_changed)
        planner_->configureChunking(new_chunk_size, new_parallel);

    RCLCPP_INFO(this->get_logger(), "[ParamUpdate] mode=%s  cart_coeff_rx=%.1f  max_iter=%d",
                cfg.use_linear ? "LINEAR" : "FREESPACE", cfg.ifopt_cart_coeff_rx, cfg.ifopt_max_iter);
    return result;
}
