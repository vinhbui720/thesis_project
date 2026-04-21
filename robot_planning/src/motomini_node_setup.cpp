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
    this->declare_parameter<double>("tracking_rate_hz", 30.0);
    this->declare_parameter<bool>("tracking_use_trajopt", false);
    this->declare_parameter<bool>("tracking_enable_collision", false);
    this->declare_parameter<int>("tracking_num_steps", 5);
    this->declare_parameter<int>("tracking_trajopt_max_iter", 5);
    this->declare_parameter<double>("tracking_max_joint_step", 0.15);
    this->declare_parameter<double>("tf_poll_rate_hz", 200.0);
    this->declare_parameter<double>("tracking_ema_alpha", 0.6);

    tracking_rate_hz_ = this->get_parameter("tracking_rate_hz").as_double();
    tf_poll_rate_hz_ = this->get_parameter("tf_poll_rate_hz").as_double();
    tracking_ema_alpha_ = this->get_parameter("tracking_ema_alpha").as_double();
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
    planner_->configureTracking(
        this->get_parameter("tracking_use_trajopt").as_bool(),
        this->get_parameter("tracking_enable_collision").as_bool(),
        this->get_parameter("tracking_num_steps").as_int(),
        this->get_parameter("tracking_trajopt_max_iter").as_int(),
        this->get_parameter("tracking_max_joint_step").as_double());
    planner_->setPlannerPeriod(1.0 / std::max(1.0, tracking_rate_hz_));

    // ---- Offline chunked planning parameters ----
    this->declare_parameter<int>("planning_chunk_size", 20);
    this->declare_parameter<int>("planning_parallel_chunks", 2);
    planner_->configureChunking(
        this->get_parameter("planning_chunk_size").as_int(),
        this->get_parameter("planning_parallel_chunks").as_int());

    // ---- Runtime-tunable planning hyperparameters (from planning_params.yaml) ----
    this->declare_parameter<std::string>("move_instruction_type", "LINEAR");
    // OMPL
    this->declare_parameter<double>("ompl_planning_time", 10.0);
    this->declare_parameter<int>("ompl_max_solutions", 5);
    this->declare_parameter<bool>("ompl_simplify", true);
    this->declare_parameter<double>("ompl_longest_valid_segment", 0.005);
    this->declare_parameter<double>("ompl_rrt_range_1", 0.05);
    this->declare_parameter<double>("ompl_rrt_range_2", 0.10);
    // TrajOptIfopt
    // Per-axis cartesian constraint coefficients
    this->declare_parameter<double>("ifopt_cart_coeff_x", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_y", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_z", 100.0);
    this->declare_parameter<double>("ifopt_cart_coeff_rx", 0.0);
    this->declare_parameter<double>("ifopt_cart_coeff_ry", 0.0);
    this->declare_parameter<double>("ifopt_cart_coeff_rz", 0.0);
    this->declare_parameter<double>("ifopt_joint_cost_coeff", 5.0);
    this->declare_parameter<double>("ifopt_coll_cost_margin", 0.02);
    this->declare_parameter<double>("ifopt_coll_cost_coeff", 500.0);
    this->declare_parameter<double>("ifopt_coll_margin_buffer", 0.02);
    // Collision evaluator: 0=DISCRETE, 1=CONTINUOUS, 2=LVS_CONTINUOUS
    this->declare_parameter<int>("ifopt_coll_eval_type", 2);
    this->declare_parameter<double>("ifopt_coll_lvs_length", 0.005);
    this->declare_parameter<double>("ifopt_smooth_vel_coeff", 0.1);
    this->declare_parameter<double>("ifopt_smooth_acc_coeff", 1.0);
    this->declare_parameter<double>("ifopt_smooth_jerk_coeff", 1.0);
    this->declare_parameter<int>("ifopt_max_iter", 300);
    this->declare_parameter<double>("ifopt_min_approx_improve", 1e-6);
    this->declare_parameter<double>("ifopt_min_trust_box_size", 1e-5);
    this->declare_parameter<double>("ifopt_initial_trust_box_size", 0.5);
    // IFOPT enable/disable flags
    this->declare_parameter<bool>("ifopt_joint_cost_enable", true);
    this->declare_parameter<bool>("ifopt_cart_constraint_enable", true);
    this->declare_parameter<bool>("ifopt_cart_cost_enable", false);
    this->declare_parameter<bool>("ifopt_coll_constraint_enable", false);
    this->declare_parameter<bool>("ifopt_coll_cost_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_vel_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_acc_enable", true);
    this->declare_parameter<bool>("ifopt_smooth_jerk_enable", true);

    {
        MotoMiniPlanning::PlanningConfig cfg;
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
        cfg.ifopt_coll_eval_type = static_cast<int>(this->get_parameter("ifopt_coll_eval_type").as_int());
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
                    "cart=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]  coll_margin=%.3f  eval=%d  lvs=%.4f",
                    instr.c_str(),
                    cfg.use_ompl_runtime ? "ON" : "OFF",
                    this->get_parameter("planning_chunk_size").as_int(),
                    this->get_parameter("planning_parallel_chunks").as_int(),
                    cfg.ifopt_cart_coeff_x, cfg.ifopt_cart_coeff_y, cfg.ifopt_cart_coeff_z,
                    cfg.ifopt_cart_coeff_rx, cfg.ifopt_cart_coeff_ry, cfg.ifopt_cart_coeff_rz,
                    cfg.ifopt_coll_cost_margin, cfg.ifopt_coll_eval_type, cfg.ifopt_coll_lvs_length);
    }

    // Progress callback: log each chunk completion during planning.
    // The full stitched trajectory is published once in startCallback after run() returns.
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
    pub_tracking_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);
    pub_tracked_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/motomini/tracked_tip_pose", 10);
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

    // ---- Tracking timer + TF poll thread + Joint state poll thread (always active; mode switched at runtime) ----
    {
        const double hz = std::max(0.1, tracking_rate_hz_);
        const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        tracking_timer_ = this->create_wall_timer(
            period, std::bind(&MotoMiniPlanningNode::trackingTick, this));
            
        tracking_stream_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&MotoMiniPlanningNode::trackingStreamTick, this));
            
        startTfPolling();
        startJointStatePolling();
        RCLCPP_INFO(this->get_logger(),
                    "Tracking ready: %.0f Hz planner, %.0f Hz TF poll, %.0f Hz joint poll, EMA=%.2f — "
                    "publish /tracking_control true to activate",
                    hz, tf_poll_rate_hz_, joint_state_poll_rate_hz_, tracking_ema_alpha_);
    }

    RCLCPP_INFO(this->get_logger(), "MotoMini Planning Node Ready.");
    RCLCPP_INFO(this->get_logger(),
                "Topics: /joint_states, /target_poses, /clear_targets, /start, /tracking_control");

    // Register runtime parameter change callback so GUI updates take effect
    // immediately without restarting the node.
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MotoMiniPlanningNode::onParameterChange, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
// Destructor — clean up TF poll thread and joint state poll thread
// ---------------------------------------------------------------------------
MotoMiniPlanningNode::~MotoMiniPlanningNode()
{
    stopTfPolling();
    stopJointStatePolling();
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

    // Initial robot pose is now captured lazily from the first /joint_states
    // message in jointStateCallback(), avoiding the URDF-default race condition.

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

// ---------------------------------------------------------------------------
// onParameterChange — propagates runtime param changes (e.g. from GUI) to planner
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
MotoMiniPlanningNode::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    MotoMiniPlanning::PlanningConfig cfg = planner_->getPlanningConfig();
    int new_chunk_size = this->get_parameter("planning_chunk_size").as_int();
    int new_parallel = this->get_parameter("planning_parallel_chunks").as_int();
    bool chunk_changed = false;

    for (const auto &p : params)
    {
        const std::string &n = p.get_name();
        try
        {
            if (n == "move_instruction_type")
                cfg.use_linear = (p.as_string() == "LINEAR");
            else if (n == "use_ompl")
                cfg.use_ompl_runtime = p.as_bool();
            else if (n == "ifopt_cart_coeff_x")
                cfg.ifopt_cart_coeff_x = p.as_double();
            else if (n == "ifopt_cart_coeff_y")
                cfg.ifopt_cart_coeff_y = p.as_double();
            else if (n == "ifopt_cart_coeff_z")
                cfg.ifopt_cart_coeff_z = p.as_double();
            else if (n == "ifopt_cart_coeff_rx")
                cfg.ifopt_cart_coeff_rx = p.as_double();
            else if (n == "ifopt_cart_coeff_ry")
                cfg.ifopt_cart_coeff_ry = p.as_double();
            else if (n == "ifopt_cart_coeff_rz")
                cfg.ifopt_cart_coeff_rz = p.as_double();
            else if (n == "ifopt_coll_eval_type")
                cfg.ifopt_coll_eval_type = static_cast<int>(p.as_int());
            else if (n == "ifopt_coll_lvs_length")
                cfg.ifopt_coll_lvs_length = p.as_double();
            else if (n == "ifopt_joint_cost_coeff")
                cfg.ifopt_joint_cost_coeff = p.as_double();
            else if (n == "ifopt_coll_cost_margin")
                cfg.ifopt_coll_cost_margin = p.as_double();
            else if (n == "ifopt_coll_cost_coeff")
                cfg.ifopt_coll_cost_coeff = p.as_double();
            else if (n == "ifopt_coll_margin_buffer")
                cfg.ifopt_coll_margin_buffer = p.as_double();
            else if (n == "ifopt_smooth_vel_coeff")
                cfg.ifopt_smooth_vel = p.as_double();
            else if (n == "ifopt_smooth_acc_coeff")
                cfg.ifopt_smooth_acc = p.as_double();
            else if (n == "ifopt_smooth_jerk_coeff")
                cfg.ifopt_smooth_jerk = p.as_double();
            else if (n == "ifopt_max_iter")
                cfg.ifopt_max_iter = static_cast<int>(p.as_int());
            else if (n == "ifopt_min_approx_improve")
                cfg.ifopt_min_approx_improve = p.as_double();
            else if (n == "ifopt_min_trust_box_size")
                cfg.ifopt_min_trust_box_size = p.as_double();
            else if (n == "ifopt_initial_trust_box_size")
                cfg.ifopt_initial_trust_box_size = p.as_double();
            else if (n == "ifopt_joint_cost_enable")
                cfg.ifopt_joint_cost_enable = p.as_bool();
            else if (n == "ifopt_cart_constraint_enable")
                cfg.ifopt_cart_constraint_enable = p.as_bool();
            else if (n == "ifopt_cart_cost_enable")
                cfg.ifopt_cart_cost_enable = p.as_bool();
            else if (n == "ifopt_coll_constraint_enable")
                cfg.ifopt_coll_constraint_enable = p.as_bool();
            else if (n == "ifopt_coll_cost_enable")
                cfg.ifopt_coll_cost_enable = p.as_bool();
            else if (n == "ifopt_smooth_vel_enable")
                cfg.ifopt_smooth_vel_enable = p.as_bool();
            else if (n == "ifopt_smooth_acc_enable")
                cfg.ifopt_smooth_acc_enable = p.as_bool();
            else if (n == "ifopt_smooth_jerk_enable")
                cfg.ifopt_smooth_jerk_enable = p.as_bool();
            else if (n == "ompl_planning_time")
                cfg.ompl_planning_time = p.as_double();
            else if (n == "ompl_max_solutions")
                cfg.ompl_max_solutions = static_cast<int>(p.as_int());
            else if (n == "ompl_simplify")
                cfg.ompl_simplify = p.as_bool();
            else if (n == "ompl_rrt_range_1")
                cfg.ompl_rrt_range_1 = p.as_double();
            else if (n == "ompl_rrt_range_2")
                cfg.ompl_rrt_range_2 = p.as_double();
            else if (n == "planning_chunk_size")
            {
                new_chunk_size = static_cast<int>(p.as_int());
                chunk_changed = true;
            }
            else if (n == "planning_parallel_chunks")
            {
                new_parallel = static_cast<int>(p.as_int());
                chunk_changed = true;
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

    RCLCPP_INFO(this->get_logger(),
                "[ParamUpdate] mode=%s  ompl=%s  cart=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]  "
                "eval=%d  lvs=%.4f  chunk=%d",
                cfg.use_linear ? "LINEAR" : "FREESPACE",
                cfg.use_ompl_runtime ? "ON" : "OFF",
                cfg.ifopt_cart_coeff_x, cfg.ifopt_cart_coeff_y, cfg.ifopt_cart_coeff_z,
                cfg.ifopt_cart_coeff_rx, cfg.ifopt_cart_coeff_ry, cfg.ifopt_cart_coeff_rz,
                cfg.ifopt_coll_eval_type, cfg.ifopt_coll_lvs_length,
                new_chunk_size);
    return result;
}
