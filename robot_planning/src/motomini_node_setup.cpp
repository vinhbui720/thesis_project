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
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_common/resource_locator.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>

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
    
    // ---- Tracking/MPC parameters ----
    this->declare_parameter<int>("tracking_horizon", 10);
    this->declare_parameter<double>("tracking_dt", 0.02);
    this->declare_parameter<double>("tracking_w_cart", 50.0);
    this->declare_parameter<double>("tracking_w_vel", 5.0);
    this->declare_parameter<double>("tracking_w_acc", 0.5);
    this->declare_parameter<double>("tracking_d_safe", 0.01);
    this->declare_parameter<int>("tracking_coll_type", 0); // 0=DISCRETE, 1=CONTINUOUS
    this->declare_parameter<int>("tracking_max_iter", 10);
    this->declare_parameter<double>("tracking_qp_reg", 0.10);
    this->declare_parameter<double>("repulsion_lookahead", 0.02);
    this->declare_parameter<double>("repulsion_safety", 0.005);
    this->declare_parameter<double>("repulsion_max_speed", 0.20);
    this->declare_parameter<double>("relax_distance", 0.01);
    this->declare_parameter<double>("relax_z_min_factor", 0.05);
    this->declare_parameter<double>("relax_rot_min_factor", 0.10);

    mpc_horizon_n_ = this->get_parameter("tracking_horizon").as_int();
    mpc_dt_ = this->get_parameter("tracking_dt").as_double();
    mpc_w_cart_ = this->get_parameter("tracking_w_cart").as_double();
    mpc_w_vel_ = this->get_parameter("tracking_w_vel").as_double();
    mpc_w_acc_ = this->get_parameter("tracking_w_acc").as_double();
    mpc_d_safe_ = this->get_parameter("tracking_d_safe").as_double();
    mpc_max_iter_ = this->get_parameter("tracking_max_iter").as_int();
    mpc_coll_type_ = this->get_parameter("tracking_coll_type").as_int();
    qp_velocity_reg_ = this->get_parameter("tracking_qp_reg").as_double();
    repulsion_lookahead_ = this->get_parameter("repulsion_lookahead").as_double();
    repulsion_safety_ = this->get_parameter("repulsion_safety").as_double();
    repulsion_max_speed_ = this->get_parameter("repulsion_max_speed").as_double();
    relax_distance_ = this->get_parameter("relax_distance").as_double();
    relax_z_min_factor_ = this->get_parameter("relax_z_min_factor").as_double();
    relax_rot_min_factor_ = this->get_parameter("relax_rot_min_factor").as_double();

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

    // ---- Setup MPC State for Tracking Mode ----
    manip_ = env_->getKinematicGroup(manipulator_group);
    if (!manip_)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to find KinematicGroup %s", manipulator_group.c_str());
    }
    else
    {
        int n_dof = manip_->numJoints();
        horizon_joints_.assign(static_cast<size_t>(mpc_horizon_n_), Eigen::VectorXd::Zero(n_dof));
        current_joints_ = Eigen::VectorXd::Zero(n_dof);
        mpc_collision_cache_ = std::make_shared<trajopt_ifopt::CollisionCache>(static_cast<size_t>(mpc_horizon_n_) * 4);
        joint_limits_ = manip_->getLimits().joint_limits;

        std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
        std::filesystem::path config_path(
            locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")
                ->getFilePath());
        task_factory_ = std::make_unique<tesseract_planning::TaskComposerPluginFactory>(config_path, *locator);
        task_executor_ = task_factory_->createTaskComposerExecutor("TaskflowExecutor");

        // DLS-IK tracking is simple and needs no pre-initialization
    }
    current_target_pose_ = Eigen::Isometry3d::Identity();
    target_velocity_linear_.setZero();
    target_velocity_angular_.setZero();

    // ---- Offline chunked planning parameters ----
    this->declare_parameter<int>("planning_chunk_size", 20);
    this->declare_parameter<int>("planning_parallel_chunks", 2);
    planner_->configureChunking(
        this->get_parameter("planning_chunk_size").as_int(),
        this->get_parameter("planning_parallel_chunks").as_int());

    // ---- Runtime-tunable planning hyperparameters (from planning_params.yaml) ----
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

    sub_tracking_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tracking_target_pose", 10,
        std::bind(&MotoMiniPlanningNode::targetPoseCallback, this, std::placeholders::_1));

    sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start", 10,
        std::bind(&MotoMiniPlanningNode::startCallback, this, std::placeholders::_1));

    sub_clear_ = this->create_subscription<std_msgs::msg::Bool>(
        "/clear_targets", 10,
        std::bind(&MotoMiniPlanningNode::clearCallback, this, std::placeholders::_1));

    sub_tracking_control_ = this->create_subscription<std_msgs::msg::Bool>(
        "/tracking_control", 10,
        std::bind(&MotoMiniPlanningNode::trackingControlCallback, this, std::placeholders::_1));

    pub_status_ = this->create_publisher<std_msgs::msg::String>("/optimization_status", 10);
    pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/path_command", 10);
    pub_tracking_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);
    pub_tracked_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/motomini/tracked_tip_pose", 10);
    pub_online_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motomini/online_joint_command", 100);

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

    mpc_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(mpc_dt_ * 1000.0)),
        std::bind(&MotoMiniPlanningNode::mpcTimerCallback, this));

    // PERFORMANCE: Cache the tracking task node once
    if (task_factory_)
    {
        mpc_task_ = task_factory_->createTaskComposerNode("TrajOptIfoptPipeline");
    }

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MotoMiniPlanningNode::onParameterChange, this, std::placeholders::_1));
}

MotoMiniPlanningNode::~MotoMiniPlanningNode()
{
    // No background threads to stop — reactive avoidance runs inline at 30 Hz
}

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

    RCLCPP_INFO(this->get_logger(), "SRDF path: %s", srdf_xml_.c_str());

    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    env_ = std::make_shared<tesseract_environment::Environment>();
    if (!env_->init(urdf_xml_, srdf_xml_, locator))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract environment.");
        return false;
    }

    // DEBUG: Verify SRDF loaded by checking ACM
    auto acm = env_->getAllowedCollisionMatrix();
    if (acm)
    {
        size_t num_disabled = 0;
        // Count disabled collision pairs (this is an estimate; Tesseract doesn't expose exact count)
        RCLCPP_INFO(this->get_logger(), "✓ ACM initialized from SRDF");

        // Log a few examples of disabled pairs
        const auto &scene_graph = env_->getSceneGraph();
        const auto &links = scene_graph->getLinks();
        std::vector<std::pair<std::string, std::string>> examples;
        for (const auto &link1 : links)
        {
            for (const auto &link2 : links)
            {
                if (link1->getName() < link2->getName() &&
                    acm->isCollisionAllowed(link1->getName(), link2->getName()))
                {
                    examples.push_back({link1->getName(), link2->getName()});
                    if (examples.size() >= 3)
                        break;
                }
            }
            if (examples.size() >= 3)
                break;
        }
        if (!examples.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Example disabled collisions:");
            for (const auto &[l1, l2] : examples)
                RCLCPP_INFO(this->get_logger(), "  %s ↔ %s", l1.c_str(), l2.c_str());
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ ACM is null — SRDF may not have loaded properly!");
    }

    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());

    // Note: Obstacle is dynamically added by ROSEnvironmentMonitor when it sees
    // the obstacle_simulator's TF broadcasts and loads obstacle.urdf.xacro from
    // the package resource locator.
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
            else if (n == "tracking_horizon") mpc_horizon_n_ = static_cast<int>(p.as_int());
            else if (n == "tracking_dt") mpc_dt_ = p.as_double();
            else if (n == "tracking_w_cart") mpc_w_cart_ = p.as_double();
            else if (n == "tracking_w_vel") mpc_w_vel_ = p.as_double();
            else if (n == "tracking_w_acc") mpc_w_acc_ = p.as_double();
            else if (n == "tracking_d_safe") mpc_d_safe_ = p.as_double();
            else if (n == "tracking_coll_type") mpc_coll_type_ = static_cast<int>(p.as_int());
            else if (n == "tracking_max_iter") mpc_max_iter_ = static_cast<int>(p.as_int());
            else if (n == "tracking_qp_reg") qp_velocity_reg_ = p.as_double();
            else if (n == "repulsion_lookahead") repulsion_lookahead_ = p.as_double();
            else if (n == "repulsion_safety") repulsion_safety_ = p.as_double();
            else if (n == "repulsion_max_speed") repulsion_max_speed_ = p.as_double();
            else if (n == "relax_distance") relax_distance_ = p.as_double();
            else if (n == "relax_z_min_factor") relax_z_min_factor_ = p.as_double();
            else if (n == "relax_rot_min_factor") relax_rot_min_factor_ = p.as_double();
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
