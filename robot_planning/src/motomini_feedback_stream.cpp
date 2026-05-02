/*
 * motomini_feedback_stream.cpp
 *
 * ROS2 Node: Real-time Cartesian pose following for MotoMini 6-DOF manipulator.
 * Uses an adaptive Cartesian admittance-inspired velocity controller with
 * Singularity-Robust (SR) inverse Jacobian mapping.
 *
 * State Machine:
 *   STATE_IDLE       – no publishing, sync joint state
 *   STATE_INIT       – move to initial pose smoothly (adaptive warm-up)
 *   STATE_POSE_FOLLOW – track streaming desired pose
 *   STATE_STOP       – no publishing, velocity zeroed
 *
 * Topics:
 *   Sub:  /joint_states                    (sensor_msgs/JointState)
 *   Sub:  /motomini/target_pose            (geometry_msgs/PoseStamped)
 *   Sub:  /motomini/target_vel             (geometry_msgs/Twist — linear=world, angular=body)
 *   Sub:  /motomini/collision_wrench       (geometry_msgs/WrenchStamped)
 *   Sub:  /pose_following/init_pose        (geometry_msgs/PoseStamped, latched)
 *   Pub:  /joint_path_command              (trajectory_msgs/JointTrajectory – arm init, one-time)
 *   Pub:  joint_command                    (trajectory_msgs/JointTrajectory – streaming)
 *   Pub:  /motomini/feedback               (geometry_msgs/Twist — current EE xyz+rpy)
 *   Pub:  /motomini/feedback_vel           (geometry_msgs/Twist — current Cartesian J·θ̇)
 *
 * Services:
 *   /pose_following/start      → reset to STATE_IDLE
 *   /pose_following/stop       → STATE_STOP
 *   /pose_following/init_start → STATE_INIT (requires init_pose to be set first)
 */

// ============================================================
// SECTION 1 – CONFIGURATION
// ============================================================

#define NODE_RATE 50 // Control loop rate [Hz]
#define NUMBER_OF_JOINT 6

// --- Joint Position Limits (degrees) ---
#define JOINT_1_S_UPPER_LIMIT_DEG 170
#define JOINT_1_S_LOWER_LIMIT_DEG -170
#define JOINT_2_L_UPPER_LIMIT_DEG 90
#define JOINT_2_L_LOWER_LIMIT_DEG -85
#define JOINT_3_U_UPPER_LIMIT_DEG 120
#define JOINT_3_U_LOWER_LIMIT_DEG -175
#define JOINT_4_R_UPPER_LIMIT_DEG 140
#define JOINT_4_R_LOWER_LIMIT_DEG -140
#define JOINT_5_B_UPPER_LIMIT_DEG 210
#define JOINT_5_B_LOWER_LIMIT_DEG -30
#define JOINT_6_T_UPPER_LIMIT_DEG 360
#define JOINT_6_T_LOWER_LIMIT_DEG -360

// --- Joint Position Limits (radians) ---
#define JOINT_1_S_UPPER_LIMIT_RAD (170.0 * M_PI / 180.0)
#define JOINT_1_S_LOWER_LIMIT_RAD (-170.0 * M_PI / 180.0)
#define JOINT_2_L_UPPER_LIMIT_RAD (90.0 * M_PI / 180.0)
#define JOINT_2_L_LOWER_LIMIT_RAD (-85.0 * M_PI / 180.0)
#define JOINT_3_U_UPPER_LIMIT_RAD (120.0 * M_PI / 180.0)
#define JOINT_3_U_LOWER_LIMIT_RAD (-175.0 * M_PI / 180.0)
#define JOINT_4_R_UPPER_LIMIT_RAD (140.0 * M_PI / 180.0)
#define JOINT_4_R_LOWER_LIMIT_RAD (-140.0 * M_PI / 180.0)
#define JOINT_5_B_UPPER_LIMIT_RAD (210.0 * M_PI / 180.0)
#define JOINT_5_B_LOWER_LIMIT_RAD (-30.0 * M_PI / 180.0)
#define JOINT_6_T_UPPER_LIMIT_RAD (360.0 * M_PI / 180.0)
#define JOINT_6_T_LOWER_LIMIT_RAD (-360.0 * M_PI / 180.0)

// --- Joint Velocity Limits [rad/s] – (315°/s, 315°/s, 420°/s, 600°/s, 600°/s, 600°/s) ---
#define JOINT_1_S_VEL_LIMIT_RADSEC (M_PI * 7.0 / 4.0)
#define JOINT_2_L_VEL_LIMIT_RADSEC (M_PI * 7.0 / 4.0)
#define JOINT_3_U_VEL_LIMIT_RADSEC (M_PI * 7.0 / 3.0)
#define JOINT_4_R_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)
#define JOINT_5_B_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)
#define JOINT_6_T_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)
#define SAFETY_VELOCITY_ALPHA 0.9 // fraction of hardware limit to use

// --- Legacy PD Parameters (accepted for backward-compatible YAML files) ---
#define DEFAULT_LEGACY_KP_MAX 3.5
#define DEFAULT_LEGACY_KO_MAX 2.5
#define DEFAULT_LEGACY_KDP 0.25
#define DEFAULT_LEGACY_KDO 0.25

// --- Adaptive admittance defaults (override via ROS params) ---
#define DEFAULT_M_POS_MIN 0.5
#define DEFAULT_M_POS_MAX 5.0
#define DEFAULT_K_POS_MIN 5.0
#define DEFAULT_K_POS_MAX 50.0
#define DEFAULT_ZETA_POS 0.9
#define DEFAULT_M_ORI_MIN 0.2
#define DEFAULT_M_ORI_MAX 2.0
#define DEFAULT_K_ORI_MIN 2.0
#define DEFAULT_K_ORI_MAX 20.0
#define DEFAULT_ZETA_ORI 0.9
#define DEFAULT_ADAPTIVE_LAMBDA 1.0
#define DEFAULT_ADAPTIVE_ALPHA_POS 30.0
#define DEFAULT_ADAPTIVE_ALPHA_ORI 6.0
#define DEFAULT_MAX_CART_LINEAR_VEL 0.5
#define DEFAULT_MAX_CART_ANGULAR_VEL 1.5

// --- Singularity-Robust inverse defaults ---
#define DEFAULT_W0 0.01 // singularity threshold (manipulability)
#define DEFAULT_K0 0.01 // SR damping coefficient

// --- Safety ---
#define POSITION_ERROR_THRESHOLD 0.0005               // [m] init completion threshold
#define SAFETY_JOINT_PADDING_RAD (5.0 * M_PI / 180.0) // [rad] padding from hard limits
#define POSE_TIMEOUT_SEC 3.0                          // POSE_FOLLOW → IDLE if no pose [s]
#define TARGET_VEL_TIMEOUT_SEC 0.5                    // stop integrating target_vel if stale [s]
#define ARM_PRE_DELAY_S 0.5                           // wait before sending path command
#define ARM_POST_DELAY_S 1.0                          // wait after path command before seeding

// ============================================================
// SECTION 2 – INCLUDES
// ============================================================

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_rosutils/utils.h>

// ============================================================
// SECTION 3 – NODE CLASS
// ============================================================

class MotoMiniFeedbackStreamNode : public rclcpp::Node
{
public:
    MotoMiniFeedbackStreamNode()
        : rclcpp::Node("motomini_feedback_stream")
    {
        // ----- Declare ROS parameters -----
        this->declare_parameter<std::string>("robot_description",
                                             "package://robot_planning/urdf/motoman_motomini.urdf");
        this->declare_parameter<std::string>("robot_description_semantic",
                                             "package://robot_planning/urdf/motoman_motomini.srdf");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("ee_link", "tool0");
        this->declare_parameter<double>("rate_hz", NODE_RATE);
        // Legacy PD params are declared so older YAML files still load. The
        // active controller below uses the adaptive m/k/zeta parameters.
        this->declare_parameter<double>("kp_max", DEFAULT_LEGACY_KP_MAX);
        this->declare_parameter<double>("ko_max", DEFAULT_LEGACY_KO_MAX);
        this->declare_parameter<double>("kdp", DEFAULT_LEGACY_KDP);
        this->declare_parameter<double>("kdo", DEFAULT_LEGACY_KDO);
        this->declare_parameter<double>("w0", DEFAULT_W0);
        this->declare_parameter<double>("k0", DEFAULT_K0);
        this->declare_parameter<double>("theta_d_lim", 3.14);
        this->declare_parameter<bool>("enable_seed", false);

        // ----- Adaptive impedance/admittance parameters -----
        // Translational virtual mass / stiffness (clamps adapt with error).
        this->declare_parameter<double>("m_pos_min", DEFAULT_M_POS_MIN);
        this->declare_parameter<double>("m_pos_max", DEFAULT_M_POS_MAX);
        this->declare_parameter<double>("k_pos_min", DEFAULT_K_POS_MIN);
        this->declare_parameter<double>("k_pos_max", DEFAULT_K_POS_MAX);
        this->declare_parameter<double>("zeta_pos", DEFAULT_ZETA_POS);
        // Orientation virtual mass / stiffness.
        this->declare_parameter<double>("m_ori_min", DEFAULT_M_ORI_MIN);
        this->declare_parameter<double>("m_ori_max", DEFAULT_M_ORI_MAX);
        this->declare_parameter<double>("k_ori_min", DEFAULT_K_ORI_MIN);
        this->declare_parameter<double>("k_ori_max", DEFAULT_K_ORI_MAX);
        this->declare_parameter<double>("zeta_ori", DEFAULT_ZETA_ORI);
        // Adaptation profile.
        this->declare_parameter<double>("adaptive_lambda", DEFAULT_ADAPTIVE_LAMBDA);
        this->declare_parameter<double>("adaptive_alpha_pos", DEFAULT_ADAPTIVE_ALPHA_POS);
        this->declare_parameter<double>("adaptive_alpha_ori", DEFAULT_ADAPTIVE_ALPHA_ORI);
        // Cartesian-velocity safety clamps applied to the integrator state.
        this->declare_parameter<double>("max_cart_linear_vel", DEFAULT_MAX_CART_LINEAR_VEL);
        this->declare_parameter<double>("max_cart_angular_vel", DEFAULT_MAX_CART_ANGULAR_VEL);
        this->declare_parameter<double>("collision_wrench_timeout_sec", 0.2);

        // Close-work collision safety: distance/normal-driven projection that
        // sits on top of the soft repulsive wrench. Disabled by default in the
        // sense that, with no constraint topic published, gamma stays at 0.
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
        this->declare_parameter<double>("velocity_filter_cutoff_hz", 15.0);
        this->declare_parameter<double>("max_cart_linear_acc", 0.8);
        this->declare_parameter<double>("max_cart_angular_acc", 2.5);
        this->declare_parameter<double>("measured_cart_linear_vel_limit", 1.0);
        this->declare_parameter<double>("measured_cart_angular_vel_limit", 3.0);
        // Mode selector: set true for velocity-only joystick (integrates target_vel into
        // desired_pose_); set false when an external node streams /motomini/target_pose
        // so that target_vel is used as feedforward only and not double-applied.
        this->declare_parameter<bool>("integrate_target_vel_to_pose", true);

        // ----- Read parameters -----
        this->get_parameter("robot_description", urdf_xml_);
        this->get_parameter("robot_description_semantic", srdf_xml_);
        manipulator_group_ = this->get_parameter("manipulator_group").as_string();
        base_link_ = this->get_parameter("base_link").as_string();
        ee_link_ = this->get_parameter("ee_link").as_string();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
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
        collision_wrench_timeout_sec_ = this->get_parameter("collision_wrench_timeout_sec").as_double();

        enable_collision_projection_ = this->get_parameter("enable_collision_projection").as_bool();
        collision_goal_suppression_ = this->get_parameter("collision_goal_suppression").as_bool();
        collision_guard_distance_ = this->get_parameter("collision_guard_distance").as_double();
        collision_task_distance_ = this->get_parameter("collision_task_distance").as_double();
        collision_stop_distance_ = this->get_parameter("collision_stop_distance").as_double();
        collision_projection_max_gamma_ =
            this->get_parameter("collision_projection_max_gamma").as_double();
        collision_constraint_timeout_sec_ =
            this->get_parameter("collision_constraint_timeout_sec").as_double();
        collision_force_scale_ = this->get_parameter("collision_force_scale").as_double();
        collision_force_max_ = this->get_parameter("collision_force_max").as_double();
        real_robot_ = this->get_parameter("real_robot").as_bool();
        velocity_filter_cutoff_hz_ =
            this->get_parameter("velocity_filter_cutoff_hz").as_double();
        max_cart_linear_acc_ = this->get_parameter("max_cart_linear_acc").as_double();
        max_cart_angular_acc_ = this->get_parameter("max_cart_angular_acc").as_double();
        measured_cart_linear_vel_limit_ =
            this->get_parameter("measured_cart_linear_vel_limit").as_double();
        measured_cart_angular_vel_limit_ =
            this->get_parameter("measured_cart_angular_vel_limit").as_double();
        integrate_target_vel_to_pose_ =
            this->get_parameter("integrate_target_vel_to_pose").as_bool();

        collision_stop_distance_ = std::max(0.0, collision_stop_distance_);
        collision_task_distance_ =
            std::max(collision_stop_distance_, collision_task_distance_);
        collision_guard_distance_ =
            std::max(collision_task_distance_, collision_guard_distance_);
        collision_projection_max_gamma_ =
            std::clamp(collision_projection_max_gamma_, 0.0, 1.0);
        collision_force_scale_ = std::max(0.0, collision_force_scale_);
        collision_force_max_ = std::max(0.0, collision_force_max_);
        velocity_filter_cutoff_hz_ = std::max(1.0, velocity_filter_cutoff_hz_);
        max_cart_linear_acc_ = std::max(0.0, max_cart_linear_acc_);
        max_cart_angular_acc_ = std::max(0.0, max_cart_angular_acc_);
        measured_cart_linear_vel_limit_ =
            std::max(0.0, measured_cart_linear_vel_limit_);
        measured_cart_angular_vel_limit_ =
            std::max(0.0, measured_cart_angular_vel_limit_);

        const auto &overrides =
            this->get_node_parameters_interface()->get_parameter_overrides();
        const bool legacy_kp_override = overrides.find("kp_max") != overrides.end();
        const bool legacy_ko_override = overrides.find("ko_max") != overrides.end();
        const bool new_k_pos_max_override = overrides.find("k_pos_max") != overrides.end();
        const bool new_k_ori_max_override = overrides.find("k_ori_max") != overrides.end();

        // Backward compatibility: if an old YAML only supplies kp_max/ko_max,
        // map those values proportionally into the new maximum stiffness terms.
        // New k_pos_max/k_ori_max parameters always take precedence.
        if (legacy_kp_override && !new_k_pos_max_override)
        {
            const double legacy_kp = this->get_parameter("kp_max").as_double();
            k_pos_max_ = DEFAULT_K_POS_MAX *
                         std::max(0.0, legacy_kp) / DEFAULT_LEGACY_KP_MAX;
        }
        if (legacy_ko_override && !new_k_ori_max_override)
        {
            const double legacy_ko = this->get_parameter("ko_max").as_double();
            k_ori_max_ = DEFAULT_K_ORI_MAX *
                         std::max(0.0, legacy_ko) / DEFAULT_LEGACY_KO_MAX;
        }
        sanitizeAdaptiveParameters();

        // ----- Kinematics -----
        if (!initializeKinematics())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Tesseract kinematics. Check if URDF/SRDF is valid.");
            throw std::runtime_error("Tesseract kinematics initialization failed");
        }

        // ----- Publishers -----
        // One-time arm controller initialisation burst
        pub_path_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);
        // Real-time streaming control
        pub_joint_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_command", 10);

        // Current EE pose (xyz + rpy) for plotting / comparison with target.
        // linear  = (x, y, z)  [m]
        // angular = (roll, pitch, yaw)  [rad]   ZYX intrinsic
        pub_feedback_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/motomini/feedback", 10);

        // Current Cartesian velocity (J·θ̇) from joint feedback.
        // linear  = (vx, vy, vz)  [m/s]      base frame
        // angular = (ωx, ωy, ωz)  [rad/s]    base frame
        pub_feedback_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/motomini/feedback_vel", 10);

        // ----- Subscribers -----
        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&MotoMiniFeedbackStreamNode::jointStateCallback, this, std::placeholders::_1));

        sub_desired_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/motomini/target_pose", 1,
            std::bind(&MotoMiniFeedbackStreamNode::desiredPoseCallback, this, std::placeholders::_1));

        sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_following/init_pose", 1,
            std::bind(&MotoMiniFeedbackStreamNode::initPoseCallback, this, std::placeholders::_1));

        sub_target_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/motomini/target_vel", 10,
            std::bind(&MotoMiniFeedbackStreamNode::targetVelCallback, this, std::placeholders::_1));

        sub_collision_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/motomini/collision_wrench", 10,
            std::bind(&MotoMiniFeedbackStreamNode::collisionWrenchCallback, this, std::placeholders::_1));

        sub_collision_distance_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motomini/collision_distance", 10,
            std::bind(&MotoMiniFeedbackStreamNode::collisionDistanceCallback, this, std::placeholders::_1));

        sub_collision_normal_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/motomini/collision_normal", 10,
            std::bind(&MotoMiniFeedbackStreamNode::collisionNormalCallback, this, std::placeholders::_1));

        // ----- Services -----
        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/start",
            std::bind(&MotoMiniFeedbackStreamNode::startCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/stop",
            std::bind(&MotoMiniFeedbackStreamNode::stopCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_init_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/init_start",
            std::bind(&MotoMiniFeedbackStreamNode::initStartCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ----- Internal state -----
        state_ = STATE_IDLE;
        last_state_ = STATE_IDLE;
        is_init_done_ = false;
        arm_init_sent_ = false;
        has_desired_pose_ = false;
        has_init_pose_ = false;
        e_p_.setZero();
        e_o_.setZero();
        t_start_ = this->now();
        t_last_ = this->now();
        t_last_pose_cb_ = this->now();
        t_last_target_vel_cb_ = this->now();
        t_last_collision_wrench_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        latest_target_vel_.setZero();
        latest_collision_wrench_.setZero();

        // ----- Control loop -----
        auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
        timer_ = this->create_wall_timer(
            period_ns,
            std::bind(&MotoMiniFeedbackStreamNode::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "motomini_feedback_stream ready. group=%s ee=%s rate=%.0f Hz",
                    manipulator_group_.c_str(), ee_link_.c_str(), rate_hz_);
        RCLCPP_INFO(this->get_logger(),
                    "Adaptive admittance: Mpos[%.3f, %.3f] Kpos[%.3f, %.3f] "
                    "Mori[%.3f, %.3f] Kori[%.3f, %.3f]",
                    m_pos_min_, m_pos_max_, k_pos_min_, k_pos_max_,
                    m_ori_min_, m_ori_max_, k_ori_min_, k_ori_max_);
        RCLCPP_INFO(this->get_logger(),
                    "Damping/adaptation: zeta_pos=%.2f zeta_ori=%.2f "
                    "lambda=%.2f alpha_pos=%.2f alpha_ori=%.2f w0=%.6f k0=%.6f",
                    zeta_pos_, zeta_ori_, adaptive_lambda_,
                    adaptive_alpha_pos_, adaptive_alpha_ori_, w0_, k0_);
        RCLCPP_INFO(this->get_logger(), "enable_seed=%s", enable_seed_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
                    "Collision safety: projection=%s goal_suppress=%s "
                    "guard=%.3f task=%.3f stop=%.3f max_gamma=%.2f "
                    "force_scale=%.2f force_max=%.2f",
                    enable_collision_projection_ ? "on" : "off",
                    collision_goal_suppression_ ? "on" : "off",
                    collision_guard_distance_, collision_task_distance_,
                    collision_stop_distance_, collision_projection_max_gamma_,
                    collision_force_scale_, collision_force_max_);
    }

private:
    // ============================================================
    // State machine enum (values match ROS1 reference)
    // ============================================================
    enum State
    {
        STATE_IDLE = 0,
        STATE_POSE_FOLLOW = 1,
        STATE_STOP = 2,
        STATE_INIT = 3,
        STATE_ARMING = 4,
    };

    // ============================================================
    // KINEMATICS INITIALISATION
    // ============================================================
    bool initializeKinematics()
    {
        auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        env_ = std::make_shared<tesseract_environment::Environment>();
        if (!env_->init(urdf_xml_, srdf_xml_, locator))
            return false;

        manip_ = env_->getKinematicGroup(manipulator_group_);
        if (!manip_)
            return false;

        joint_names_ = manip_->getJointNames();
        qdot_filtered_ =
            Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
        return !joint_names_.empty();
    }

    // ============================================================
    // MATH UTILITIES
    // ============================================================

    // Damped pseudo-inverse (Moore-Penrose)
    static Eigen::MatrixXd calcPseudoInverse(const Eigen::MatrixXd &J)
    {
        return J.transpose() * (J * J.transpose()).inverse();
    }

    // Singularity-Robust (SR) inverse with Nakamura damping
    static Eigen::MatrixXd calcSrInverse(const Eigen::MatrixXd &J,
                                         double w, double w0, double k0)
    {
        const double w0_safe = std::max(1e-9, w0);
        const double k = (w < w0_safe) ? k0 * std::pow(1.0 - w / w0_safe, 2.0) : 0.0;
        const Eigen::Index m = J.rows();
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);
        return J.transpose() * (J * J.transpose() + k * I).inverse();
    }

    void sanitizeAdaptiveParameters()
    {
        m_pos_min_ = std::max(1e-6, m_pos_min_);
        m_pos_max_ = std::max(1e-6, m_pos_max_);
        if (m_pos_min_ > m_pos_max_)
            std::swap(m_pos_min_, m_pos_max_);

        k_pos_min_ = std::max(1e-6, k_pos_min_);
        k_pos_max_ = std::max(1e-6, k_pos_max_);
        if (k_pos_min_ > k_pos_max_)
            std::swap(k_pos_min_, k_pos_max_);

        m_ori_min_ = std::max(1e-6, m_ori_min_);
        m_ori_max_ = std::max(1e-6, m_ori_max_);
        if (m_ori_min_ > m_ori_max_)
            std::swap(m_ori_min_, m_ori_max_);

        k_ori_min_ = std::max(1e-6, k_ori_min_);
        k_ori_max_ = std::max(1e-6, k_ori_max_);
        if (k_ori_min_ > k_ori_max_)
            std::swap(k_ori_min_, k_ori_max_);

        zeta_pos_ = std::max(0.0, zeta_pos_);
        zeta_ori_ = std::max(0.0, zeta_ori_);
        adaptive_lambda_ = std::max(0.0, adaptive_lambda_);
        adaptive_alpha_pos_ = std::max(0.0, adaptive_alpha_pos_);
        adaptive_alpha_ori_ = std::max(0.0, adaptive_alpha_ori_);
        max_cart_linear_vel_ = std::max(0.0, max_cart_linear_vel_);
        max_cart_angular_vel_ = std::max(0.0, max_cart_angular_vel_);
        velocity_filter_cutoff_hz_ = std::max(1.0, velocity_filter_cutoff_hz_);
        max_cart_linear_acc_ = std::max(0.0, max_cart_linear_acc_);
        max_cart_angular_acc_ = std::max(0.0, max_cart_angular_acc_);
        measured_cart_linear_vel_limit_ =
            std::max(0.0, measured_cart_linear_vel_limit_);
        measured_cart_angular_vel_limit_ =
            std::max(0.0, measured_cart_angular_vel_limit_);
        w0_ = std::max(1e-9, w0_);
        k0_ = std::max(0.0, k0_);
    }

    // Orientation error: rotation matrix → axis-angle vector
    static Eigen::Vector3d orientationError(const Eigen::Matrix3d &des_R,
                                            const Eigen::Matrix3d &cur_R)
    {
        Eigen::Matrix3d err_R = des_R * cur_R.transpose();
        double trace = err_R.trace();
        double angle = std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0));
        if (std::abs(angle) < 1e-8)
            return Eigen::Vector3d::Zero();
        Eigen::Vector3d axis(
            err_R(2, 1) - err_R(1, 2),
            err_R(0, 2) - err_R(2, 0),
            err_R(1, 0) - err_R(0, 1));
        axis /= (2.0 * std::sin(angle));
        return angle * axis;
    }

    // ============================================================
    // JOINT-STATE HELPERS
    // ============================================================

    // Extract joint positions in the kinematic group's order
    bool currentJointVector(Eigen::VectorXd &q) const
    {
        if (!last_joint_state_)
            return false;
        q.resize(static_cast<Eigen::Index>(joint_names_.size()));
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(),
                                joint_names_[i]);
            if (it == last_joint_state_->name.end())
                return false;
            q[static_cast<Eigen::Index>(i)] =
                last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
        }
        return true;
    }

    Eigen::VectorXd filterJointVelocity(const Eigen::VectorXd &qdot_raw, double dt)
    {
        if (qdot_raw.size() == 0 || !qdot_raw.allFinite())
            return Eigen::VectorXd::Zero(qdot_raw.size());

        const rclcpp::Time now = this->now();
        if (qdot_filtered_.size() != qdot_raw.size())
        {
            qdot_filtered_ = qdot_raw;
            first_velocity_read_ = false;
            t_last_velocity_filter_update_ = now;
            have_velocity_filter_update_ = true;
            return qdot_filtered_;
        }

        if (!first_velocity_read_ && have_velocity_filter_update_)
        {
            const double cache_age = (now - t_last_velocity_filter_update_).seconds();
            const double same_cycle_window = 0.5 / std::max(1.0, rate_hz_);
            if (cache_age >= 0.0 && cache_age < same_cycle_window)
                return qdot_filtered_;
        }

        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
        const double max_cutoff = std::max(1.0, 0.45 * std::max(1.0, rate_hz_));
        const double cutoff =
            std::clamp(velocity_filter_cutoff_hz_, 1.0, max_cutoff);
        const double rc = 1.0 / (2.0 * M_PI * cutoff);
        const double alpha = dt_safe / (rc + dt_safe);

        if (first_velocity_read_)
        {
            qdot_filtered_ = qdot_raw;
            first_velocity_read_ = false;
        }
        else
        {
            qdot_filtered_ = alpha * qdot_raw + (1.0 - alpha) * qdot_filtered_;
        }

        t_last_velocity_filter_update_ = now;
        have_velocity_filter_update_ = true;
        return qdot_filtered_;
    }

    bool getMeasuredJointVelocity(const Eigen::VectorXd &q,
                                  double dt_hint,
                                  Eigen::VectorXd &qdot_out,
                                  Eigen::VectorXd &q_prev,
                                  rclcpp::Time &t_prev_q,
                                  bool &have_q_prev)
    {
        qdot_out = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

        bool velocity_valid = false;
        // real_robot=true  → try joint_state hardware velocity first (falls back if invalid)
        // real_robot=false → always use pose-differentiation; hardware vel is ignored
        if (real_robot_ &&
            last_joint_state_ &&
            last_joint_state_->velocity.size() >= joint_names_.size())
        {
            Eigen::VectorXd qdot_driver(static_cast<Eigen::Index>(joint_names_.size()));
            velocity_valid = true;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto it = std::find(last_joint_state_->name.begin(),
                                    last_joint_state_->name.end(),
                                    joint_names_[i]);
                if (it == last_joint_state_->name.end())
                {
                    velocity_valid = false;
                    break;
                }

                const size_t idx =
                    static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
                if (idx >= last_joint_state_->velocity.size())
                {
                    velocity_valid = false;
                    break;
                }

                const double v = last_joint_state_->velocity[idx];
                if (!std::isfinite(v))
                {
                    velocity_valid = false;
                    break;
                }

                qdot_driver[static_cast<Eigen::Index>(i)] = v;
            }

            if (velocity_valid && qdot_driver.allFinite())
            {
                qdot_out = qdot_driver;
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                //                      "[VEL] real_robot=true → using hardware joint_state velocity.");
            }
        }

        const rclcpp::Time now = this->now();
        const double dt_prev = have_q_prev ? (now - t_prev_q).seconds() : dt_hint;
        if (!velocity_valid)
        {
            if (real_robot_)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "[VEL] real_robot=true but hardware velocity invalid — falling back to pose diff.");
            else
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "[VEL] real_robot=false → using pose-differentiation velocity.");
            if (have_q_prev && q_prev.size() == q.size() && dt_prev > 1e-6)
                qdot_out = (q - q_prev) / dt_prev;
            else
                qdot_out.setZero();
        }

        q_prev = q;
        t_prev_q = now;
        have_q_prev = true;

        const double filter_dt =
            (dt_prev > 1e-6 && dt_prev < 1.0) ? dt_prev : dt_hint;
        // Always filter: pose-diff velocity (real_robot=false) is the noisiest signal
        // and needs the low-pass filter even more than hardware velocity does.
        qdot_out = filterJointVelocity(qdot_out, filter_dt);

        return qdot_out.allFinite();
    }

    // Seed tracked_positions_ / tracked_velocities_ from the real joint state
    bool initTrackedPositions()
    {
        Eigen::VectorXd q;
        if (!currentJointVector(q))
            return false;
        tracked_positions_.assign(q.data(), q.data() + q.size());
        tracked_velocities_.assign(joint_names_.size(), 0.0);
        return true;
    }

    // ============================================================
    // FORWARD KINEMATICS
    // ============================================================
    bool getEEPose(const Eigen::VectorXd &q,
                   Eigen::Vector3d &pos,
                   Eigen::Matrix3d &rot) const
    {
        auto fk = manip_->calcFwdKin(q);
        auto it = fk.find(ee_link_);
        if (it == fk.end())
            return false;
        pos = it->second.translation();
        rot = it->second.rotation();
        return true;
    }

    // Compute current EE pose from joint feedback and publish (xyz, rpy).
    // RPY uses ZYX intrinsic (roll about X, pitch about Y, yaw about Z).
    void publishFeedback()
    {
        Eigen::VectorXd q;
        if (!currentJointVector(q))
            return;
        Eigen::Vector3d pos;
        Eigen::Matrix3d rot;
        if (!getEEPose(q, pos, rot))
            return;

        const Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = pos.x();
        msg.linear.y = pos.y();
        msg.linear.z = pos.z();
        msg.angular.x = rpy.x();
        msg.angular.y = rpy.y();
        msg.angular.z = rpy.z();
        pub_feedback_->publish(msg);

        const double dt_hint = have_q_prev_ ? (this->now() - t_prev_q_).seconds()
                                            : (1.0 / std::max(1.0, rate_hz_));
        Eigen::VectorXd qdot;
        if (!getMeasuredJointVelocity(q, dt_hint, qdot, q_prev_, t_prev_q_, have_q_prev_))
            return;

        const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
        const Eigen::VectorXd v_cart = J * qdot;

        geometry_msgs::msg::Twist vmsg;
        vmsg.linear.x = v_cart(0);
        vmsg.linear.y = v_cart(1);
        vmsg.linear.z = v_cart(2);
        vmsg.angular.x = v_cart(3);
        vmsg.angular.y = v_cart(4);
        vmsg.angular.z = v_cart(5);
        pub_feedback_vel_->publish(vmsg);
    }

    // ============================================================
    // SAFETY CHECKS
    // ============================================================

    double clampJointVelocityLimits(Eigen::VectorXd &theta_d)
    {
        static const double lim[NUMBER_OF_JOINT] = {
            JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        };

        double scale = 1.0;
        const int n = std::min<int>(NUMBER_OF_JOINT, theta_d.size());

        for (int i = 0; i < n; ++i)
        {
            const double a = std::abs(theta_d[i]);
            if (a > lim[i] && a > 1e-12)
            {
                scale = std::min(scale, lim[i] / a);
            }
        }

        if (scale < 1.0)
        {
            theta_d *= scale;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Joint velocity clamped with scale %.3f. Continuing tracking.",
                scale);
        }

        return scale;
    }

    bool checkCartesianVelocitySafety(const Eigen::Matrix<double, 6, 1> &xdot_actual)
    {
        const double linear = xdot_actual.head<3>().norm();
        const double angular = xdot_actual.tail<3>().norm();

        if (measured_cart_linear_vel_limit_ > 0.0 &&
            linear > measured_cart_linear_vel_limit_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Measured Cartesian linear velocity %.4f m/s exceeds limit %.4f m/s",
                                 linear, measured_cart_linear_vel_limit_);
            return false;
        }

        if (measured_cart_angular_vel_limit_ > 0.0 &&
            angular > measured_cart_angular_vel_limit_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Measured Cartesian angular velocity %.4f rad/s exceeds limit %.4f rad/s",
                                 angular, measured_cart_angular_vel_limit_);
            return false;
        }

        return true;
    }

    void clampCartesianVelocity(Eigen::Matrix<double, 6, 1> &xdot) const
    {
        const double lin_n = xdot.head<3>().norm();
        if (lin_n > max_cart_linear_vel_ && lin_n > 1e-9)
            xdot.head<3>() *= max_cart_linear_vel_ / lin_n;

        const double ang_n = xdot.tail<3>().norm();
        if (ang_n > max_cart_angular_vel_ && ang_n > 1e-9)
            xdot.tail<3>() *= max_cart_angular_vel_ / ang_n;
    }

    void limitCartesianAcceleration(Eigen::Matrix<double, 6, 1> &xdot_next,
                                    const Eigen::Matrix<double, 6, 1> &xdot_prev,
                                    double dt) const
    {
        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));

        if (max_cart_linear_acc_ > 0.0)
        {
            const Eigen::Vector3d dv_lin = xdot_next.head<3>() - xdot_prev.head<3>();
            const double max_dv_lin = max_cart_linear_acc_ * dt_safe;
            if (dv_lin.norm() > max_dv_lin && dv_lin.norm() > 1e-9)
                xdot_next.head<3>() = xdot_prev.head<3>() + dv_lin.normalized() * max_dv_lin;
        }

        if (max_cart_angular_acc_ > 0.0)
        {
            const Eigen::Vector3d dv_ang = xdot_next.tail<3>() - xdot_prev.tail<3>();
            const double max_dv_ang = max_cart_angular_acc_ * dt_safe;
            if (dv_ang.norm() > max_dv_ang && dv_ang.norm() > 1e-9)
                xdot_next.tail<3>() = xdot_prev.tail<3>() + dv_ang.normalized() * max_dv_ang;
        }
    }

    bool checkPositionLimits(const std::vector<double> &pos) const
    {
        static const double lower[NUMBER_OF_JOINT] = {
            JOINT_1_S_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
        };
        static const double upper[NUMBER_OF_JOINT] = {
            JOINT_1_S_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
        };
        for (int i = 0; i < NUMBER_OF_JOINT; ++i)
        {
            if (pos[i] <= lower[i] || pos[i] >= upper[i])
            {
                RCLCPP_WARN(this->get_logger(),
                            "Joint %d position %.4f rad outside safe range [%.4f, %.4f]",
                            i, pos[i], lower[i], upper[i]);
                return false;
            }
        }
        return true;
    }

    // ============================================================
    // PUBLISH HELPERS
    // ============================================================

    void publishToTopic(
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &pub,
        const std::vector<double> &pos,
        const std::vector<double> &vel,
        double time_sec)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = pos;
        pt.velocities = vel;
        pt.time_from_start = rclcpp::Duration::from_seconds(time_sec);

        traj.points.push_back(pt);
        pub->publish(traj);
    }

    // Arm controller initialisation — sent ONCE to /joint_path_command
    void publishArmInit()
    {
        if (tracked_positions_.empty())
            return;
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishToTopic(pub_path_cmd_, tracked_positions_, zero_vel, 0.5);
        RCLCPP_INFO(this->get_logger(),
                    "Arm init sent to /joint_path_command (positions=current, vel=0, t=0.5s)");
    }

    // Optional seed — anchors starting position on /joint_command to avoid
    // INVALID_DATA_START_POS on stateful controllers (enable via enable_seed=true)
    void seed()
    {
        if (tracked_positions_.empty())
            return;
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishToTopic(pub_joint_cmd_, tracked_positions_, zero_vel, 0.0);
        streaming_time_ = 0.0;
        is_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Seed sent to joint_command (t=0)");
    }

    // Streaming publish to joint_command with relative time_from_start
    void publishTrajectory(const std::vector<double> &pos,
                           const std::vector<double> &vel)
    {
        publishToTopic(pub_joint_cmd_, pos, vel, streaming_time_);
        streaming_time_ += 1.0 / rate_hz_;
    }

    // ============================================================
    // CORE CONTROL STEP  (shared by STATE_INIT and STATE_POSE_FOLLOW)
    //
    // ADAPTIVE CARTESIAN ADMITTANCE-INSPIRED VELOCITY CONTROLLER.
    //
    // Virtual second-order dynamics shape the *reference* Cartesian
    // velocity ẋ_ref:
    //
    //     ẍ_ref = M_d⁻¹ · ( K_d·e + D_e·(ẋ_des−ẋ_ref) − F_coll − D_d·ẋ_ref )
    //     ẋ_ref ← ẋ_ref + ẍ_ref · dt
    //     θ̇_cmd = J⁺_SR(q) · ẋ_ref
    //
    // Adaptive scaling:
    //   s_t = 1 − exp(−λ·t)             (warm-up since activation)
    //   s_e = tanh(α·|e|)                (error-magnitude weighting)
    //   s_w = clamp(w/w0, 0.2, 1.0)      (manipulability — soften near singular)
    //   K_d = s_w · [K_min + s_t·s_e·(K_max−K_min)]
    //   M_d = M_max − s_t·s_e·(M_max−M_min)
    //   D_d = 2ζ√(M_d·K_d)               (critical-damping coefficient;
    //                                     used for both D_d and D_e here)
    //
    // NOTE: This is *admittance-style* — the underlying Yaskawa controller
    // is still position/velocity-driven, so we shape the commanded ẋ_ref
    // rather than closing a torque-level impedance loop.
    // ============================================================
    bool computeControlStep(const Eigen::VectorXd &q,
                            const Eigen::Vector3d &des_pos,
                            const Eigen::Matrix3d &des_rot,
                            double dt,
                            Eigen::VectorXd &theta_d)
    {
        // --- Forward kinematics ---
        Eigen::Vector3d ee_pos;
        Eigen::Matrix3d ee_rot;
        if (!getEEPose(q, ee_pos, ee_rot))
            return false;

        // --- Pose error (target − actual) ---
        e_p_ = des_pos - ee_pos;
        e_o_ = orientationError(des_rot, ee_rot);

        // --- Adaptive scaling factors ---
        const double t_active = (this->now() - t_start_).seconds();
        const double s_t = 1.0 - std::exp(-adaptive_lambda_ * std::max(0.0, t_active));
        const double s_e_pos = std::tanh(adaptive_alpha_pos_ * e_p_.norm());
        const double s_e_ori = std::tanh(adaptive_alpha_ori_ * e_o_.norm());

        // --- Jacobian + manipulability ---
        const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
        const double w = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
        const double w0_safe = std::max(1e-9, w0_);
        const double s_w = std::clamp(w / w0_safe, 0.2, 1.0);
        if (w <= w0_)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Near singularity (w=%.6f ≤ w0=%.6f). SR-damping active.", w, w0_);

        // --- Adaptive virtual-impedance gains (separate trans / rot) ---
        const double k_pos_var = s_w *
                                 (k_pos_min_ + s_t * s_e_pos * (k_pos_max_ - k_pos_min_));
        const double m_pos_var = m_pos_max_ -
                                 s_t * s_e_pos * (m_pos_max_ - m_pos_min_);
        const double d_pos_var = 2.0 * zeta_pos_ *
                                 std::sqrt(std::max(1e-12, m_pos_var * k_pos_var));

        const double k_ori_var = s_w *
                                 (k_ori_min_ + s_t * s_e_ori * (k_ori_max_ - k_ori_min_));
        const double m_ori_var = m_ori_max_ -
                                 s_t * s_e_ori * (m_ori_max_ - m_ori_min_);
        const double d_ori_var = 2.0 * zeta_ori_ *
                                 std::sqrt(std::max(1e-12, m_ori_var * k_ori_var));

        // --- Measured joint velocity: filtered for safety/telemetry only. ---
        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
        Eigen::VectorXd qdot;
        if (!getMeasuredJointVelocity(q, dt_safe, qdot, q_prev_ctrl_, t_prev_q_ctrl_, have_q_prev_ctrl_))
            return false;

        const Eigen::Matrix<double, 6, 1> xdot_actual = J * qdot;
        if (!checkCartesianVelocitySafety(xdot_actual))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Measured Cartesian velocity safety exceeded → STATE_STOP");
            state_ = STATE_STOP;
            return false;
        }

        // --- Desired Cartesian velocity (target_vel if fresh, else 0) ---
        Eigen::Matrix<double, 6, 1> xdot_des = Eigen::Matrix<double, 6, 1>::Zero();
        const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
        if (dt_vel < TARGET_VEL_TIMEOUT_SEC)
        {
            xdot_des.head<3>() = latest_target_vel_.head<3>();
            // /motomini/target_vel angular is body-frame. Convert it to the
            // base frame before applying reference-state damping.
            xdot_des.tail<3>() = des_rot * latest_target_vel_.tail<3>();
        }
        if (xdot_des.head<3>().norm() <= targetVelocityDeadband())
            xdot_des.head<3>().setZero();
        if (xdot_des.tail<3>().norm() <= targetVelocityDeadband())
            xdot_des.tail<3>().setZero();

        // --- Collision wrench from /motomini/collision_wrench ---
        // The debug node publishes a repulsive push-away wrench. The callback
        // stores the controller-side sign so this term can be used directly.
        Eigen::Matrix<double, 6, 1> F_collision;
        F_collision.setZero();
        const double dt_collision_wrench =
            (this->now() - t_last_collision_wrench_cb_).seconds();
        if (dt_collision_wrench < collision_wrench_timeout_sec_)
            F_collision = latest_collision_wrench_;

        // Scale + clamp the soft repulsive force so it stays small relative to
        // the goal force during close work. The hard "no deeper" rule is
        // enforced separately by the velocity projection below.
        F_collision.head<3>() *= collision_force_scale_;
        const double f_norm = F_collision.head<3>().norm();
        if (collision_force_max_ > 0.0 && f_norm > collision_force_max_ && f_norm > 1e-9)
            F_collision.head<3>() *= collision_force_max_ / f_norm;
        F_collision = filterCollisionWrench(F_collision, dt_safe);

        // --- Close-work safety projection: distance/normal from the collision
        //     node feed a blended velocity & goal-force suppression. ---
        const double dt_dist =
            (this->now() - t_last_collision_distance_cb_).seconds();
        const double dt_norm =
            (this->now() - t_last_collision_normal_cb_).seconds();
        const bool collision_constraint_active =
            (dt_dist < collision_constraint_timeout_sec_) &&
            (dt_norm < collision_constraint_timeout_sec_) &&
            latest_collision_normal_.allFinite() &&
            (latest_collision_normal_.norm() > 1e-6) &&
            std::isfinite(latest_collision_distance_);

        double gamma = 0.0;
        Eigen::Vector3d n_away = Eigen::Vector3d::Zero();
        if (collision_constraint_active)
        {
            n_away = latest_collision_normal_.normalized();
            const double span =
                std::max(1e-6, collision_guard_distance_ - collision_task_distance_);
            gamma = 1.0 - std::clamp(
                              (latest_collision_distance_ - collision_task_distance_) / span,
                              0.0, 1.0);
            gamma = std::min(gamma, collision_projection_max_gamma_);
        }

        // ------------------------------------------------------------
        // Collision wall behavior:
        // Keep tangential collision force, fade/remove normal spring force.
        // n_away points away from the obstacle.
        // F_pub is the actual published push/tangent force from collision node.
        // F_collision is stored with opposite sign in this controller.
        // ------------------------------------------------------------
        if (collision_constraint_active && gamma > 0.0)
        {
            Eigen::Vector3d F_pub = -F_collision.head<3>();

            if (F_pub.allFinite() && n_away.allFinite() && n_away.norm() > 1e-9)
            {
                const double f_n = F_pub.dot(n_away);

                if (f_n > 0.0)
                {
                    // Positive normal component = push-away spring.
                    // Fade it out as the hard velocity wall becomes active.
                    const Eigen::Vector3d F_normal = f_n * n_away;
                    const Eigen::Vector3d F_tangent = F_pub - F_normal;

                    F_pub = F_tangent + (1.0 - gamma) * F_normal;
                }
                else if (f_n < 0.0)
                {
                    // Never allow collision wrench to push into the obstacle.
                    F_pub -= f_n * n_away;
                }

                // Store back using controller's sign convention.
                F_collision.head<3>() = -F_pub;
            }
        }

        // ------------------------------------------------------------
        // Project desired/feedforward velocity too.
        // This prevents target_vel from creating a damping force into the obstacle.
        // Tangent and away velocity are preserved.
        // ------------------------------------------------------------
        if (collision_constraint_active && gamma > 0.0)
        {
            const double vd_into = xdot_des.head<3>().dot(n_away);

            if (vd_into < 0.0)
                xdot_des.head<3>() -= gamma * vd_into * n_away;
        }

        // --- Virtual acceleration per motomini_optimal_controller_fix.md ---
        // Correct form: ẍ_ref = M⁻¹·( K·e + D·(ẋ_des − ẋ_ref) − F_coll )
        // The damping term D acts ONLY on the velocity error (ẋ_des − ẋ_ref).
        // Do NOT subtract an extra D·ẋ_ref: that would create double damping
        // (K·e + D·ẋ_des − 2D·ẋ_ref) which can satisfy ẍ_ref=0 while e≠0.
        //
        // With this form, at steady state ẍ_ref=0 requires K·e=0 → e=0. ✓
        const Eigen::Matrix<double, 6, 1> v_err = xdot_des - xdot_ref_;

        Eigen::Vector3d F_goal_pos = k_pos_var * e_p_ + d_pos_var * v_err.head<3>();
        Eigen::Vector3d F_goal_ori = k_ori_var * e_o_ + d_ori_var * v_err.tail<3>();

        // ------------------------------------------------------------
        // Wall reaction force.
        // If goal force tries to go into obstacle, create equal opposite
        // reaction scaled by gamma. This cancels inward force, not push away.
        // ------------------------------------------------------------
        if (collision_goal_suppression_ && gamma > 0.0)
        {
            const double F_into = F_goal_pos.dot(n_away);

            if (F_into < 0.0)
            {
                const Eigen::Vector3d F_wall_reaction =
                    -gamma * F_into * n_away;

                F_goal_pos += F_wall_reaction;
            }
        }

        Eigen::Matrix<double, 6, 1> xddot_ref;
        xddot_ref.head<3>() = (F_goal_pos - F_collision.head<3>()) / std::max(1e-9, m_pos_var);
        xddot_ref.tail<3>() = (F_goal_ori - F_collision.tail<3>()) / std::max(1e-9, m_ori_var);

        // --- Integrate ẋ_ref ---
        const Eigen::Matrix<double, 6, 1> xdot_prev = xdot_ref_;
        Eigen::Matrix<double, 6, 1> xdot_next = xdot_ref_ + xddot_ref * dt_safe;
        limitCartesianAcceleration(xdot_next, xdot_prev, dt_safe);
        xdot_ref_ = xdot_next;

        // --- Velocity projection: hard "no deeper into obstacle" rule on the
        //     translational part of the integrator state. Tangent and away
        //     motion stay untouched. ---
        if (enable_collision_projection_ && gamma > 0.0)
        {
            const double v_into = xdot_ref_.head<3>().dot(n_away);
            if (v_into < 0.0)
                xdot_ref_.head<3>() -= gamma * v_into * n_away;
        }

        // --- Safety clamps on the integrator state ---
        clampCartesianVelocity(xdot_ref_);

        // --- SR-inverse Jacobian → joint velocity command ---
        theta_d = calcSrInverse(J, w, w0_, k0_) * xdot_ref_;

        // Clamp joint velocity instead of stopping the controller.
        const double joint_scale = clampJointVelocityLimits(theta_d);

        // Anti-windup for the Cartesian velocity integrator.
        // After clamping theta_d, update xdot_ref_ to the Cartesian velocity
        // that the clamped joint command can actually produce.
        if (joint_scale < 1.0)
        {
            const Eigen::VectorXd xdot_limited = J * theta_d;
            if (xdot_limited.size() == 6 && xdot_limited.allFinite())
            {
                xdot_ref_ = xdot_limited;
                clampCartesianVelocity(xdot_ref_);
            }
        }

        // After joint velocity clamp anti-windup, project again.
        // Joint saturation can slightly reintroduce an into-surface component.
        if (joint_scale < 1.0 &&
            enable_collision_projection_ &&
            collision_constraint_active &&
            gamma > 0.0)
        {
            const double v_into = xdot_ref_.head<3>().dot(n_away);

            if (v_into < 0.0)
                xdot_ref_.head<3>() -= gamma * v_into * n_away;
        }

        return true;
    }

    bool initializeReferenceVelocityFromMeasuredState()
    {
        Eigen::VectorXd q;
        if (!currentJointVector(q))
        {
            xdot_ref_.setZero();
            return false;
        }

        Eigen::VectorXd qdot;
        if (!getMeasuredJointVelocity(q,
                                      1.0 / std::max(1.0, rate_hz_),
                                      qdot,
                                      q_prev_ctrl_,
                                      t_prev_q_ctrl_,
                                      have_q_prev_ctrl_))
        {
            xdot_ref_.setZero();
            return false;
        }

        const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
        const Eigen::VectorXd xdot_measured = J * qdot;
        if (xdot_measured.size() != 6 || !xdot_measured.allFinite())
        {
            xdot_ref_.setZero();
            return false;
        }

        xdot_ref_ = xdot_measured;
        clampCartesianVelocity(xdot_ref_);
        return true;
    }

    double lowPassAlpha(double cutoff_hz, double dt) const
    {
        if (dt <= 0.0)
            return 1.0;
        return std::clamp(1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt), 0.0, 1.0);
    }

    double targetVelocityDeadband() const
    {
        return std::max(0.002, 0.01 * max_cart_linear_vel_);
    }

    double collisionForceAttackHz() const
    {
        const double tau = std::max(0.02, 0.2 * collision_wrench_timeout_sec_);
        return 1.0 / tau;
    }

    double collisionForceReleaseHz() const
    {
        return std::max(1.0, 0.35 * collisionForceAttackHz());
    }

    Eigen::Matrix<double, 6, 1> filterCollisionWrench(
        const Eigen::Matrix<double, 6, 1> &raw_wrench,
        double dt)
    {
        if (dt <= 0.0 || dt > 1.0)
        {
            filtered_collision_wrench_ = raw_wrench;
            return filtered_collision_wrench_;
        }

        const double cutoff_hz =
            (raw_wrench.head<3>().norm() >= filtered_collision_wrench_.head<3>().norm())
                ? collisionForceAttackHz()
                : collisionForceReleaseHz();
        const double alpha = lowPassAlpha(cutoff_hz, dt);
        filtered_collision_wrench_ += alpha * (raw_wrench - filtered_collision_wrench_);
        if (filtered_collision_wrench_.norm() < 1e-6 && raw_wrench.norm() < 1e-6)
            filtered_collision_wrench_.setZero();
        return filtered_collision_wrench_;
    }

    // Reset the virtual integrator state when entering IDLE/STOP or starting
    // INIT/POSE_FOLLOW.
    void resetVirtualState()
    {
        xdot_ref_.setZero();
        have_q_prev_ctrl_ = false;
        q_prev_ctrl_.resize(0);
        first_velocity_read_ = true;
        qdot_filtered_.resize(0);
        have_velocity_filter_update_ = false;
        t_last_velocity_filter_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        latest_collision_wrench_.setZero();
        filtered_collision_wrench_.setZero();
        t_last_collision_wrench_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        latest_collision_distance_ = std::numeric_limits<double>::infinity();
        latest_collision_normal_.setZero();
        t_last_collision_distance_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        t_last_collision_normal_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    void resetControlWindow()
    {
        resetVirtualState();
        e_p_.setZero();
        e_o_.setZero();
        t_start_ = this->now();
        t_last_ = this->now();
    }

    // ============================================================
    // CALLBACKS
    // ============================================================

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;

        // Send arm-init once as soon as a valid joint state arrives
        if (!arm_init_sent_ &&
            static_cast<int>(msg->name.size()) >= NUMBER_OF_JOINT)
        {
            if (initTrackedPositions())
            {
                publishArmInit();
                arm_init_sent_ = true;
            }
        }
    }

    // Streaming desired pose — triggers IDLE → ARMING → POSE_FOLLOW
    void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        desired_pose_ = *msg;
        has_desired_pose_ = true;
        t_last_pose_cb_ = this->now();

        if (state_ == STATE_IDLE)
        {
            if (initTrackedPositions())
            {
                pending_state_ = STATE_POSE_FOLLOW;
                state_ = STATE_ARMING;
                RCLCPP_INFO(this->get_logger(), "Target received: STATE_IDLE → STATE_ARMING");
            }
        }
    }

    // Streaming target velocity.
    // The velocity is integrated into desired_pose_ each tick (in handlePoseFollow).
    // This callback just caches what arrives. The integration guard in handlePoseFollow
    // uses the deadband to skip near-zero velocities, so no explicit zeroing is needed here.
    void targetVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_target_vel_ << msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z;
        t_last_target_vel_cb_ = this->now();

        // Velocity-only entry: seed desired_pose_ from current EE and start tracking.
        if (state_ == STATE_IDLE &&
            latest_target_vel_.head<3>().norm() > targetVelocityDeadband())
        {
            Eigen::VectorXd q;
            Eigen::Vector3d ee_pos;
            Eigen::Matrix3d ee_rot;
            if (!currentJointVector(q) || !getEEPose(q, ee_pos, ee_rot))
                return;
            Eigen::Quaterniond qee(ee_rot);
            desired_pose_.header.frame_id = base_link_;
            desired_pose_.pose.position.x = ee_pos.x();
            desired_pose_.pose.position.y = ee_pos.y();
            desired_pose_.pose.position.z = ee_pos.z();
            desired_pose_.pose.orientation.w = qee.w();
            desired_pose_.pose.orientation.x = qee.x();
            desired_pose_.pose.orientation.y = qee.y();
            desired_pose_.pose.orientation.z = qee.z();
            has_desired_pose_ = true;
            t_last_pose_cb_ = this->now();

            if (initTrackedPositions())
            {
                pending_state_ = STATE_POSE_FOLLOW;
                state_ = STATE_ARMING;
                RCLCPP_INFO(this->get_logger(), "Target velocity received: STATE_IDLE → STATE_ARMING");
            }
        }
    }

    // Debug wrench is published as a repulsive push-away wrench. Cache the
    // controller-side sign so the existing "- F_collision" term uses it
    // directly without changing the main control law.
    void collisionWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        latest_collision_wrench_ << -msg->wrench.force.x,
            -msg->wrench.force.y,
            -msg->wrench.force.z,
            -msg->wrench.torque.x,
            -msg->wrench.torque.y,
            -msg->wrench.torque.z;
        t_last_collision_wrench_cb_ = this->now();
    }

    // Closest-contact distance from the collision node. Used together with
    // the n_away normal to compute the safety projection blend factor gamma.
    void collisionDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        latest_collision_distance_ = msg->data;
        t_last_collision_distance_cb_ = this->now();
    }

    // Closest-contact push-away normal in the wrench / base frame. A zero
    // vector means "no active constraint" and disables the projection.
    void collisionNormalCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        latest_collision_normal_ << msg->vector.x, msg->vector.y, msg->vector.z;
        t_last_collision_normal_cb_ = this->now();
    }

    // Latched init target pose (published once before calling /init_start)
    void initPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        init_pose_ = *msg;
        has_init_pose_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Init pose received: (%.4f, %.4f, %.4f)",
                    init_pose_.pose.position.x,
                    init_pose_.pose.position.y,
                    init_pose_.pose.position.z);
    }

    // /pose_following/start → reset to STATE_IDLE
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_IDLE;
        is_active_ = false;
        arm_trigger_sent_ = false;
        resetControlWindow();
        tracked_positions_.clear();
        tracked_velocities_.clear();
        is_init_done_ = false;
        res->success = true;
        res->message = "Reset to STATE_IDLE and re-armed.";
        RCLCPP_INFO(this->get_logger(), "/start → STATE_IDLE (ready to re-arm)");
    }

    // /pose_following/stop → STATE_STOP (immediate, zero velocity publishing)
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_STOP;
        resetControlWindow();
        res->success = true;
        res->message = "Stopped — STATE_STOP (holding position)";
        RCLCPP_INFO(this->get_logger(), "/stop → STATE_STOP");
    }

    // /pose_following/init_start → STATE_IDLE → STATE_ARMING → STATE_INIT
    void initStartCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!has_init_pose_)
        {
            res->success = false;
            res->message = "No init pose received. Publish to /pose_following/init_pose first.";
            return;
        }
        if (state_ != STATE_IDLE)
        {
            res->success = false;
            res->message = "Must be in STATE_IDLE to start init move.";
            return;
        }
        if (!initTrackedPositions())
        {
            res->success = false;
            res->message = "No joint state available yet.";
            return;
        }
        is_init_done_ = false;
        pending_state_ = STATE_INIT;
        state_ = STATE_ARMING;
        res->success = true;
        res->message = "STATE_IDLE → STATE_ARMING → STATE_INIT";
        RCLCPP_INFO(this->get_logger(), "/init_start → STATE_ARMING");
    }

    // ============================================================
    // STATE HANDLERS
    // ============================================================

    // IDLE — sync internal positions from real robot; publish if active
    void handleIdle()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            resetVirtualState();
            RCLCPP_INFO(this->get_logger(), "STATE_IDLE: syncing joint state.");
        }
        if (last_joint_state_)
        {
            initTrackedPositions();
            if (is_active_)
            {
                std::vector<double> zero_vel(joint_names_.size(), 0.0);
                publishTrajectory(tracked_positions_, zero_vel);
            }
        }
    }

    // STOP — zero velocity, sync from real robot; publish if active
    void handleStop()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            resetVirtualState();
            RCLCPP_INFO(this->get_logger(),
                        "STATE_STOP: velocity zeroed. Still publishing current state.");
        }
        if (last_joint_state_)
        {
            initTrackedPositions();
            if (is_active_)
            {
                std::vector<double> zero_vel(joint_names_.size(), 0.0);
                publishTrajectory(tracked_positions_, zero_vel);
            }
        }
    }

    // ARMING — robust sequence: Path Command (Arm Trigger) → Delay → Seed → Active
    void handleArming()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            t_arming_start_ = this->now();
            arm_trigger_sent_ = false;
            RCLCPP_INFO(this->get_logger(), "STATE_ARMING: sequence started (Pre-delay: %.1fs)", ARM_PRE_DELAY_S);
        }

        double elapsed = (this->now() - t_arming_start_).seconds();
        
        // Step 1: Send Path Command after pre-delay
        if (!arm_trigger_sent_)
        {
            if (elapsed < ARM_PRE_DELAY_S) return;
            if (initTrackedPositions())
            {
                publishArmInit();
                arm_trigger_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "STATE_ARMING: Arm trigger sent (Post-delay: %.1fs)", ARM_POST_DELAY_S);
            }
            return;
        }

        // Step 2: Wait post-delay then send Seed and transition
        if (elapsed < ARM_PRE_DELAY_S + ARM_POST_DELAY_S) return;

        if (initTrackedPositions())
        {
            seed();
            resetControlWindow();
            initializeReferenceVelocityFromMeasuredState();
            state_ = pending_state_;
            RCLCPP_INFO(this->get_logger(), "STATE_ARMING → %s", 
                        (state_ == STATE_INIT ? "STATE_INIT" : "STATE_POSE_FOLLOW"));
        }
    }

    // INIT — move to init_pose_ using adaptive admittance + SR-inverse
    void handleInit()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            // initializeReferenceVelocityFromMeasuredState already called in handleArming
            RCLCPP_INFO(this->get_logger(), "STATE_INIT: moving to init pose...");
        }

        if (!has_init_pose_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "STATE_INIT: waiting for /pose_following/init_pose.");
            return;
        }

        Eigen::VectorXd q;
        if (!currentJointVector(q))
            return;

        // Check completion (position error below threshold)
        Eigen::Vector3d ee_pos;
        Eigen::Matrix3d ee_rot;
        if (!getEEPose(q, ee_pos, ee_rot))
            return;

        double dx = std::abs(init_pose_.pose.position.x - ee_pos.x());
        double dy = std::abs(init_pose_.pose.position.y - ee_pos.y());
        double dz = std::abs(init_pose_.pose.position.z - ee_pos.z());

        if (dx < POSITION_ERROR_THRESHOLD &&
            dy < POSITION_ERROR_THRESHOLD &&
            dz < POSITION_ERROR_THRESHOLD)
        {
            is_init_done_ = true;
            resetControlWindow();
            initializeReferenceVelocityFromMeasuredState();
            state_ = STATE_POSE_FOLLOW;
            RCLCPP_INFO(this->get_logger(),
                        "Init complete (err: %.5f, %.5f, %.5f m). STATE_INIT → STATE_POSE_FOLLOW",
                        dx, dy, dz);
            return;
        }

        // Desired pose from init_pose_
        Eigen::Quaterniond q_des(
            init_pose_.pose.orientation.w,
            init_pose_.pose.orientation.x,
            init_pose_.pose.orientation.y,
            init_pose_.pose.orientation.z);
        q_des.normalize();
        Eigen::Vector3d des_pos(
            init_pose_.pose.position.x,
            init_pose_.pose.position.y,
            init_pose_.pose.position.z);

        double dt = (this->now() - t_last_).seconds();
        t_last_ = this->now();

        Eigen::VectorXd theta_d;
        if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt, theta_d))
            return;

        // Joint velocity is clamped inside computeControlStep().
        // Do not enter STATE_STOP for normal command saturation.

        if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
            !initTrackedPositions())
        {
            return;
        }

        // Integrate: derive velocity consistently
        const std::vector<double> prev_pos = tracked_positions_;
        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
        for (size_t i = 0; i < tracked_positions_.size(); ++i)
        {
            tracked_positions_[i] =
                q[static_cast<Eigen::Index>(i)] +
                theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
            tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
        }

        // Position safety — revert and hold on limit violation
        if (!checkPositionLimits(tracked_positions_))
        {
            RCLCPP_WARN(this->get_logger(), "[INIT] Joint position limit → holding.");
            tracked_positions_ = prev_pos;
            std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
        }

        publishTrajectory(tracked_positions_, tracked_velocities_);
    }

    // POSE_FOLLOW — track streaming desired_pose_ via adaptive admittance + SR-inverse
    void handlePoseFollow()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            // initializeReferenceVelocityFromMeasuredState already called in handleArming
            RCLCPP_INFO(this->get_logger(),
                        "STATE_POSE_FOLLOW: tracking /motomini/target_pose");
        }

        if (!has_desired_pose_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "STATE_POSE_FOLLOW: waiting for desired pose on "
                                 "/motomini/target_pose.");
            return;
        }

        // Pose timeout — keep active but hold current position
        double dt_cb = (this->now() - t_last_pose_cb_).seconds();
        if (dt_cb > POSE_TIMEOUT_SEC)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Pose input timeout (%.2f s > %.2f s) — holding position.", dt_cb, POSE_TIMEOUT_SEC);
            
            if (last_joint_state_)
            {
                initTrackedPositions();
                std::vector<double> zero_vel(joint_names_.size(), 0.0);
                publishTrajectory(tracked_positions_, zero_vel);
            }
            return;
        }

        Eigen::VectorXd q;
        if (!currentJointVector(q))
            return;

        Eigen::Quaterniond q_des(
            desired_pose_.pose.orientation.w,
            desired_pose_.pose.orientation.x,
            desired_pose_.pose.orientation.y,
            desired_pose_.pose.orientation.z);
        q_des.normalize();
        Eigen::Vector3d des_pos(
            desired_pose_.pose.position.x,
            desired_pose_.pose.position.y,
            desired_pose_.pose.position.z);

        double dt = (this->now() - t_last_).seconds();
        t_last_ = this->now();
        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));

        // Integrate streaming target_vel into the desired pose.
        // Only integrate when the velocity is fresh AND above the deadband.
        // When the user releases (vel=0 callback), targetVelCallback has already
        // frozen desired_pose_ at the current EE — so we just skip integration here.
        const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
        const bool vel_fresh = dt_vel < TARGET_VEL_TIMEOUT_SEC;
        const bool vel_active = vel_fresh &&
                                (latest_target_vel_.head<3>().norm() > targetVelocityDeadband() ||
                                 latest_target_vel_.tail<3>().norm() > targetVelocityDeadband());
        // Integrate target_vel into desired_pose_ only when in Mode B (joystick/vel-only).
        // Mode A (external pose streamer): integrate_target_vel_to_pose=false → target_vel
        // is feedforward only in computeControlStep; do NOT double-apply it here.
        if (integrate_target_vel_to_pose_ && vel_active)
        {
            des_pos.x() += latest_target_vel_(0) * dt_safe;
            des_pos.y() += latest_target_vel_(1) * dt_safe;
            des_pos.z() += latest_target_vel_(2) * dt_safe;
            const Eigen::Quaterniond delta =
                Eigen::AngleAxisd(latest_target_vel_(3) * dt_safe, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(latest_target_vel_(4) * dt_safe, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(latest_target_vel_(5) * dt_safe, Eigen::Vector3d::UnitZ());
            q_des = (q_des * delta).normalized();

            desired_pose_.pose.position.x = des_pos.x();
            desired_pose_.pose.position.y = des_pos.y();
            desired_pose_.pose.position.z = des_pos.z();
            desired_pose_.pose.orientation.w = q_des.w();
            desired_pose_.pose.orientation.x = q_des.x();
            desired_pose_.pose.orientation.y = q_des.y();
            desired_pose_.pose.orientation.z = q_des.z();
            t_last_pose_cb_ = this->now(); 
        }

        Eigen::VectorXd theta_d;
        if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt_safe, theta_d))
            return;

        // Joint velocity is clamped inside computeControlStep().
        // Do not enter STATE_STOP for normal command saturation.

        if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
            !initTrackedPositions())
        {
            return;
        }

        // Integrate: derive velocity consistently
        const std::vector<double> prev_pos = tracked_positions_;
        for (size_t i = 0; i < tracked_positions_.size(); ++i)
        {
            tracked_positions_[i] =
                q[static_cast<Eigen::Index>(i)] +
                theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
            tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
        }

        // Position safety — revert and hold on limit violation
        if (!checkPositionLimits(tracked_positions_))
        {
            RCLCPP_WARN(this->get_logger(), "[FOLLOW] Joint position limit → holding.");
            tracked_positions_ = prev_pos;
            std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
        }

        publishTrajectory(tracked_positions_, tracked_velocities_);
    }

    // ============================================================
    // MAIN CONTROL TICK  (called by timer at rate_hz_)
    // ============================================================
    void tick()
    {
        publishFeedback();

        switch (state_)
        {
        case STATE_IDLE:
            handleIdle();
            break;
        case STATE_ARMING:
            handleArming();
            break;
        case STATE_INIT:
            handleInit();
            break;
        case STATE_POSE_FOLLOW:
            handlePoseFollow();
            break;
        case STATE_STOP:
            handleStop();
            break;
        }
    }

    // ============================================================
    // MEMBER VARIABLES
    // ============================================================

    // --- Configuration ---
    std::string urdf_xml_, srdf_xml_, manipulator_group_, base_link_, ee_link_;
    double rate_hz_;
    double w0_, k0_;
    bool enable_seed_;
    bool real_robot_{true};
    double velocity_filter_cutoff_hz_{15.0};
    double max_cart_linear_acc_{0.8};
    double max_cart_angular_acc_{2.5};
    double measured_cart_linear_vel_limit_{1.0};
    double measured_cart_angular_vel_limit_{3.0};

    // --- Adaptive Cartesian admittance gains ---
    double m_pos_min_, m_pos_max_, k_pos_min_, k_pos_max_, zeta_pos_;
    double m_ori_min_, m_ori_max_, k_ori_min_, k_ori_max_, zeta_ori_;
    bool integrate_target_vel_to_pose_{true}; // Mode B (joystick). Set false for Mode A (ext pose).
    double adaptive_lambda_, adaptive_alpha_pos_, adaptive_alpha_ori_;
    double max_cart_linear_vel_, max_cart_angular_vel_;

    // --- Virtual Cartesian-velocity integrator state ---
    Eigen::Matrix<double, 6, 1> xdot_ref_{Eigen::Matrix<double, 6, 1>::Zero()};

    // --- Measured θ̇ history for safety/start initialization ---
    Eigen::VectorXd q_prev_ctrl_;
    rclcpp::Time t_prev_q_ctrl_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_ctrl_{false};
    Eigen::VectorXd qdot_filtered_;
    bool first_velocity_read_{true};
    rclcpp::Time t_last_velocity_filter_update_{0, 0, RCL_ROS_TIME};
    bool have_velocity_filter_update_{false};

    // --- Runtime state ---
    State state_, last_state_;
    bool is_init_done_;
    bool arm_init_sent_;
    bool has_desired_pose_;
    bool has_init_pose_;

    // --- Control errors ---
    Eigen::Vector3d e_p_;
    Eigen::Vector3d e_o_;

    // --- Time ---
    rclcpp::Time t_start_;                    // reset at each IDLE → active transition
    rclcpp::Time t_last_;                     // last tick timestamp
    rclcpp::Time t_last_pose_cb_;             // last desired-pose callback time
    rclcpp::Time t_last_target_vel_cb_;       // last target-velocity callback time
    rclcpp::Time t_last_collision_wrench_cb_; // last collision-wrench callback time

    // --- Streaming target velocity (linear=base, angular=body) ---
    Eigen::Matrix<double, 6, 1> latest_target_vel_{Eigen::Matrix<double, 6, 1>::Zero()};

    // --- Latest collision wrench from /motomini/collision_wrench ---
    Eigen::Matrix<double, 6, 1> latest_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
    double collision_wrench_timeout_sec_;

    // --- Close-work safety projection (distance/normal driven) ---
    bool enable_collision_projection_{true};
    bool collision_goal_suppression_{true};
    double collision_guard_distance_{0.03};
    double collision_task_distance_{0.005};
    double collision_stop_distance_{0.001};
    double collision_projection_max_gamma_{1.0};
    double collision_constraint_timeout_sec_{0.2};
    double collision_force_scale_{1.0};
    double collision_force_max_{5.0};

    double latest_collision_distance_{std::numeric_limits<double>::infinity()};
    Eigen::Vector3d latest_collision_normal_{Eigen::Vector3d::Zero()};
    rclcpp::Time t_last_collision_distance_cb_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_collision_normal_cb_{0, 0, RCL_ROS_TIME};
    Eigen::Matrix<double, 6, 1> filtered_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};

    // --- Numerical θ̇ history for feedback_vel (J·θ̇) ---
    Eigen::VectorXd q_prev_;
    rclcpp::Time t_prev_q_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_{false};

    // --- Trajectory integration ---
    std::vector<double> tracked_positions_;
    std::vector<double> tracked_velocities_;
    std::vector<std::string> joint_names_;

    // --- Kinematics ---
    tesseract_environment::Environment::Ptr env_;
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;

    // --- ROS interfaces ---
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_path_cmd_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_feedback_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_feedback_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target_vel_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_collision_wrench_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_collision_distance_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_collision_normal_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_desired_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_init_pose_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_start_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    geometry_msgs::msg::PoseStamped desired_pose_;
    geometry_msgs::msg::PoseStamped init_pose_;

    // --- Arming & Streaming ---
    double streaming_time_{0.0};
    rclcpp::Time t_arming_start_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};
    State pending_state_{STATE_IDLE};
    bool is_active_{false}; // True if we have ever started streaming since last /start
};

// ============================================================
// MAIN
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniFeedbackStreamNode>());
    rclcpp::shutdown();
    return 0;
}
