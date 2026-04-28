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
#define SAFETY_VELOCITY_ALPHA 0.65 // fraction of hardware limit to use

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

// ============================================================
// SECTION 2 – INCLUDES
// ============================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
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
            throw std::runtime_error("Failed to initialize Tesseract kinematics");

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
        latest_target_vel_.setZero();

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

        // Cartesian velocity = J(q) · θ̇.
        // We derive θ̇ by numerical differentiation of observed joint positions,
        // because many drivers/simulators leave the JointState.velocity field
        // empty or zero. This guarantees the topic always reflects real motion.
        const rclcpp::Time t_now = this->now();
        Eigen::VectorXd qdot = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
        if (have_q_prev_ && q_prev_.size() == q.size())
        {
            const double dt_q = (t_now - t_prev_q_).seconds();
            if (dt_q > 1e-6)
                qdot = (q - q_prev_) / dt_q;
        }
        q_prev_ = q;
        t_prev_q_ = t_now;
        have_q_prev_ = true;

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

    bool checkVelocityLimits(const Eigen::VectorXd &theta_d) const
    {
        static const double lim[NUMBER_OF_JOINT] = {
            JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
            JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        };
        for (int i = 0; i < NUMBER_OF_JOINT; ++i)
        {
            if (std::abs(theta_d[i]) > lim[i])
            {
                RCLCPP_WARN(this->get_logger(),
                            "Joint %d velocity %.4f rad/s exceeds limit %.4f rad/s",
                            i, theta_d[i], lim[i]);
                return false;
            }
        }
        return true;
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
        RCLCPP_INFO(this->get_logger(), "Seed sent to joint_command (t=0)");
    }

    // Streaming publish to joint_command with relative time_from_start
    void publishTrajectory(const std::vector<double> &pos,
                           const std::vector<double> &vel)
    {
        double t_rel = (this->now() - t_start_).seconds();
        publishToTopic(pub_joint_cmd_, pos, vel, t_rel);
    }

    // ============================================================
    // CORE CONTROL STEP  (shared by STATE_INIT and STATE_POSE_FOLLOW)
    //
    // ADAPTIVE CARTESIAN ADMITTANCE-INSPIRED VELOCITY CONTROLLER.
    //
    // Virtual second-order dynamics shape the *reference* Cartesian
    // velocity ẋ_ref:
    //
    //     ẍ_ref = M_d⁻¹ · ( K_d·e − D_e·ė − F_coll − D_d·ẋ_ref )
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

        // --- Joint velocity θ̇: prefer driver field, fall back to numerical diff ---
        Eigen::VectorXd qdot = Eigen::VectorXd::Zero(q.size());
        bool used_driver = false;
        if (last_joint_state_ &&
            last_joint_state_->velocity.size() >= last_joint_state_->name.size())
        {
            Eigen::VectorXd qdot_drv(static_cast<Eigen::Index>(joint_names_.size()));
            bool ok = true;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto it = std::find(last_joint_state_->name.begin(),
                                    last_joint_state_->name.end(),
                                    joint_names_[i]);
                if (it == last_joint_state_->name.end()) { ok = false; break; }
                const size_t idx = std::distance(last_joint_state_->name.begin(), it);
                if (idx >= last_joint_state_->velocity.size()) { ok = false; break; }
                qdot_drv[static_cast<Eigen::Index>(i)] = last_joint_state_->velocity[idx];
            }
            if (ok && qdot_drv.allFinite())
            {
                qdot = qdot_drv;
                used_driver = true;
            }
        }
        if (!used_driver && have_q_prev_ctrl_ && q_prev_ctrl_.size() == q.size())
        {
            const double dt_q = (this->now() - t_prev_q_ctrl_).seconds();
            if (dt_q > 1e-6)
                qdot = (q - q_prev_ctrl_) / dt_q;
        }
        q_prev_ctrl_ = q;
        t_prev_q_ctrl_ = this->now();
        have_q_prev_ctrl_ = true;

        const Eigen::Matrix<double, 6, 1> xdot_actual = J * qdot;

        // --- Desired Cartesian velocity (target_vel if fresh, else 0) ---
        Eigen::Matrix<double, 6, 1> xdot_des = Eigen::Matrix<double, 6, 1>::Zero();
        const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
        if (dt_vel < TARGET_VEL_TIMEOUT_SEC)
        {
            xdot_des.head<3>() = latest_target_vel_.head<3>();
            // /motomini/target_vel angular is body-frame. Convert it to the
            // base frame before comparing with J(q)·θ̇.
            xdot_des.tail<3>() = des_rot * latest_target_vel_.tail<3>();
        }

        const Eigen::Matrix<double, 6, 1> velocity_error = xdot_actual - xdot_des;

        // --- External/contact wrench placeholder. Future collision avoidance
        //     or force feedback should write a Cartesian wrench here. The sign
        //     convention below treats positive F_collision as a disturbance
        //     that the virtual dynamics should yield against.
        Eigen::Matrix<double, 6, 1> F_collision;
        F_collision.setZero();

        // --- Virtual acceleration ẍ_ref = M⁻¹·(K·e − D·ė − F − D·ẋ_ref) ---
        Eigen::Matrix<double, 6, 1> xddot_ref;
        xddot_ref.head<3>() = (k_pos_var * e_p_
                               - d_pos_var * velocity_error.head<3>()
                               - F_collision.head<3>()
                               - d_pos_var * xdot_ref_.head<3>())
                              / std::max(1e-9, m_pos_var);
        xddot_ref.tail<3>() = (k_ori_var * e_o_
                               - d_ori_var * velocity_error.tail<3>()
                               - F_collision.tail<3>()
                               - d_ori_var * xdot_ref_.tail<3>())
                              / std::max(1e-9, m_ori_var);

        // --- Integrate ẋ_ref ---
        const double dt_safe =
            (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
        xdot_ref_ += xddot_ref * dt_safe;

        // --- Safety clamps on the integrator state ---
        const double lin_n = xdot_ref_.head<3>().norm();
        if (lin_n > max_cart_linear_vel_ && lin_n > 1e-9)
            xdot_ref_.head<3>() *= max_cart_linear_vel_ / lin_n;
        const double ang_n = xdot_ref_.tail<3>().norm();
        if (ang_n > max_cart_angular_vel_ && ang_n > 1e-9)
            xdot_ref_.tail<3>() *= max_cart_angular_vel_ / ang_n;

        // --- SR-inverse Jacobian → joint velocity command ---
        theta_d = calcSrInverse(J, w, w0_, k0_) * xdot_ref_;

        return true;
    }

    // Reset the virtual integrator state when entering IDLE/STOP or starting
    // INIT/POSE_FOLLOW.
    void resetVirtualState()
    {
        xdot_ref_.setZero();
        have_q_prev_ctrl_ = false;
        q_prev_ctrl_.resize(0);
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

    // Streaming desired pose — triggers IDLE → POSE_FOLLOW automatically
    void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        desired_pose_ = *msg;
        has_desired_pose_ = true;
        t_last_pose_cb_ = this->now();

        if (state_ == STATE_IDLE)
        {
            if (initTrackedPositions())
            {
                resetControlWindow();
                if (enable_seed_)
                    seed();
                state_ = STATE_POSE_FOLLOW;
                RCLCPP_INFO(this->get_logger(), "STATE_IDLE → STATE_POSE_FOLLOW");
            }
        }
    }

    // Streaming target velocity. Linear is base-frame, angular is body-frame
    // (post-multiplied into the desired-pose quaternion). Integrated by the
    // controller into desired_pose_ each tick while in STATE_POSE_FOLLOW.
    void targetVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_target_vel_ << msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z;
        t_last_target_vel_cb_ = this->now();

        // Velocity-only entry: seed desired_pose_ from current EE and start tracking.
        if (state_ == STATE_IDLE && latest_target_vel_.norm() > 0.0)
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
                resetControlWindow();
                if (enable_seed_)
                    seed();
                state_ = STATE_POSE_FOLLOW;
                RCLCPP_INFO(this->get_logger(),
                            "STATE_IDLE → STATE_POSE_FOLLOW (target_vel input)");
            }
        }
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
        resetControlWindow();
        tracked_positions_.clear();
        tracked_velocities_.clear();
        is_init_done_ = false;
        res->success = true;
        res->message = "Reset to STATE_IDLE";
        RCLCPP_INFO(this->get_logger(), "/start → STATE_IDLE");
    }

    // /pose_following/stop → STATE_STOP (immediate, no more publishing)
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_STOP;
        resetControlWindow();
        res->success = true;
        res->message = "Stopped — STATE_STOP";
        RCLCPP_INFO(this->get_logger(), "/stop → STATE_STOP");
    }

    // /pose_following/init_start → STATE_INIT
    // Requires: init_pose set via /pose_following/init_pose, current state = IDLE
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
        resetControlWindow();
        if (enable_seed_)
            seed();
        state_ = STATE_INIT;
        res->success = true;
        res->message = "STATE_IDLE → STATE_INIT";
        RCLCPP_INFO(this->get_logger(), "/init_start → STATE_INIT");
    }

    // ============================================================
    // STATE HANDLERS
    // ============================================================

    // IDLE — sync internal positions from real robot; no publishing
    void handleIdle()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            resetVirtualState();
            RCLCPP_INFO(this->get_logger(), "STATE_IDLE: syncing joint state, no output.");
        }
        if (last_joint_state_)
            initTrackedPositions();
    }

    // STOP — zero velocity, sync from real robot; no publishing
    void handleStop()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            resetVirtualState();
            RCLCPP_INFO(this->get_logger(),
                        "STATE_STOP: velocity zeroed, no output. Call /start to resume.");
        }
        if (last_joint_state_)
            initTrackedPositions();
    }

    // INIT — move to init_pose_ using adaptive admittance + SR-inverse
    void handleInit()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
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

        // Velocity safety
        if (!checkVelocityLimits(theta_d))
        {
            RCLCPP_WARN(this->get_logger(),
                        "[INIT] Joint velocity exceeded limit → STATE_STOP");
            resetControlWindow();
            state_ = STATE_STOP;
            return;
        }

        // Integrate
        std::vector<double> prev_pos = tracked_positions_;
        for (size_t i = 0; i < tracked_positions_.size(); ++i)
        {
            tracked_positions_[i] += theta_d[static_cast<Eigen::Index>(i)] * dt;
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

        RCLCPP_DEBUG(this->get_logger(),
                     "[INIT] err(%.4f,%.4f,%.4f)",
                     dx, dy, dz);
    }

    // POSE_FOLLOW — track streaming desired_pose_ via adaptive admittance + SR-inverse
    void handlePoseFollow()
    {
        if (last_state_ != state_)
        {
            last_state_ = state_;
            resetVirtualState();
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

        // Pose timeout — go idle if input stalls
        double dt_cb = (this->now() - t_last_pose_cb_).seconds();
        if (dt_cb > POSE_TIMEOUT_SEC)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Pose input timeout (%.2f s > %.2f s) → STATE_IDLE", dt_cb, POSE_TIMEOUT_SEC);
            resetControlWindow();
            state_ = STATE_IDLE;
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

        // Integrate streaming target_vel into the desired pose.
        // linear = base frame (added directly), angular = body frame (post-mul).
        const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
        if (dt_vel < TARGET_VEL_TIMEOUT_SEC && latest_target_vel_.norm() > 0.0)
        {
            des_pos.x() += latest_target_vel_(0) * dt;
            des_pos.y() += latest_target_vel_(1) * dt;
            des_pos.z() += latest_target_vel_(2) * dt;
            const Eigen::Quaterniond delta =
                Eigen::AngleAxisd(latest_target_vel_(3) * dt, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(latest_target_vel_(4) * dt, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(latest_target_vel_(5) * dt, Eigen::Vector3d::UnitZ());
            q_des = (q_des * delta).normalized();

            // Persist the integrated target so the next pose-only callback
            // doesn't snap the robot back to a stale absolute target.
            desired_pose_.pose.position.x = des_pos.x();
            desired_pose_.pose.position.y = des_pos.y();
            desired_pose_.pose.position.z = des_pos.z();
            desired_pose_.pose.orientation.w = q_des.w();
            desired_pose_.pose.orientation.x = q_des.x();
            desired_pose_.pose.orientation.y = q_des.y();
            desired_pose_.pose.orientation.z = q_des.z();
            t_last_pose_cb_ = this->now(); // velocity stream keeps follow alive
        }

        Eigen::VectorXd theta_d;
        if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt, theta_d))
            return;

        RCLCPP_DEBUG(this->get_logger(),
                     "[FOLLOW] e_p=(%.4f,%.4f,%.4f) e_o=(%.4f,%.4f,%.4f)",
                     e_p_(0), e_p_(1), e_p_(2), e_o_(0), e_o_(1), e_o_(2));

        // Velocity safety
        if (!checkVelocityLimits(theta_d))
        {
            RCLCPP_WARN(this->get_logger(),
                        "[FOLLOW] Joint velocity exceeded limit → STATE_STOP");
            resetControlWindow();
            state_ = STATE_STOP;
            return;
        }

        // Integrate: stateless overwrite — always base on actual joint state
        std::vector<double> prev_pos = tracked_positions_;
        for (size_t i = 0; i < tracked_positions_.size(); ++i)
        {
            tracked_positions_[i] += theta_d[static_cast<Eigen::Index>(i)] * dt;
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

    // --- Adaptive Cartesian admittance gains ---
    double m_pos_min_, m_pos_max_, k_pos_min_, k_pos_max_, zeta_pos_;
    double m_ori_min_, m_ori_max_, k_ori_min_, k_ori_max_, zeta_ori_;
    double adaptive_lambda_, adaptive_alpha_pos_, adaptive_alpha_ori_;
    double max_cart_linear_vel_, max_cart_angular_vel_;

    // --- Virtual Cartesian-velocity integrator state ---
    Eigen::Matrix<double, 6, 1> xdot_ref_{Eigen::Matrix<double,6,1>::Zero()};

    // --- Numerical θ̇ history for the controller's velocity-error term ---
    Eigen::VectorXd q_prev_ctrl_;
    rclcpp::Time t_prev_q_ctrl_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_ctrl_{false};

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
    rclcpp::Time t_start_;             // reset at each IDLE → active transition
    rclcpp::Time t_last_;              // last tick timestamp
    rclcpp::Time t_last_pose_cb_;      // last desired-pose callback time
    rclcpp::Time t_last_target_vel_cb_;// last target-velocity callback time

    // --- Streaming target velocity (linear=base, angular=body) ---
    Eigen::Matrix<double, 6, 1> latest_target_vel_{Eigen::Matrix<double,6,1>::Zero()};

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
