#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_rosutils/utils.h>

namespace robot_planning
{

  constexpr double kNodeRate = 50.0;
  constexpr int kNumberOfJoint = 6;

  constexpr double kJoint1UpperRad = 170.0 * M_PI / 180.0;
  constexpr double kJoint1LowerRad = -170.0 * M_PI / 180.0;
  constexpr double kJoint2UpperRad = 90.0 * M_PI / 180.0;
  constexpr double kJoint2LowerRad = -85.0 * M_PI / 180.0;
  constexpr double kJoint3UpperRad = 120.0 * M_PI / 180.0;
  constexpr double kJoint3LowerRad = -175.0 * M_PI / 180.0;
  constexpr double kJoint4UpperRad = 140.0 * M_PI / 180.0;
  constexpr double kJoint4LowerRad = -140.0 * M_PI / 180.0;
  constexpr double kJoint5UpperRad = 210.0 * M_PI / 180.0;
  constexpr double kJoint5LowerRad = -30.0 * M_PI / 180.0;
  constexpr double kJoint6UpperRad = 360.0 * M_PI / 180.0;
  constexpr double kJoint6LowerRad = -360.0 * M_PI / 180.0;

  constexpr double kJoint1VelLimit = M_PI * 7.0 / 4.0;
  constexpr double kJoint2VelLimit = M_PI * 7.0 / 4.0;
  constexpr double kJoint3VelLimit = M_PI * 7.0 / 3.0;
  constexpr double kJoint4VelLimit = M_PI * 10.0 / 3.0;
  constexpr double kJoint5VelLimit = M_PI * 10.0 / 3.0;
  constexpr double kJoint6VelLimit = M_PI * 10.0 / 3.0;
  constexpr double kSafetyVelocityAlpha = 1.0;

  constexpr double kDefaultLegacyKpMax = 3.5;
  constexpr double kDefaultLegacyKoMax = 2.5;
  constexpr double kDefaultLegacyKdp = 0.25;
  constexpr double kDefaultLegacyKdo = 0.25;

  constexpr double kDefaultMPosMin = 0.5;
  constexpr double kDefaultMPosMax = 5.0;
  constexpr double kDefaultKPosMin = 5.0;
  constexpr double kDefaultKPosMax = 50.0;
  constexpr double kDefaultZetaPos = 0.9;
  constexpr double kDefaultMOriMin = 0.2;
  constexpr double kDefaultMOriMax = 2.0;
  constexpr double kDefaultKOriMin = 2.0;
  constexpr double kDefaultKOriMax = 20.0;
  constexpr double kDefaultZetaOri = 0.9;
  constexpr double kDefaultAdaptiveLambda = 1.0;
  constexpr double kDefaultAdaptiveAlphaPos = 30.0;
  constexpr double kDefaultAdaptiveAlphaOri = 6.0;
  constexpr double kDefaultMaxCartLinearVel = 0.5;
  constexpr double kDefaultMaxCartAngularVel = 1.5;

  constexpr double kDefaultIGainPos = 1200.0;
  constexpr double kDefaultIGainOri = 350.0;
  constexpr double kDefaultIClampPos = 0.04;
  constexpr double kDefaultIClampOri = 0.40;
  constexpr double kDefaultIForceRef = 1.0;
  constexpr double kDefaultIForceShape = 2.0;
  constexpr double kDefaultIForceMinScale = 0.0;
  constexpr double kDefaultICollisionDecayRate = 6.0;

  constexpr double kDefaultW0 = 0.01;
  constexpr double kDefaultK0 = 0.01;

  constexpr double kPositionErrorThreshold = 0.0005;
  constexpr double kSafetyJointPaddingRad = 5.0 * M_PI / 180.0;
  constexpr double kPoseTimeoutSec = 3.0;
  constexpr double kTargetVelTimeoutSec = 0.5;
  constexpr double kArmPreDelaySec = 0.5;
  constexpr double kArmPostDelaySec = 1.0;

  struct AdaptivePhaseConfig
  {
    double m_pos_min{kDefaultMPosMin};
    double m_pos_max{kDefaultMPosMax};
    double k_pos_min{kDefaultKPosMin};
    double k_pos_max{kDefaultKPosMax};
    double zeta_pos{kDefaultZetaPos};
    double m_ori_min{kDefaultMOriMin};
    double m_ori_max{kDefaultMOriMax};
    double k_ori_min{kDefaultKOriMin};
    double k_ori_max{kDefaultKOriMax};
    double zeta_ori{kDefaultZetaOri};
    double adaptive_lambda{kDefaultAdaptiveLambda};
    double adaptive_alpha_pos{kDefaultAdaptiveAlphaPos};
    double adaptive_alpha_ori{kDefaultAdaptiveAlphaOri};
    double max_cart_linear_vel{kDefaultMaxCartLinearVel};
    double max_cart_angular_vel{kDefaultMaxCartAngularVel};
    double i_gain_pos{kDefaultIGainPos};
    double i_gain_ori{kDefaultIGainOri};
    double i_clamp_pos{kDefaultIClampPos};
    double i_clamp_ori{kDefaultIClampOri};
    double i_force_ref{kDefaultIForceRef};
    double i_force_shape{kDefaultIForceShape};
    double i_force_min_scale{kDefaultIForceMinScale};
    double i_collision_decay_rate{kDefaultICollisionDecayRate};
  };

  class MotoMiniFeedbackStreamNode : public rclcpp::Node
  {
  public:
    MotoMiniFeedbackStreamNode();

  private:
    enum State
    {
      STATE_IDLE = 0,
      STATE_POSE_FOLLOW = 1,
      STATE_STOP = 2,
      STATE_INIT = 3,
      STATE_ARMING = 4,
    };

    enum class ControlPhase
    {
      Default = 0,
      Approach,
      Near,
      Tracking,
    };

    struct PhaseProfileSet
    {
      AdaptivePhaseConfig base{};
      AdaptivePhaseConfig approach{};
      AdaptivePhaseConfig near{};
      AdaptivePhaseConfig tracking{};
    };

    void declareParameters();
    void loadParameters();
    void logConfiguration() const;
    bool initializeKinematics();

    static Eigen::MatrixXd calcPseudoInverse(const Eigen::MatrixXd &J);
    static Eigen::MatrixXd calcSrInverse(const Eigen::MatrixXd &J, double w, double w0, double k0);
    static Eigen::Vector3d orientationError(const Eigen::Matrix3d &des_R, const Eigen::Matrix3d &cur_R);

    std::string phaseParamName(const std::string &phase_prefix, const std::string &suffix) const;
    void declarePhaseParameters(const std::string &phase_prefix, const AdaptivePhaseConfig &defaults);
    AdaptivePhaseConfig readPhaseParameters(
        const std::string &phase_prefix, const AdaptivePhaseConfig &fallback) const;
    void sanitizeAdaptiveConfig(AdaptivePhaseConfig &config) const;
    void refreshActivePhaseConfig();
    ControlPhase parseControlPhase(const std::string &text) const;
    const char *phaseLabel(ControlPhase phase) const;
    bool updateAdaptiveField(AdaptivePhaseConfig &config, const std::string &key, double value) const;
    rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter> &params);

    bool currentJointVector(Eigen::VectorXd &q) const;
    Eigen::VectorXd filterJointVelocity(const Eigen::VectorXd &qdot_raw, double dt);
    bool getMeasuredJointVelocity(
        const Eigen::VectorXd &q, double dt_hint, Eigen::VectorXd &qdot_out,
        Eigen::VectorXd &q_prev, rclcpp::Time &t_prev_q, bool &have_q_prev);
    bool initTrackedPositions();
    bool getEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Matrix3d &rot) const;
    void publishFeedback();

    double clampJointVelocityLimits(Eigen::VectorXd &theta_d);
    bool checkCartesianVelocitySafety(const Eigen::Matrix<double, 6, 1> &xdot_actual);
    void clampCartesianVelocity(Eigen::Matrix<double, 6, 1> &xdot) const;
    void limitCartesianAcceleration(
        Eigen::Matrix<double, 6, 1> &xdot_next,
        const Eigen::Matrix<double, 6, 1> &xdot_prev,
        double dt) const;
    bool checkPositionLimits(const std::vector<double> &pos) const;

    void publishToTopic(
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &pub,
        const std::vector<double> &pos, const std::vector<double> &vel, double time_sec);
    void publishArmInit();
    void seed();
    void publishTrajectory(const std::vector<double> &pos, const std::vector<double> &vel);

    bool computeControlStep(
        const Eigen::VectorXd &q,
        const Eigen::Vector3d &des_pos,
        const Eigen::Matrix3d &des_rot,
        double dt,
        Eigen::VectorXd &theta_d);
    bool initializeReferenceVelocityFromMeasuredState();
    double lowPassAlpha(double cutoff_hz, double dt) const;
    double targetVelocityDeadband() const;
    double collisionForceAttackHz() const;
    double collisionForceReleaseHz() const;
    Eigen::Matrix<double, 6, 1> filterCollisionWrench(
        const Eigen::Matrix<double, 6, 1> &raw_wrench, double dt);
    void resetVirtualState();
    void resetControlWindow();

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void targetVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void collisionWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void collisionDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void collisionNormalCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void initPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlPhaseCallback(const std_msgs::msg::String::SharedPtr msg);
    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void initStartCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void handleIdle();
    void handleStop();
    void handleArming();
    void handleInit();
    void handlePoseFollow();
    void tick();

    const char *stateLabel(State state) const;
    void transitionTo(State next_state, const char *reason = nullptr);

    std::string urdf_xml_;
    std::string srdf_xml_;
    std::string manipulator_group_;
    std::string base_link_;
    std::string ee_link_;
    double rate_hz_{kNodeRate};
    double w0_{kDefaultW0};
    double k0_{kDefaultK0};
    bool enable_seed_{false};
    bool real_robot_{true};
    double velocity_filter_cutoff_hz_{15.0};
    double max_cart_linear_acc_{0.8};
    double max_cart_angular_acc_{2.5};
    double measured_cart_linear_vel_limit_{1.0};
    double measured_cart_angular_vel_limit_{3.0};
    bool integrate_target_vel_to_pose_{true};
    double collision_wrench_timeout_sec_{0.2};
    bool enable_collision_projection_{true};
    bool collision_goal_suppression_{true};
    double collision_guard_distance_{0.03};
    double collision_task_distance_{0.005};
    double collision_stop_distance_{0.001};
    double collision_projection_max_gamma_{1.0};
    double collision_constraint_timeout_sec_{0.2};
    double collision_force_scale_{1.0};
    double collision_force_max_{5.0};

    PhaseProfileSet phase_profiles_{};
    ControlPhase active_phase_{ControlPhase::Default};
    AdaptivePhaseConfig active_phase_config_{};

    Eigen::Matrix<double, 6, 1> xdot_ref_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::VectorXd q_prev_ctrl_;
    rclcpp::Time t_prev_q_ctrl_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_ctrl_{false};
    Eigen::VectorXd qdot_filtered_;
    bool first_velocity_read_{true};
    rclcpp::Time t_last_velocity_filter_update_{0, 0, RCL_ROS_TIME};
    bool have_velocity_filter_update_{false};

    State state_{STATE_IDLE};
    State last_state_{STATE_IDLE};
    bool is_init_done_{false};
    bool arm_init_sent_{false};
    bool has_desired_pose_{false};
    bool has_init_pose_{false};

    Eigen::Vector3d e_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e_o_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e_p_int_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e_o_int_{Eigen::Vector3d::Zero()};

    rclcpp::Time t_start_;
    rclcpp::Time t_last_;
    rclcpp::Time t_last_pose_cb_;
    rclcpp::Time t_last_target_vel_cb_;
    rclcpp::Time t_last_collision_wrench_cb_{0, 0, RCL_ROS_TIME};

    Eigen::Matrix<double, 6, 1> latest_target_vel_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> latest_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
    double latest_collision_distance_{std::numeric_limits<double>::infinity()};
    Eigen::Vector3d latest_collision_normal_{Eigen::Vector3d::Zero()};
    rclcpp::Time t_last_collision_distance_cb_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_collision_normal_cb_{0, 0, RCL_ROS_TIME};
    Eigen::Matrix<double, 6, 1> filtered_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};

    Eigen::VectorXd q_prev_;
    rclcpp::Time t_prev_q_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_{false};

    std::vector<double> tracked_positions_;
    std::vector<double> tracked_velocities_;
    std::vector<std::string> joint_names_;

    tesseract_environment::Environment::Ptr env_;
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_path_cmd_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_feedback_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_feedback_vel_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_target_vel_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_collision_wrench_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_collision_distance_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_collision_normal_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_desired_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_init_pose_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control_phase_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_start_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    geometry_msgs::msg::PoseStamped desired_pose_;
    geometry_msgs::msg::PoseStamped init_pose_;

    double streaming_time_{0.0};
    rclcpp::Time t_arming_start_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};
    State pending_state_{STATE_IDLE};
    bool is_active_{false};
  };

} // namespace robot_planning
