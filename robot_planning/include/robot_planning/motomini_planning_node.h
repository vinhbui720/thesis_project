/**
 * @file motomini_planning_node.h
 * @brief MotoMini ROS2 planning node — class declaration
 *
 * Split from the original monolithic motomini_planning_node.cpp so each
 * logical concern lives in its own translation unit:
 *   motomini_node_setup.cpp     — constructor, postInit, env init
 *   motomini_node_callbacks.cpp — ROS subscription callbacks + monitor
 *   motomini_node_tracking.cpp  — switchable Cartesian feedback-stream controller
 *   motomini_node_publish.cpp   — trajectory & status publishers
 *   motomini_node_main.cpp      — main()
 *
 * This node retains the offline planner and waypoint accumulation, and also
 * embeds the feedback-stream controller from motomini_feedback_stream.cpp
 * behind /tracking_control.
 *
 * @author Bùi Quang Vinh
 */

#ifndef ROBOT_PLANNING_MOTOMINI_PLANNING_NODE_H
#define ROBOT_PLANNING_MOTOMINI_PLANNING_NODE_H

// Planning logic
#include <robot_planning/motomini_planning.h>

// ROS2 core
#include "rclcpp/rclcpp.hpp"

// ROS2 message types
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Tesseract Environment & Viz
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_visualization/visualization.h>

// Tesseract Command Language
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/utils.h>

// Tesseract Task Composer
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

// Tesseract Common
#include <tesseract_common/joint_state.h>
#include <tesseract_common/profile_dictionary.h>

// Eigen
#include <Eigen/Geometry>

// STL
#include <limits>
#include <memory>
#include <string>
#include <vector>

class MotoMiniPlanningNode : public rclcpp::Node
{
public:
    MotoMiniPlanningNode();
    ~MotoMiniPlanningNode();
    void postInit();

private:
    // ---- Configuration ----
    std::string urdf_xml_;
    std::string srdf_xml_;

    // ---- Core Objects ----
    std::shared_ptr<tesseract_environment::Environment> env_;
    std::shared_ptr<Vinhtesseract_examples::MotoMiniPlanning> planner_;
    std::shared_ptr<tesseract_visualization::Visualization> plotter_;
    std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitor> monitor_;

    // ---- Data Cache ----
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::vector<geometry_msgs::msg::Pose> accumulated_targets_;

    // ---- Subscribers ----
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_targets_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_clear_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tracking_control_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_desired_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_init_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_target_vel_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_collision_wrench_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_collision_distance_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_collision_normal_;

    // ---- Publishers ----
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_path_cmd_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_joint_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_online_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracked_pose_;

    // Always-on Cartesian feedback (matches motomini_feedback_stream contract):
    //   /motomini/feedback     — current EE pose as Twist (linear=xyz, angular=rpy)
    //   /motomini/feedback_vel — current Cartesian velocity J(q)·θ̇
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_feedback_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_feedback_vel_;

    // External offline trajectory streamer control. When tracking mode is on,
    // stop motomini_traj_streamer so only tracking writes /joint_command.
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr traj_stream_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr traj_stream_stop_client_;

    // ---- Timers ----
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp::TimerBase::SharedPtr wp_tf_timer_;

    // ---- Execution / publish status ----
    bool is_executing_{false};
    rclcpp::Time execution_start_time_;
    double expected_execution_duration_{0.0};

    // ---- Debug ----
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ---- Kinematics (used for FK/Jacobian when publishing feedback) ----
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;
    std::string ee_link_{"tool0"};
    std::string base_link_{"world"};
    Eigen::MatrixX2d joint_limits_;
    Eigen::MatrixX2d velocity_limits_;
    std::vector<std::string> joint_names_;

    // ---- Measured θ̇ history for Cartesian feedback (J·θ̇) ----
    Eigen::VectorXd q_prev_;
    rclcpp::Time t_prev_q_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_{false};
    Eigen::VectorXd qdot_filtered_;
    bool first_velocity_read_{true};
    rclcpp::Time t_last_velocity_filter_update_{0, 0, RCL_ROS_TIME};
    bool have_velocity_filter_update_{false};

    enum class TrackingStreamState
    {
        IDLE = 0,
        POSE_FOLLOW = 1,
        STOP = 2,
        INIT = 3,
        ARMING = 4,
    };

    // ---- Feedback-stream mode ----
    bool tracking_enabled_{false};
    TrackingStreamState tracking_state_{TrackingStreamState::IDLE};
    TrackingStreamState last_tracking_state_{TrackingStreamState::IDLE};
    bool has_desired_pose_{false};
    bool has_init_pose_{false};
    bool is_init_done_{false};
    bool enable_seed_{false};
    bool real_robot_{true};
    bool integrate_target_vel_to_pose_{true};

    geometry_msgs::msg::PoseStamped desired_pose_;
    geometry_msgs::msg::PoseStamped init_pose_;
    std::vector<double> tracked_positions_;
    std::vector<double> tracked_velocities_;

    // ---- Feedback-stream timing ----
    double rate_hz_{50.0};
    rclcpp::Time t_start_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_pose_cb_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_target_vel_cb_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_collision_wrench_cb_{0, 0, RCL_ROS_TIME};

    // ---- Adaptive Cartesian admittance gains ----
    double m_pos_min_{0.5};
    double m_pos_max_{5.0};
    double k_pos_min_{5.0};
    double k_pos_max_{50.0};
    double zeta_pos_{0.9};
    double m_ori_min_{0.2};
    double m_ori_max_{2.0};
    double k_ori_min_{2.0};
    double k_ori_max_{20.0};
    double zeta_ori_{0.9};
    double adaptive_lambda_{1.0};
    double adaptive_alpha_pos_{30.0};
    double adaptive_alpha_ori_{6.0};
    double max_cart_linear_vel_{0.5};
    double max_cart_angular_vel_{1.5};
    double max_cart_linear_acc_{0.8};
    double max_cart_angular_acc_{2.5};
    double velocity_filter_cutoff_hz_{15.0};
    double measured_cart_linear_vel_limit_{1.0};
    double measured_cart_angular_vel_limit_{3.0};
    double w0_{0.01};
    double k0_{0.01};

    Eigen::Matrix<double, 6, 1> xdot_ref_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Vector3d e_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e_o_{Eigen::Vector3d::Zero()};

    // ---- Measured θ̇ history for safety/start initialization ----
    Eigen::VectorXd q_prev_ctrl_;
    rclcpp::Time t_prev_q_ctrl_{0, 0, RCL_ROS_TIME};
    bool have_q_prev_ctrl_{false};

    // ---- Streaming target velocity and collision input ----
    Eigen::Matrix<double, 6, 1> latest_target_vel_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> latest_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
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
    double latest_collision_distance_{std::numeric_limits<double>::infinity()};
    Eigen::Vector3d latest_collision_normal_{Eigen::Vector3d::Zero()};
    rclcpp::Time t_last_collision_distance_cb_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_last_collision_normal_cb_{0, 0, RCL_ROS_TIME};
    Eigen::Matrix<double, 6, 1> filtered_collision_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
    double streaming_time_{0.0};
    rclcpp::Time t_arming_start_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};
    TrackingStreamState pending_tracking_state_{TrackingStreamState::IDLE};
    bool is_active_{false};

    // Private helpers
    bool initializeEnvironment();

    // Feedback-stream controller (motomini_node_tracking.cpp)
    void publishFeedback();
    void feedbackTimerCallback();
    void trackingControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void initPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void targetVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void collisionWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void collisionDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void collisionNormalCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void sanitizeTrackingParameters();
    bool currentJointVector(Eigen::VectorXd &q) const;
    Eigen::VectorXd filterJointVelocity(const Eigen::VectorXd &qdot_raw, double dt);
    bool getMeasuredJointVelocity(const Eigen::VectorXd &q,
                                  double dt_hint,
                                  Eigen::VectorXd &qdot_out,
                                  Eigen::VectorXd &q_prev,
                                  rclcpp::Time &t_prev_q,
                                  bool &have_q_prev);
    double lowPassAlpha(double cutoff_hz, double dt) const;
    double targetVelocityDeadband() const;
    double collisionForceAttackHz() const;
    double collisionForceReleaseHz() const;
    Eigen::Matrix<double, 6, 1> filterCollisionWrench(
        const Eigen::Matrix<double, 6, 1> &raw_wrench,
        double dt);
    bool initializeReferenceVelocityFromMeasuredState();
    bool initTrackedPositions();
    bool getEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Matrix3d &rot) const;
    bool computeControlStep(const Eigen::VectorXd &q,
                            const Eigen::Vector3d &des_pos,
                            const Eigen::Matrix3d &des_rot,
                            double dt,
                            Eigen::VectorXd &theta_d);
    bool checkVelocityLimits(const Eigen::VectorXd &theta_d) const;
    bool checkCartesianVelocitySafety(const Eigen::Matrix<double, 6, 1> &xdot_actual);
    void clampCartesianVelocity(Eigen::Matrix<double, 6, 1> &xdot) const;
    void limitCartesianAcceleration(Eigen::Matrix<double, 6, 1> &xdot_next,
                                    const Eigen::Matrix<double, 6, 1> &xdot_prev,
                                    double dt) const;
    bool checkPositionLimits(const std::vector<double> &pos,
                             const std::vector<double> &reference = {}) const;
    void publishStreamPoint(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &pub,
                            const std::vector<double> &pos,
                            const std::vector<double> &vel,
                            double time_sec);
    void publishArmInit();
    void seedStreamingCommand();
    void publishStreamingTrajectory(const std::vector<double> &pos,
                                    const std::vector<double> &vel);
    void resetVirtualState();
    void resetControlWindow();
    void enterPoseFollowFromCurrentPose();
    void requestTrajectoryStreamerStart();
    void requestTrajectoryStreamerStop();
    void sendTriggerIfReady(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
                            const char *service_name);
    void handleTrackingIdle();
    void handleTrackingStop();
    void handleTrackingArming();
    void handleTrackingInit();
    void handleTrackingPoseFollow();

    // Callbacks (motomini_node_callbacks.cpp)
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void targetPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void clearCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void monitorExecution();
    void publishWaypointsTFs();

    // Publish helpers (motomini_node_publish.cpp)
    void publishStatus(const std::string &status);
    bool publishTrajectory(const tesseract_common::JointTrajectory &tess_traj,
                           const std::vector<std::string> &joint_names,
                           double start_delay_sec = 0.10,
                           double min_step_dt_sec = 0.02);

    // Parameter change callback — propagates GUI/service param updates to the planner
    rcl_interfaces::msg::SetParametersResult
    onParameterChange(const std::vector<rclcpp::Parameter> &params);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // ROBOT_PLANNING_MOTOMINI_PLANNING_NODE_H
