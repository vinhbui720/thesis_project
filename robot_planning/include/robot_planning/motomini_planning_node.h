/**
 * @file motomini_planning_node.h
 * @brief MotoMini ROS2 planning node — class declaration
 *
 * Split from the original monolithic motomini_planning_node.cpp so each
 * logical concern lives in its own translation unit:
 *   motomini_node_setup.cpp     — constructor, postInit, env init
 *   motomini_node_callbacks.cpp — ROS subscription callbacks + monitor
 *   motomini_node_tracking.cpp  — Cartesian servo with QP-CBF collision avoidance
 *   motomini_node_publish.cpp   — trajectory & status publishers
 *   motomini_node_main.cpp      — main()
 *
 * Tracking mode features QP-CBF (Control Barrier Function) collision avoidance:
 *   • Single continuous mode: always tracking the target via DLS-IK
 *   • At 30 Hz, solve QP: min ½‖J q̇ - v_des‖²_W + ½λ‖q̇‖² subject to safety constraints
 *   • Hard collision avoidance: nᵢᵀ J q̇ ≥ −γ(dᵢ − d_safe) for each near-contact
 *   • Result: robot curves smoothly around obstacles while tracking target
 *   • Mathematically rigorous safety, no state machine, no background threads
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
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

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

    // ---- Publishers ----
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_tracking_stream_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_online_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracked_pose_;

    // ---- Timers ----
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr tracking_timer_;
    rclcpp::TimerBase::SharedPtr tracking_stream_timer_;
    rclcpp::TimerBase::SharedPtr wp_tf_timer_;

    // ---- Execution Monitoring ----
    bool is_executing_{false};
    std::vector<double> final_joint_target_;
    std::vector<std::string> target_joint_names_;
    rclcpp::Time execution_start_time_;
    double expected_execution_duration_{0.0};
    static constexpr double JOINT_TOLERANCE = 0.05; // radians (~2.8 deg)
    static constexpr double TIMEOUT_BUFFER = 5.0;   // seconds

    // ---- Debug ----
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ---- MPC Tracking Mode ----
    bool tracking_enabled_{false}; // runtime switch: true = MPC tracking, false = static
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;
    std::vector<Eigen::VectorXd> horizon_joints_; // size N
    Eigen::VectorXd current_joints_;              // latest from /joint_states
    Eigen::Isometry3d current_target_pose_;       // latest from TF/topic
    rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d target_velocity_linear_{0.0, 0.0, 0.0};
    Eigen::Vector3d target_velocity_angular_{0.0, 0.0, 0.0};
    bool target_initialized_{false};

    mutable std::mutex _mpc_state_mutex;
    mutable std::mutex _mpc_target_mutex;
    mutable std::shared_mutex env_mutex_;
    std::shared_ptr<trajopt_ifopt::CollisionCache> mpc_collision_cache_;

    // Tesseract Planning Factory
    std::unique_ptr<tesseract_planning::TaskComposerPluginFactory> task_factory_;
    std::unique_ptr<tesseract_planning::TaskComposerExecutor> task_executor_;
    tesseract_planning::TaskComposerNode::UPtr mpc_task_;

    rclcpp::TimerBase::SharedPtr mpc_timer_; // 50 Hz

    // MPC Config
    int mpc_horizon_n_{10};
    double mpc_dt_{0.02};
    double mpc_w_cart_{10.0};
    double mpc_w_vel_{1.0};
    double mpc_w_acc_{0.5};
    double mpc_d_safe_{0.01};
    int mpc_max_iter_{3};
    int mpc_coll_type_{0}; // 0 = DISCRETE, 1 = CONTINUOUS
    std::string ee_link_{"magnetic_link"};
    std::string base_link_{"world"};
    Eigen::MatrixX2d joint_limits_; // [min, max] for each joint

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracking_target_;

    // ---- DLS Tracking Parameters ----
    double dls_damping_{0.1};     // Damping factor for DLS-IK (higher = more robust, less precise)
    double qp_velocity_reg_{0.5}; // Damping/regularization for DLS (replaces QP's λ)
    size_t qp_fallback_count_{0}; // Track failures for diagnostics (even though we're not using QP anymore)

    // Repulsion/Relaxation parameters
    double repulsion_lookahead_{0.01};
    double repulsion_safety_{0.02};
    double repulsion_max_speed_{0.20};
    double relax_distance_{0.5};
    double relax_z_min_factor_{0.5};
    double relax_rot_min_factor_{0.10};

    // Private helpers
    bool initializeEnvironment();

    // MPC Phase Methods
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void mpcTimerCallback();
    Eigen::Isometry3d predictTargetPose(int step_k);
    Eigen::VectorXd computeDlsExtrapolation(const Eigen::VectorXd &q_last, const Eigen::Isometry3d &target_next);
    Eigen::VectorXd solveQpVelocity(const Eigen::VectorXd &q, const Eigen::Isometry3d &target_pose);
    Eigen::Matrix<double, 6, 1> computeCartesianRepulsion(const Eigen::VectorXd &q,
                                                          Eigen::Matrix<double, 6, 1> &weight_diag,
                                                          double &min_dist);
    void buildAndPublishTrajectory();

    // Callbacks (motomini_node_callbacks.cpp)
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void targetPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void clearCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void trackingControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void monitorExecution();
    void publishWaypointsTFs();

    // Publish helpers (motomini_node_publish.cpp)
    void publishStatus(const std::string &status);
    void publishTrajectory(const tesseract_common::JointTrajectory &tess_traj,
                           const std::vector<std::string> &joint_names,
                           double start_delay_sec = 0.10,
                           double min_step_dt_sec = 0.02);
    void publishTrackingTrajectory(const tesseract_common::JointTrajectory &tess_traj,
                                   const std::vector<std::string> &joint_names);

    // Parameter change callback — propagates GUI/service param updates to the planner
    rcl_interfaces::msg::SetParametersResult
    onParameterChange(const std::vector<rclcpp::Parameter> &params);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // ROBOT_PLANNING_MOTOMINI_PLANNING_NODE_H
