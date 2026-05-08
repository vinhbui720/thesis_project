/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — embedded feedback-stream tracking controller.
 *
 * This adapts the control structure from motomini_feedback_stream.cpp into the
 * planning node. Keep this path aligned with motomini_feedback_stream.cpp.
 *
 * Mode switch:
 *   /tracking_control true  -> enable streaming controller
 *   /tracking_control false -> disable streaming controller and return to planning
 *
 * Streaming topics intentionally match motomini_feedback_stream.cpp:
 *   Sub: /motomini/target_pose, /motomini/target_vel,
 *        /pose_following/init_pose, /motomini/collision_*
 *   Pub: /joint_path_command, joint_command,
 *        /motomini/feedback, /motomini/feedback_vel
 */

#include <robot_planning/motomini_planning_node.h>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    constexpr int NUMBER_OF_JOINT = 6;
    constexpr double SAFETY_VELOCITY_ALPHA = 1.0;
    constexpr double SAFETY_JOINT_PADDING_RAD = 5.0 * M_PI / 180.0;
    constexpr double POSITION_ERROR_THRESHOLD = 0.0005;
    constexpr double POSE_TIMEOUT_SEC = 3.0;
    constexpr double TARGET_VEL_TIMEOUT_SEC = 0.5;
    constexpr double ARM_PRE_DELAY_S = 0.5;
    constexpr double ARM_POST_DELAY_S = 1.0;

    const std::array<std::string, NUMBER_OF_JOINT> FALLBACK_JOINT_NAMES = {
        "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

    const std::array<double, NUMBER_OF_JOINT> FALLBACK_LOWER = {
        -170.0 * M_PI / 180.0,
        -85.0 * M_PI / 180.0,
        -175.0 * M_PI / 180.0,
        -140.0 * M_PI / 180.0,
        -30.0 * M_PI / 180.0,
        -360.0 * M_PI / 180.0};

    const std::array<double, NUMBER_OF_JOINT> FALLBACK_UPPER = {
        170.0 * M_PI / 180.0,
        90.0 * M_PI / 180.0,
        120.0 * M_PI / 180.0,
        140.0 * M_PI / 180.0,
        210.0 * M_PI / 180.0,
        360.0 * M_PI / 180.0};

    const std::array<double, NUMBER_OF_JOINT> FALLBACK_VELOCITY = {
        M_PI * 7.0 / 4.0,
        M_PI * 7.0 / 4.0,
        M_PI * 7.0 / 3.0,
        M_PI * 10.0 / 3.0,
        M_PI * 10.0 / 3.0,
        M_PI * 10.0 / 3.0};

    Eigen::MatrixXd calcSrInverse(const Eigen::MatrixXd &jacobian,
                                  double manipulability,
                                  double w0,
                                  double k0)
    {
        const double w0_safe = std::max(1e-9, w0);
        const double damping =
            (manipulability < w0_safe)
                ? k0 * std::pow(1.0 - manipulability / w0_safe, 2.0)
                : 0.0;
        const Eigen::Index rows = jacobian.rows();
        const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(rows, rows);
        return jacobian.transpose() *
               (jacobian * jacobian.transpose() + damping * identity).inverse();
    }

    Eigen::Vector3d orientationError(const Eigen::Matrix3d &desired,
                                     const Eigen::Matrix3d &current)
    {
        const Eigen::Matrix3d error = desired * current.transpose();
        const double angle =
            std::acos(std::clamp(0.5 * (error.trace() - 1.0), -1.0, 1.0));
        if (std::abs(angle) < 1e-8)
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d axis(error(2, 1) - error(1, 2),
                             error(0, 2) - error(2, 0),
                             error(1, 0) - error(0, 1));
        axis /= (2.0 * std::sin(angle));
        return angle * axis;
    }

    Eigen::Quaterniond normalizedQuaternion(const geometry_msgs::msg::Quaternion &msg)
    {
        Eigen::Quaterniond q(msg.w, msg.x, msg.y, msg.z);
        if (q.norm() < 1e-9)
            return Eigen::Quaterniond::Identity();
        q.normalize();
        return q;
    }
} // namespace

const char *MotoMiniPlanningNode::trackingStateLabel(TrackingStreamState state) const
{
    switch (state)
    {
    case TrackingStreamState::IDLE:
        return "STATE_IDLE";
    case TrackingStreamState::POSE_FOLLOW:
        return "STATE_POSE_FOLLOW";
    case TrackingStreamState::STOP:
        return "STATE_STOP";
    case TrackingStreamState::INIT:
        return "STATE_INIT";
    case TrackingStreamState::ARMING:
        return "STATE_ARMING";
    default:
        return "STATE_UNKNOWN";
    }
}

void MotoMiniPlanningNode::transitionTrackingState(TrackingStreamState next_state,
                                                   const char *reason)
{
    if (tracking_state_ == next_state)
        return;

    if (reason != nullptr && reason[0] != '\0')
    {
        RCLCPP_INFO(this->get_logger(), "%s --> %s (%s)",
                    trackingStateLabel(tracking_state_),
                    trackingStateLabel(next_state),
                    reason);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "%s --> %s",
                    trackingStateLabel(tracking_state_),
                    trackingStateLabel(next_state));
    }
    tracking_state_ = next_state;
    // Publish the new state so external nodes (e.g. Python scripts) can poll it.
    publishStatus(trackingStateLabel(next_state));
}

void MotoMiniPlanningNode::sanitizeTrackingParameters()
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
    i_gain_pos_ = std::max(0.0, i_gain_pos_);
    i_gain_ori_ = std::max(0.0, i_gain_ori_);
    i_clamp_pos_ = std::max(0.0, i_clamp_pos_);
    i_clamp_ori_ = std::max(0.0, i_clamp_ori_);
    i_force_ref_ = std::max(1e-6, i_force_ref_);
    i_force_shape_ = std::max(0.0, i_force_shape_);
    i_force_min_scale_ = std::clamp(i_force_min_scale_, 0.0, 1.0);
    i_collision_decay_rate_ = std::max(0.0, i_collision_decay_rate_);
    max_cart_linear_acc_ = std::max(0.0, max_cart_linear_acc_);
    max_cart_angular_acc_ = std::max(0.0, max_cart_angular_acc_);
    velocity_filter_cutoff_hz_ = std::max(1.0, velocity_filter_cutoff_hz_);
    measured_cart_linear_vel_limit_ =
        std::max(0.0, measured_cart_linear_vel_limit_);
    measured_cart_angular_vel_limit_ =
        std::max(0.0, measured_cart_angular_vel_limit_);
    w0_ = std::max(1e-9, w0_);
    k0_ = std::max(0.0, k0_);

    collision_stop_distance_ = std::max(0.0, collision_stop_distance_);
    collision_task_distance_ =
        std::max(collision_stop_distance_, collision_task_distance_);
    collision_guard_distance_ =
        std::max(collision_task_distance_, collision_guard_distance_);
    collision_projection_max_gamma_ =
        std::clamp(collision_projection_max_gamma_, 0.0, 1.0);
    collision_force_scale_ = std::max(0.0, collision_force_scale_);
    collision_force_max_ = std::max(0.0, collision_force_max_);
}

bool MotoMiniPlanningNode::currentJointVector(Eigen::VectorXd &q) const
{
    if (!last_joint_state_ || joint_names_.empty())
        return false;

    q.resize(static_cast<Eigen::Index>(joint_names_.size()));
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        auto it = std::find(last_joint_state_->name.begin(),
                            last_joint_state_->name.end(),
                            joint_names_[i]);
        if (it == last_joint_state_->name.end())
            return false;

        const size_t idx =
            static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
        if (idx >= last_joint_state_->position.size())
            return false;
        q[static_cast<Eigen::Index>(i)] = last_joint_state_->position[idx];
    }
    return true;
}

Eigen::VectorXd MotoMiniPlanningNode::filterJointVelocity(const Eigen::VectorXd &qdot_raw,
                                                          double dt)
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

bool MotoMiniPlanningNode::getMeasuredJointVelocity(const Eigen::VectorXd &q,
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
            qdot_out = qdot_driver;
    }

    const rclcpp::Time now = this->now();
    const double dt_prev = have_q_prev ? (now - t_prev_q).seconds() : dt_hint;
    if (!velocity_valid)
    {
        // if (real_robot_)
        //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //                          "[VEL] real_robot=true but hardware velocity invalid — falling back to pose diff.");
        // else
        //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //                          "[VEL] real_robot=false → using pose-differentiation velocity.");
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

bool MotoMiniPlanningNode::initTrackedPositions()
{
    Eigen::VectorXd q;
    if (!currentJointVector(q))
        return false;

    tracked_positions_.assign(q.data(), q.data() + q.size());
    tracked_velocities_.assign(static_cast<size_t>(q.size()), 0.0);
    return true;
}

bool MotoMiniPlanningNode::getEEPose(const Eigen::VectorXd &q,
                                     Eigen::Vector3d &pos,
                                     Eigen::Matrix3d &rot) const
{
    if (!manip_)
        return false;

    const auto fk = manip_->calcFwdKin(q);
    auto it = fk.find(ee_link_);
    if (it == fk.end())
        return false;

    pos = it->second.translation();
    rot = it->second.rotation();
    return true;
}

void MotoMiniPlanningNode::publishFeedback()
{
    Eigen::VectorXd q;
    if (!currentJointVector(q))
        return;

    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    if (!getEEPose(q, pos, rot))
        return;

    const Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    if (pub_feedback_)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = pos.x();
        msg.linear.y = pos.y();
        msg.linear.z = pos.z();
        msg.angular.x = rpy.x();
        msg.angular.y = rpy.y();
        msg.angular.z = rpy.z();
        pub_feedback_->publish(msg);
    }

    const double dt_hint = have_q_prev_ ? (this->now() - t_prev_q_).seconds()
                                        : (1.0 / std::max(1.0, rate_hz_));
    Eigen::VectorXd qdot;
    if (!getMeasuredJointVelocity(q, dt_hint, qdot, q_prev_, t_prev_q_, have_q_prev_))
        return;

    if (pub_feedback_vel_)
    {
        const Eigen::MatrixXd jacobian = manip_->calcJacobian(q, base_link_, ee_link_);
        const Eigen::VectorXd v_cart = jacobian * qdot;

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = base_link_;
        msg.twist.linear.x = v_cart(0);
        msg.twist.linear.y = v_cart(1);
        msg.twist.linear.z = v_cart(2);
        msg.twist.angular.x = v_cart(3);
        msg.twist.angular.y = v_cart(4);
        msg.twist.angular.z = v_cart(5);
        pub_feedback_vel_->publish(msg);
    }
}

bool MotoMiniPlanningNode::checkCartesianVelocitySafety(
    const Eigen::Matrix<double, 6, 1> &xdot_actual)
{
    const double linear = xdot_actual.head<3>().norm();
    const double angular = xdot_actual.tail<3>().norm();

    if (measured_cart_linear_vel_limit_ > 0.0 &&
        linear > measured_cart_linear_vel_limit_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Measured Cartesian linear velocity %.4f m/s exceeds tracking limit %.4f m/s",
                             linear, measured_cart_linear_vel_limit_);
        return false;
    }

    if (measured_cart_angular_vel_limit_ > 0.0 &&
        angular > measured_cart_angular_vel_limit_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Measured Cartesian angular velocity %.4f rad/s exceeds tracking limit %.4f rad/s",
                             angular, measured_cart_angular_vel_limit_);
        return false;
    }

    return true;
}

void MotoMiniPlanningNode::clampCartesianVelocity(Eigen::Matrix<double, 6, 1> &xdot) const
{
    const double linear_norm = xdot.head<3>().norm();
    if (linear_norm > max_cart_linear_vel_ && linear_norm > 1e-9)
        xdot.head<3>() *= max_cart_linear_vel_ / linear_norm;

    const double angular_norm = xdot.tail<3>().norm();
    if (angular_norm > max_cart_angular_vel_ && angular_norm > 1e-9)
        xdot.tail<3>() *= max_cart_angular_vel_ / angular_norm;
}

void MotoMiniPlanningNode::limitCartesianAcceleration(
    Eigen::Matrix<double, 6, 1> &xdot_next,
    const Eigen::Matrix<double, 6, 1> &xdot_prev,
    double dt) const
{
    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));

    if (max_cart_linear_acc_ > 0.0)
    {
        const Eigen::Vector3d dv_linear = xdot_next.head<3>() - xdot_prev.head<3>();
        const double max_dv_linear = max_cart_linear_acc_ * dt_safe;
        if (dv_linear.norm() > max_dv_linear && dv_linear.norm() > 1e-9)
            xdot_next.head<3>() =
                xdot_prev.head<3>() + dv_linear.normalized() * max_dv_linear;
    }

    if (max_cart_angular_acc_ > 0.0)
    {
        const Eigen::Vector3d dv_angular = xdot_next.tail<3>() - xdot_prev.tail<3>();
        const double max_dv_angular = max_cart_angular_acc_ * dt_safe;
        if (dv_angular.norm() > max_dv_angular && dv_angular.norm() > 1e-9)
            xdot_next.tail<3>() =
                xdot_prev.tail<3>() + dv_angular.normalized() * max_dv_angular;
    }
}

bool MotoMiniPlanningNode::checkPositionLimits(const std::vector<double> &pos) const
{
    if (pos.empty())
        return false;

    for (size_t i = 0; i < pos.size(); ++i)
    {
        double lower = -std::numeric_limits<double>::infinity();
        double upper = std::numeric_limits<double>::infinity();
        if (joint_limits_.rows() == static_cast<Eigen::Index>(pos.size()) &&
            joint_limits_.cols() >= 2)
        {
            lower = joint_limits_(static_cast<Eigen::Index>(i), 0) + SAFETY_JOINT_PADDING_RAD;
            upper = joint_limits_(static_cast<Eigen::Index>(i), 1) - SAFETY_JOINT_PADDING_RAD;
        }
        else if (pos.size() == NUMBER_OF_JOINT)
        {
            lower = FALLBACK_LOWER[i] + SAFETY_JOINT_PADDING_RAD;
            upper = FALLBACK_UPPER[i] - SAFETY_JOINT_PADDING_RAD;
        }

        const bool below = pos[i] <= lower;
        const bool above = pos[i] >= upper;
        if (below || above)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Joint %zu position %.4f rad outside safe range [%.4f, %.4f]",
                        i, pos[i], lower, upper);
            return false;
        }
    }
    return true;
}

void MotoMiniPlanningNode::publishStreamPoint(
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &pub,
    const std::vector<double> &pos,
    const std::vector<double> &vel,
    double time_sec)
{
    if (!pub || pos.empty() || pos.size() != vel.size())
        return;

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    if (!joint_names_.empty())
    {
        traj.joint_names = joint_names_;
    }
    else
    {
        traj.joint_names.assign(FALLBACK_JOINT_NAMES.begin(), FALLBACK_JOINT_NAMES.end());
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = pos;
    point.velocities = vel;
    point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.0, time_sec));
    traj.points.push_back(point);
    pub->publish(traj);
}

void MotoMiniPlanningNode::publishArmInit()
{
    if (tracked_positions_.empty())
        return;

    std::vector<double> zero_vel(tracked_positions_.size(), 0.0);
    publishStreamPoint(pub_stream_path_cmd_, tracked_positions_, zero_vel, 0.5);
    RCLCPP_INFO(this->get_logger(),
                "Tracking arm init sent to /joint_path_command.");
}

void MotoMiniPlanningNode::seedStreamingCommand()
{
    if (tracked_positions_.empty())
        return;

    std::vector<double> zero_vel(tracked_positions_.size(), 0.0);
    publishStreamPoint(pub_stream_joint_cmd_, tracked_positions_, zero_vel, 0.0);
    streaming_time_ = 0.0;
    is_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Tracking seed sent to joint_command.");
}

void MotoMiniPlanningNode::publishStreamingTrajectory(const std::vector<double> &pos,
                                                      const std::vector<double> &vel)
{
    publishStreamPoint(pub_stream_joint_cmd_, pos, vel, streaming_time_);
    streaming_time_ += 1.0 / std::max(1.0, rate_hz_);
}

void MotoMiniPlanningNode::sendTriggerIfReady(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const char *service_name)
{
    if (!client || !client->service_is_ready())
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "%s service is not available; no trajectory-streamer mode request sent.",
                     service_name);
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request);
}

void MotoMiniPlanningNode::requestTrajectoryStreamerStart()
{
    sendTriggerIfReady(traj_stream_start_client_, "/pose_following/start");
}

void MotoMiniPlanningNode::requestTrajectoryStreamerStop()
{
    sendTriggerIfReady(traj_stream_stop_client_, "/pose_following/stop");
}

bool MotoMiniPlanningNode::initializeReferenceVelocityFromMeasuredState()
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

    const Eigen::MatrixXd jacobian = manip_->calcJacobian(q, base_link_, ee_link_);
    const Eigen::VectorXd xdot_measured = jacobian * qdot;
    if (xdot_measured.size() != 6 || !xdot_measured.allFinite())
    {
        xdot_ref_.setZero();
        return false;
    }

    xdot_ref_ = xdot_measured;
    clampCartesianVelocity(xdot_ref_);
    return true;
}

double MotoMiniPlanningNode::lowPassAlpha(double cutoff_hz, double dt) const
{
    if (dt <= 0.0)
        return 1.0;
    return std::clamp(1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt), 0.0, 1.0);
}

double MotoMiniPlanningNode::targetVelocityDeadband() const
{
    return std::max(0.002, 0.01 * max_cart_linear_vel_);
}

double MotoMiniPlanningNode::collisionForceAttackHz() const
{
    const double tau = std::max(0.02, 0.2 * collision_wrench_timeout_sec_);
    return 1.0 / tau;
}

double MotoMiniPlanningNode::collisionForceReleaseHz() const
{
    return std::max(1.0, 0.35 * collisionForceAttackHz());
}

Eigen::Matrix<double, 6, 1> MotoMiniPlanningNode::filterCollisionWrench(
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

void MotoMiniPlanningNode::resetVirtualState()
{
    xdot_ref_.setZero();
    e_p_int_.setZero();
    e_o_int_.setZero();
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

void MotoMiniPlanningNode::resetControlWindow()
{
    resetVirtualState();
    e_p_.setZero();
    e_o_.setZero();
    t_start_ = this->now();
    t_last_ = this->now();
}

void MotoMiniPlanningNode::enterPoseFollowFromCurrentPose()
{
    Eigen::VectorXd q;
    Eigen::Vector3d ee_pos;
    Eigen::Matrix3d ee_rot;
    if (!currentJointVector(q) || !getEEPose(q, ee_pos, ee_rot))
        return;

    const Eigen::Quaterniond qee(ee_rot);
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
        pending_tracking_state_ = TrackingStreamState::POSE_FOLLOW;
        transitionTrackingState(TrackingStreamState::ARMING, "target velocity command received");
    }
}

void MotoMiniPlanningNode::trackingControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg)
        return;

    if (msg->data)
    {
        if (tracking_enabled_)
            return;

        is_executing_ = false;
        planner_->stopOnlinePlanner();
        requestTrajectoryStreamerStop();

        tracking_enabled_ = true;
        tracking_state_ = TrackingStreamState::IDLE;
        last_tracking_state_ = TrackingStreamState::IDLE;
        arm_trigger_sent_ = false;
        pending_tracking_state_ = TrackingStreamState::IDLE;
        is_active_ = false;
        is_init_done_ = false;
        has_desired_pose_ = false;
        has_init_pose_ = false;
        latest_target_vel_.setZero();
        t_last_target_vel_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        resetControlWindow();

        publishStatus("Mode: Tracking");
        RCLCPP_INFO(this->get_logger(),
                    "Mode -> TRACKING feedback stream. Arming from current EE pose.");

        // Immediately arm from the current EE pose so the controller enters
        // POSE_FOLLOW and is ready before the first target arrives.
        // desired_pose_ is seeded to the current EE position so the robot holds
        // its current position until a real tracking target is received.
        enterPoseFollowFromCurrentPose();
    }
    else
    {
        if (!tracking_enabled_)
            return;

        tracking_enabled_ = false;
        tracking_state_ = TrackingStreamState::IDLE;
        last_tracking_state_ = TrackingStreamState::IDLE;
        arm_trigger_sent_ = false;
        pending_tracking_state_ = TrackingStreamState::IDLE;
        is_active_ = false;
        tracked_positions_.clear();
        tracked_velocities_.clear();
        has_desired_pose_ = false;
        has_init_pose_ = false;
        latest_target_vel_.setZero();
        t_last_target_vel_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        resetControlWindow();
        requestTrajectoryStreamerStart();

        publishStatus("Mode: Planning");
        RCLCPP_INFO(this->get_logger(),
                    "Mode -> PLANNING. Feedback remains active; streaming commands stopped.");
    }
}

void MotoMiniPlanningNode::desiredPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg)
        return;

    if (!tracking_enabled_)
        return;

    desired_pose_ = *msg;
    has_desired_pose_ = true;
    t_last_pose_cb_ = this->now();

    if (tracking_state_ == TrackingStreamState::IDLE ||
        tracking_state_ == TrackingStreamState::STOP)
    {
        if (initTrackedPositions())
        {
            pending_tracking_state_ = TrackingStreamState::POSE_FOLLOW;
            transitionTrackingState(TrackingStreamState::ARMING, "target pose received");
        }
    }
}

void MotoMiniPlanningNode::initPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg)
        return;

    if (!tracking_enabled_)
        return;

    init_pose_ = *msg;
    has_init_pose_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Tracking init pose received: (%.4f, %.4f, %.4f)",
                init_pose_.pose.position.x,
                init_pose_.pose.position.y,
                init_pose_.pose.position.z);

    if ((tracking_state_ == TrackingStreamState::IDLE ||
         tracking_state_ == TrackingStreamState::STOP) &&
        initTrackedPositions())
    {
        pending_tracking_state_ = TrackingStreamState::INIT;
        transitionTrackingState(TrackingStreamState::ARMING, "init pose received");
    }
}

void MotoMiniPlanningNode::targetVelCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (!msg)
        return;

    if (!tracking_enabled_)
        return;

    latest_target_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
        msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    t_last_target_vel_cb_ = this->now();

    if (tracking_state_ == TrackingStreamState::IDLE &&
        latest_target_vel_.head<3>().norm() > targetVelocityDeadband())
    {
        enterPoseFollowFromCurrentPose();
    }
}

void MotoMiniPlanningNode::collisionWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    if (!msg)
        return;

    latest_collision_wrench_ << -msg->wrench.force.x,
        -msg->wrench.force.y,
        -msg->wrench.force.z,
        -msg->wrench.torque.x,
        -msg->wrench.torque.y,
        -msg->wrench.torque.z;
    t_last_collision_wrench_cb_ = this->now();
}

void MotoMiniPlanningNode::collisionDistanceCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    if (!msg)
        return;

    latest_collision_distance_ = msg->data;
    t_last_collision_distance_cb_ = this->now();
}

void MotoMiniPlanningNode::collisionNormalCallback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    if (!msg)
        return;

    latest_collision_normal_ << msg->vector.x, msg->vector.y, msg->vector.z;
    t_last_collision_normal_cb_ = this->now();
}

bool MotoMiniPlanningNode::computeControlStep(const Eigen::VectorXd &q,
                                              const Eigen::Vector3d &des_pos,
                                              const Eigen::Matrix3d &des_rot,
                                              double dt,
                                              Eigen::VectorXd &theta_d)
{
    Eigen::Vector3d ee_pos;
    Eigen::Matrix3d ee_rot;
    if (!getEEPose(q, ee_pos, ee_rot))
        return false;

    e_p_ = des_pos - ee_pos;
    e_o_ = orientationError(des_rot, ee_rot);

    const double t_active = (this->now() - t_start_).seconds();
    const double s_t = 1.0 - std::exp(-adaptive_lambda_ * std::max(0.0, t_active));
    const double s_e_pos = std::tanh(adaptive_alpha_pos_ * e_p_.norm());
    const double s_e_ori = std::tanh(adaptive_alpha_ori_ * e_o_.norm());

    const Eigen::MatrixXd jacobian = manip_->calcJacobian(q, base_link_, ee_link_);
    const double manipulability =
        std::sqrt(std::max(0.0, (jacobian * jacobian.transpose()).determinant()));
    const double w0_safe = std::max(1e-9, w0_);
    const double s_w = std::clamp(manipulability / w0_safe, 0.2, 1.0);

    const double k_pos_var =
        s_w * (k_pos_min_ + s_t * s_e_pos * (k_pos_max_ - k_pos_min_));
    const double m_pos_var =
        m_pos_max_ - s_t * s_e_pos * (m_pos_max_ - m_pos_min_);
    const double d_pos_var =
        2.0 * zeta_pos_ * std::sqrt(std::max(1e-12, m_pos_var * k_pos_var));
    const double k_ori_var =
        s_w * (k_ori_min_ + s_t * s_e_ori * (k_ori_max_ - k_ori_min_));
    const double m_ori_var =
        m_ori_max_ - s_t * s_e_ori * (m_ori_max_ - m_ori_min_);
    const double d_ori_var =
        2.0 * zeta_ori_ * std::sqrt(std::max(1e-12, m_ori_var * k_ori_var));

    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    e_p_int_ += e_p_ * dt_safe;
    e_o_int_ += e_o_ * dt_safe;
    e_p_int_ = e_p_int_.cwiseMax(Eigen::Vector3d::Constant(-i_clamp_pos_))
                   .cwiseMin(Eigen::Vector3d::Constant(i_clamp_pos_));
    e_o_int_ = e_o_int_.cwiseMax(Eigen::Vector3d::Constant(-i_clamp_ori_))
                   .cwiseMin(Eigen::Vector3d::Constant(i_clamp_ori_));
    if (!e_p_int_.allFinite())
        e_p_int_.setZero();
    if (!e_o_int_.allFinite())
        e_o_int_.setZero();

    Eigen::VectorXd qdot;
    if (!getMeasuredJointVelocity(q, dt_safe, qdot, q_prev_ctrl_, t_prev_q_ctrl_, have_q_prev_ctrl_))
        return false;

    const Eigen::Matrix<double, 6, 1> xdot_actual = jacobian * qdot;
    if (!checkCartesianVelocitySafety(xdot_actual))
    {
        transitionTrackingState(TrackingStreamState::STOP, "Cartesian velocity safety exceeded");
        return false;
    }

    Eigen::Matrix<double, 6, 1> xdot_des =
        Eigen::Matrix<double, 6, 1>::Zero();
    if ((this->now() - t_last_target_vel_cb_).seconds() < TARGET_VEL_TIMEOUT_SEC)
    {
        xdot_des.head<3>() = latest_target_vel_.head<3>();
        // /motomini/target_vel angular is body-frame; damping uses base-frame
        // reference velocity.
        xdot_des.tail<3>() = des_rot * latest_target_vel_.tail<3>();
    }
    if (xdot_des.head<3>().norm() <= targetVelocityDeadband())
        xdot_des.head<3>().setZero();
    if (xdot_des.tail<3>().norm() <= targetVelocityDeadband())
        xdot_des.tail<3>().setZero();
    Eigen::Matrix<double, 6, 1> f_collision =
        Eigen::Matrix<double, 6, 1>::Zero();
    if ((this->now() - t_last_collision_wrench_cb_).seconds() <
        collision_wrench_timeout_sec_)
    {
        f_collision = latest_collision_wrench_;
    }
    f_collision.head<3>() *= collision_force_scale_;
    const double f_norm = f_collision.head<3>().norm();
    if (collision_force_max_ > 0.0 && f_norm > collision_force_max_ && f_norm > 1e-9)
        f_collision.head<3>() *= collision_force_max_ / f_norm;
    f_collision = filterCollisionWrench(f_collision, dt_safe);

    const double collision_force_mag = f_collision.head<3>().norm();
    const double force_ratio = collision_force_mag / std::max(1e-6, i_force_ref_);
    const double attenuation =
        std::exp(-std::pow(std::max(0.0, force_ratio), i_force_shape_));
    const double i_scale =
        i_force_min_scale_ + (1.0 - i_force_min_scale_) * attenuation;
    const double i_gain_pos_eff = i_gain_pos_ * i_scale;
    const double i_gain_ori_eff = i_gain_ori_ * i_scale;
    if (i_collision_decay_rate_ > 0.0 && i_scale < 1.0)
    {
        const double leak =
            std::exp(-i_collision_decay_rate_ * (1.0 - i_scale) * dt_safe);
        e_p_int_ *= leak;
        e_o_int_ *= leak;
    }

    const bool collision_constraint_active =
        ((this->now() - t_last_collision_distance_cb_).seconds() <
         collision_constraint_timeout_sec_) &&
        ((this->now() - t_last_collision_normal_cb_).seconds() <
         collision_constraint_timeout_sec_) &&
        latest_collision_normal_.allFinite() &&
        latest_collision_normal_.norm() > 1e-6 &&
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

    if (collision_constraint_active && gamma > 0.0)
    {
        Eigen::Vector3d f_pub = -f_collision.head<3>();
        if (f_pub.allFinite() && n_away.allFinite() && n_away.norm() > 1e-9)
        {
            const double f_n = f_pub.dot(n_away);
            if (f_n > 0.0)
            {
                const Eigen::Vector3d f_normal = f_n * n_away;
                const Eigen::Vector3d f_tangent = f_pub - f_normal;
                f_pub = f_tangent + (1.0 - gamma) * f_normal;
            }
            else if (f_n < 0.0)
            {
                f_pub -= f_n * n_away;
            }
            f_collision.head<3>() = -f_pub;
        }
    }

    if (collision_constraint_active && gamma > 0.0)
    {
        const double vd_into = xdot_des.head<3>().dot(n_away);
        if (vd_into < 0.0)
            xdot_des.head<3>() -= gamma * vd_into * n_away;
    }

    const Eigen::Matrix<double, 6, 1> v_err = xdot_des - xdot_ref_;
    Eigen::Vector3d f_goal_pos =
        k_pos_var * e_p_ + i_gain_pos_eff * e_p_int_ + d_pos_var * v_err.head<3>();
    Eigen::Vector3d f_goal_ori =
        k_ori_var * e_o_ + i_gain_ori_eff * e_o_int_ + d_ori_var * v_err.tail<3>();

    if (collision_goal_suppression_ && gamma > 0.0)
    {
        const double f_into = f_goal_pos.dot(n_away);
        if (f_into < 0.0)
            f_goal_pos += -gamma * f_into * n_away;
    }

    Eigen::Matrix<double, 6, 1> xddot_ref;
    xddot_ref.head<3>() =
        (f_goal_pos - f_collision.head<3>()) / std::max(1e-9, m_pos_var);
    xddot_ref.tail<3>() =
        (f_goal_ori - f_collision.tail<3>()) / std::max(1e-9, m_ori_var);

    const Eigen::Matrix<double, 6, 1> xdot_prev = xdot_ref_;
    Eigen::Matrix<double, 6, 1> xdot_next = xdot_ref_ + xddot_ref * dt_safe;
    limitCartesianAcceleration(xdot_next, xdot_prev, dt_safe);
    xdot_ref_ = xdot_next;

    if (enable_collision_projection_ && gamma > 0.0)
    {
        const double v_into = xdot_ref_.head<3>().dot(n_away);
        if (v_into < 0.0)
            xdot_ref_.head<3>() -= gamma * v_into * n_away;
    }

    clampCartesianVelocity(xdot_ref_);

    theta_d = calcSrInverse(jacobian, manipulability, w0_, k0_) * xdot_ref_;
    if (!theta_d.allFinite())
        return false;

    double scale = 1.0;
    const int n_j = std::min<int>(NUMBER_OF_JOINT, static_cast<int>(theta_d.size()));
    for (int i = 0; i < n_j; ++i)
    {
        double lim = std::numeric_limits<double>::infinity();
        if (velocity_limits_.rows() == theta_d.size() && velocity_limits_.cols() >= 2)
            lim = std::max(std::abs(velocity_limits_(i, 0)),
                           std::abs(velocity_limits_(i, 1)));
        else if (theta_d.size() == NUMBER_OF_JOINT)
            lim = FALLBACK_VELOCITY[static_cast<size_t>(i)];
        lim *= SAFETY_VELOCITY_ALPHA;
        const double a = std::abs(theta_d[i]);
        if (std::isfinite(lim) && a > lim && a > 1e-12)
            scale = std::min(scale, lim / a);
    }
    if (scale < 1.0)
    {
        theta_d *= scale;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Joint velocity clamped with scale %.3f. Continuing tracking.", scale);
        const Eigen::VectorXd xdot_limited = jacobian * theta_d;
        if (xdot_limited.size() == 6 && xdot_limited.allFinite())
        {
            xdot_ref_ = xdot_limited;
            clampCartesianVelocity(xdot_ref_);
        }
        if (enable_collision_projection_ && collision_constraint_active && gamma > 0.0)
        {
            const double v_into = xdot_ref_.head<3>().dot(n_away);
            if (v_into < 0.0)
                xdot_ref_.head<3>() -= gamma * v_into * n_away;
        }
    }

    return true;
}

void MotoMiniPlanningNode::handleTrackingIdle()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        resetVirtualState();
    }

    if (last_joint_state_)
    {
        initTrackedPositions();
        if (is_active_)
        {
            std::vector<double> zero_vel(joint_names_.size(), 0.0);
            publishStreamingTrajectory(tracked_positions_, zero_vel);
        }
    }
}

void MotoMiniPlanningNode::handleTrackingStop()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        resetVirtualState();
    }

    if (last_joint_state_)
    {
        initTrackedPositions();
        if (is_active_)
        {
            std::vector<double> zero_vel(joint_names_.size(), 0.0);
            publishStreamingTrajectory(tracked_positions_, zero_vel);
        }
    }
}

void MotoMiniPlanningNode::handleTrackingArming()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        t_arming_start_ = this->now();
        arm_trigger_sent_ = false;
    }

    const double elapsed = (this->now() - t_arming_start_).seconds();

    if (!arm_trigger_sent_)
    {
        if (elapsed < ARM_PRE_DELAY_S)
            return;
        if (initTrackedPositions())
        {
            publishArmInit();
            arm_trigger_sent_ = true;
        }
        return;
    }

    if (elapsed < ARM_PRE_DELAY_S + ARM_POST_DELAY_S)
        return;

    if (initTrackedPositions())
    {
        seedStreamingCommand();
        resetControlWindow();
        initializeReferenceVelocityFromMeasuredState();
        transitionTrackingState(pending_tracking_state_, "arming complete");
    }
}

void MotoMiniPlanningNode::handleTrackingInit()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
    }

    if (!has_init_pose_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Tracking INIT waiting for /pose_following/init_pose.");
        return;
    }

    Eigen::VectorXd q;
    if (!currentJointVector(q))
        return;

    Eigen::Vector3d ee_pos;
    Eigen::Matrix3d ee_rot;
    if (!getEEPose(q, ee_pos, ee_rot))
        return;

    const double dx = std::abs(init_pose_.pose.position.x - ee_pos.x());
    const double dy = std::abs(init_pose_.pose.position.y - ee_pos.y());
    const double dz = std::abs(init_pose_.pose.position.z - ee_pos.z());
    if (dx < POSITION_ERROR_THRESHOLD &&
        dy < POSITION_ERROR_THRESHOLD &&
        dz < POSITION_ERROR_THRESHOLD)
    {
        is_init_done_ = true;
        resetControlWindow();
        initializeReferenceVelocityFromMeasuredState();
        transitionTrackingState(TrackingStreamState::POSE_FOLLOW, "init pose reached");
        return;
    }

    const Eigen::Quaterniond q_des = normalizedQuaternion(init_pose_.pose.orientation);
    const Eigen::Vector3d des_pos(init_pose_.pose.position.x,
                                  init_pose_.pose.position.y,
                                  init_pose_.pose.position.z);

    const double dt = (this->now() - t_last_).seconds();
    t_last_ = this->now();

    Eigen::VectorXd theta_d;
    if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt, theta_d))
        return;

    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
        !initTrackedPositions())
    {
        return;
    }

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

    if (!checkPositionLimits(tracked_positions_))
    {
        tracked_positions_ = prev_pos;
        std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
    }

    publishStreamingTrajectory(tracked_positions_, tracked_velocities_);
}

void MotoMiniPlanningNode::handleTrackingPoseFollow()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
    }

    if (!has_desired_pose_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Tracking POSE_FOLLOW waiting for /motomini/target_pose.");
        return;
    }

    const double pose_age = (this->now() - t_last_pose_cb_).seconds();
    if (pose_age > POSE_TIMEOUT_SEC)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Tracking pose input timeout %.2f s -- holding position.",
                             pose_age);
        if (last_joint_state_)
        {
            initTrackedPositions();
            std::vector<double> zero_vel(joint_names_.size(), 0.0);
            publishStreamingTrajectory(tracked_positions_, zero_vel);
        }
        return;
    }

    Eigen::VectorXd q;
    if (!currentJointVector(q))
        return;

    Eigen::Quaterniond q_des = normalizedQuaternion(desired_pose_.pose.orientation);
    Eigen::Vector3d des_pos(desired_pose_.pose.position.x,
                            desired_pose_.pose.position.y,
                            desired_pose_.pose.position.z);

    const double dt = (this->now() - t_last_).seconds();
    t_last_ = this->now();
    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));

    const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
    const bool vel_fresh = dt_vel < TARGET_VEL_TIMEOUT_SEC;
    const bool vel_active =
        vel_fresh &&
        (latest_target_vel_.head<3>().norm() > targetVelocityDeadband() ||
         latest_target_vel_.tail<3>().norm() > targetVelocityDeadband());
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

    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
        !initTrackedPositions())
    {
        return;
    }

    const std::vector<double> prev_pos = tracked_positions_;
    for (size_t i = 0; i < tracked_positions_.size(); ++i)
    {
        tracked_positions_[i] =
            q[static_cast<Eigen::Index>(i)] +
            theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
        tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
    }

    if (!checkPositionLimits(tracked_positions_))
    {
        tracked_positions_ = prev_pos;
        std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
    }

    publishStreamingTrajectory(tracked_positions_, tracked_velocities_);
}

void MotoMiniPlanningNode::feedbackTimerCallback()
{
    publishFeedback();

    if (!tracking_enabled_)
        return;

    switch (tracking_state_)
    {
    case TrackingStreamState::IDLE:
        handleTrackingIdle();
        break;
    case TrackingStreamState::ARMING:
        handleTrackingArming();
        break;
    case TrackingStreamState::INIT:
        handleTrackingInit();
        break;
    case TrackingStreamState::POSE_FOLLOW:
        handleTrackingPoseFollow();
        break;
    case TrackingStreamState::STOP:
        handleTrackingStop();
        break;
    }
}
