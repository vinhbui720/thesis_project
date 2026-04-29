/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — embedded feedback-stream tracking controller.
 *
 * This adapts the control structure from motomini_feedback_stream.cpp into the
 * planning node. The standalone feedback-stream file remains unchanged.
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
constexpr double SAFETY_VELOCITY_ALPHA = 0.65;
constexpr double SAFETY_JOINT_PADDING_RAD = 5.0 * M_PI / 180.0;
constexpr double POSITION_ERROR_THRESHOLD = 0.0005;
constexpr double POSE_TIMEOUT_SEC = 3.0;
constexpr double TARGET_VEL_TIMEOUT_SEC = 0.5;

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

    const rclcpp::Time now = this->now();
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(q.size());
    bool velocity_obtained = false;

    if (real_robot_)
    {
        if (last_joint_state_ &&
            last_joint_state_->velocity.size() >= last_joint_state_->name.size())
        {
            Eigen::VectorXd qdot_driver(static_cast<Eigen::Index>(joint_names_.size()));
            bool valid = true;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto it = std::find(last_joint_state_->name.begin(),
                                    last_joint_state_->name.end(),
                                    joint_names_[i]);
                if (it == last_joint_state_->name.end())
                {
                    valid = false;
                    break;
                }
                const size_t idx =
                    static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
                if (idx >= last_joint_state_->velocity.size())
                {
                    valid = false;
                    break;
                }
                qdot_driver[static_cast<Eigen::Index>(i)] = last_joint_state_->velocity[idx];
            }
            if (valid && qdot_driver.allFinite())
            {
                qdot = qdot_driver;
                velocity_obtained = true;
            }
        }
    }

    if (!velocity_obtained && have_q_prev_ && q_prev_.size() == q.size())
    {
        const double dt = (now - t_prev_q_).seconds();
        if (dt > 1e-6)
            qdot = (q - q_prev_) / dt;
    }
    q_prev_ = q;
    t_prev_q_ = now;
    have_q_prev_ = true;

    if (pub_feedback_vel_)
    {
        const Eigen::MatrixXd jacobian = manip_->calcJacobian(q, base_link_, ee_link_);
        const Eigen::VectorXd v_cart = jacobian * qdot;

        geometry_msgs::msg::Twist msg;
        msg.linear.x = v_cart(0);
        msg.linear.y = v_cart(1);
        msg.linear.z = v_cart(2);
        msg.angular.x = v_cart(3);
        msg.angular.y = v_cart(4);
        msg.angular.z = v_cart(5);
        pub_feedback_vel_->publish(msg);
    }
}

bool MotoMiniPlanningNode::checkVelocityLimits(const Eigen::VectorXd &theta_d) const
{
    if (theta_d.size() == 0)
        return false;

    for (Eigen::Index i = 0; i < theta_d.size(); ++i)
    {
        double limit = std::numeric_limits<double>::infinity();
        if (velocity_limits_.rows() == theta_d.size() && velocity_limits_.cols() >= 2)
        {
            limit = std::max(std::abs(velocity_limits_(i, 0)),
                             std::abs(velocity_limits_(i, 1)));
        }
        else if (theta_d.size() == NUMBER_OF_JOINT)
        {
            limit = FALLBACK_VELOCITY[static_cast<size_t>(i)];
        }

        limit *= SAFETY_VELOCITY_ALPHA;
        if (std::isfinite(limit) && std::abs(theta_d[i]) > limit)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Joint %ld velocity %.4f rad/s exceeds streaming limit %.4f rad/s",
                        static_cast<long>(i), theta_d[i], limit);
            return false;
        }
    }
    return true;
}

bool MotoMiniPlanningNode::checkPositionLimits(
    const std::vector<double> &pos,
    const std::vector<double> &reference) const
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
            if (reference.size() == pos.size())
            {
                const double eps = 1e-9;
                const bool recovering_from_lower =
                    below && reference[i] <= lower && pos[i] >= reference[i] - eps;
                const bool recovering_from_upper =
                    above && reference[i] >= upper && pos[i] <= reference[i] + eps;
                if (recovering_from_lower || recovering_from_upper)
                    continue;
            }

            RCLCPP_WARN(this->get_logger(),
                        "Joint %zu position %.4f rad outside streaming safe range [%.4f, %.4f]",
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
    RCLCPP_INFO(this->get_logger(), "Tracking seed sent to joint_command.");
}

void MotoMiniPlanningNode::publishStreamingTrajectory(const std::vector<double> &pos,
                                                      const std::vector<double> &vel)
{
    const double t_rel = (this->now() - t_start_).seconds();
    publishStreamPoint(pub_stream_joint_cmd_, pos, vel, t_rel);
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

void MotoMiniPlanningNode::resetVirtualState()
{
    xdot_ref_.setZero();
    have_q_prev_ctrl_ = false;
    q_prev_ctrl_.resize(0);
    latest_collision_wrench_.setZero();
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

void MotoMiniPlanningNode::ensureStreamingInitialized()
{
    if (!tracking_enabled_ || stream_arm_init_sent_)
        return;

    if (initTrackedPositions())
    {
        publishArmInit();
        stream_arm_init_sent_ = true;
    }
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
        resetControlWindow();
        if (enable_seed_)
            seedStreamingCommand();
        tracking_state_ = TrackingStreamState::POSE_FOLLOW;
        RCLCPP_INFO(this->get_logger(),
                    "Tracking stream: IDLE -> POSE_FOLLOW from target velocity.");
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
        last_tracking_state_ = TrackingStreamState::STOP;
        stream_arm_init_sent_ = false;
        is_init_done_ = false;
        has_desired_pose_ = false;
        has_init_pose_ = false;
        latest_target_vel_.setZero();
        t_last_target_vel_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        resetControlWindow();
        ensureStreamingInitialized();

        publishStatus("Mode: Tracking");
        RCLCPP_INFO(this->get_logger(),
                    "Mode -> TRACKING feedback stream. Waiting for fresh tracking input.");
    }
    else
    {
        if (!tracking_enabled_)
            return;

        tracking_enabled_ = false;
        tracking_state_ = TrackingStreamState::IDLE;
        last_tracking_state_ = TrackingStreamState::IDLE;
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
            resetControlWindow();
            if (enable_seed_)
                seedStreamingCommand();
            tracking_state_ = TrackingStreamState::POSE_FOLLOW;
            RCLCPP_INFO(this->get_logger(),
                        "Tracking stream: IDLE -> POSE_FOLLOW from /motomini/target_pose.");
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
        resetControlWindow();
        if (enable_seed_)
            seedStreamingCommand();
        tracking_state_ = TrackingStreamState::INIT;
    }
}

void MotoMiniPlanningNode::targetVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!msg)
        return;

    if (!tracking_enabled_)
        return;

    latest_target_vel_ << msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z;
    t_last_target_vel_cb_ = this->now();

    if (tracking_state_ == TrackingStreamState::IDLE &&
        latest_target_vel_.norm() > 0.0)
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

    const double active_time = (this->now() - t_start_).seconds();
    const double s_t = 1.0 - std::exp(-adaptive_lambda_ * std::max(0.0, active_time));
    const double s_e_pos = std::tanh(adaptive_alpha_pos_ * e_p_.norm());
    const double s_e_ori = std::tanh(adaptive_alpha_ori_ * e_o_.norm());

    const Eigen::MatrixXd jacobian = manip_->calcJacobian(q, base_link_, ee_link_);
    const double determinant = (jacobian * jacobian.transpose()).determinant();
    const double manipulability = std::sqrt(std::max(0.0, determinant));
    const double s_w = std::clamp(manipulability / std::max(1e-9, w0_), 0.2, 1.0);
    if (manipulability <= w0_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Near singularity (w=%.6f <= w0=%.6f). SR damping active.",
                             manipulability, w0_);
    }

    const double k_pos = s_w * (k_pos_min_ + s_t * s_e_pos * (k_pos_max_ - k_pos_min_));
    const double m_pos = m_pos_max_ - s_t * s_e_pos * (m_pos_max_ - m_pos_min_);
    const double d_pos = 2.0 * zeta_pos_ * std::sqrt(std::max(1e-12, m_pos * k_pos));

    const double k_ori = s_w * (k_ori_min_ + s_t * s_e_ori * (k_ori_max_ - k_ori_min_));
    const double m_ori = m_ori_max_ - s_t * s_e_ori * (m_ori_max_ - m_ori_min_);
    const double d_ori = 2.0 * zeta_ori_ * std::sqrt(std::max(1e-12, m_ori * k_ori));

    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(q.size());
    bool velocity_obtained = false;

    if (real_robot_)
    {
        if (last_joint_state_ &&
            last_joint_state_->velocity.size() >= last_joint_state_->name.size())
        {
            Eigen::VectorXd qdot_driver(static_cast<Eigen::Index>(joint_names_.size()));
            bool valid = true;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto it = std::find(last_joint_state_->name.begin(),
                                    last_joint_state_->name.end(),
                                    joint_names_[i]);
                if (it == last_joint_state_->name.end())
                {
                    valid = false;
                    break;
                }
                const size_t idx =
                    static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
                if (idx >= last_joint_state_->velocity.size())
                {
                    valid = false;
                    break;
                }
                qdot_driver[static_cast<Eigen::Index>(i)] = last_joint_state_->velocity[idx];
            }
            if (valid && qdot_driver.allFinite())
            {
                qdot = qdot_driver;
                velocity_obtained = true;
            }
        }
    }

    if (!velocity_obtained && have_q_prev_ctrl_ && q_prev_ctrl_.size() == q.size())
    {
        const double dt_q = (this->now() - t_prev_q_ctrl_).seconds();
        if (dt_q > 1e-6)
            qdot = (q - q_prev_ctrl_) / dt_q;
    }
    q_prev_ctrl_ = q;
    t_prev_q_ctrl_ = this->now();
    have_q_prev_ctrl_ = true;

    const Eigen::Matrix<double, 6, 1> xdot_actual = jacobian * qdot;

    Eigen::Matrix<double, 6, 1> xdot_des =
        Eigen::Matrix<double, 6, 1>::Zero();
    if ((this->now() - t_last_target_vel_cb_).seconds() < TARGET_VEL_TIMEOUT_SEC)
    {
        xdot_des.head<3>() = latest_target_vel_.head<3>();
        xdot_des.tail<3>() = des_rot * latest_target_vel_.tail<3>();
    }
    const Eigen::Matrix<double, 6, 1> velocity_error = xdot_actual - xdot_des;

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

    Eigen::Vector3d f_goal_pos = k_pos * e_p_ - d_pos * velocity_error.head<3>();
    if (collision_goal_suppression_ && gamma > 0.0)
    {
        const double goal_into = f_goal_pos.dot(n_away);
        if (goal_into < 0.0)
            f_goal_pos -= gamma * goal_into * n_away;
    }

    Eigen::Matrix<double, 6, 1> xddot_ref;
    xddot_ref.head<3>() =
        (f_goal_pos - f_collision.head<3>() - d_pos * xdot_ref_.head<3>()) /
        std::max(1e-9, m_pos);
    xddot_ref.tail<3>() =
        (k_ori * e_o_ - d_ori * velocity_error.tail<3>() -
         f_collision.tail<3>() - d_ori * xdot_ref_.tail<3>()) /
        std::max(1e-9, m_ori);

    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    xdot_ref_ += xddot_ref * dt_safe;

    if (enable_collision_projection_ && gamma > 0.0)
    {
        const double v_into = xdot_ref_.head<3>().dot(n_away);
        if (v_into < 0.0)
            xdot_ref_.head<3>() -= gamma * v_into * n_away;
    }

    const double linear_norm = xdot_ref_.head<3>().norm();
    if (linear_norm > max_cart_linear_vel_ && linear_norm > 1e-9)
        xdot_ref_.head<3>() *= max_cart_linear_vel_ / linear_norm;

    const double angular_norm = xdot_ref_.tail<3>().norm();
    if (angular_norm > max_cart_angular_vel_ && angular_norm > 1e-9)
        xdot_ref_.tail<3>() *= max_cart_angular_vel_ / angular_norm;

    theta_d = calcSrInverse(jacobian, manipulability, w0_, k0_) * xdot_ref_;
    return theta_d.allFinite();
}

void MotoMiniPlanningNode::handleTrackingIdle()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        resetVirtualState();
        RCLCPP_INFO(this->get_logger(),
                    "Tracking stream IDLE: syncing joint state, no command output.");
    }

    if (last_joint_state_)
        initTrackedPositions();
}

void MotoMiniPlanningNode::handleTrackingStop()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        resetVirtualState();
        RCLCPP_INFO(this->get_logger(),
                    "Tracking stream STOP: command output disabled.");
    }

    if (last_joint_state_)
        initTrackedPositions();
}

void MotoMiniPlanningNode::handleTrackingInit()
{
    if (last_tracking_state_ != tracking_state_)
    {
        last_tracking_state_ = tracking_state_;
        RCLCPP_INFO(this->get_logger(),
                    "Tracking stream INIT: moving to /pose_following/init_pose.");
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
        tracking_state_ = TrackingStreamState::POSE_FOLLOW;
        RCLCPP_INFO(this->get_logger(),
                    "Tracking init complete. INIT -> POSE_FOLLOW.");
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

    if (!checkVelocityLimits(theta_d))
    {
        resetControlWindow();
        tracking_state_ = TrackingStreamState::STOP;
        return;
    }

    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
        !initTrackedPositions())
    {
        return;
    }

    std::vector<double> prev_pos = tracked_positions_;
    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    for (size_t i = 0; i < tracked_positions_.size(); ++i)
    {
        tracked_positions_[i] += theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
        tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
    }

    if (!checkPositionLimits(tracked_positions_, prev_pos))
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
        resetVirtualState();
        RCLCPP_INFO(this->get_logger(),
                    "Tracking stream POSE_FOLLOW: tracking /motomini/target_pose.");
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
        RCLCPP_WARN(this->get_logger(),
                    "Tracking pose input timeout %.2f s. POSE_FOLLOW -> IDLE.",
                    pose_age);
        resetControlWindow();
        tracking_state_ = TrackingStreamState::IDLE;
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

    if ((this->now() - t_last_target_vel_cb_).seconds() < TARGET_VEL_TIMEOUT_SEC &&
        latest_target_vel_.norm() > 0.0)
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

    if (!checkVelocityLimits(theta_d))
    {
        resetControlWindow();
        tracking_state_ = TrackingStreamState::STOP;
        return;
    }

    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) &&
        !initTrackedPositions())
    {
        return;
    }

    std::vector<double> prev_pos = tracked_positions_;
    for (size_t i = 0; i < tracked_positions_.size(); ++i)
    {
        tracked_positions_[i] += theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
        tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
    }

    if (!checkPositionLimits(tracked_positions_, prev_pos))
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

    ensureStreamingInitialized();
    switch (tracking_state_)
    {
    case TrackingStreamState::IDLE:
        handleTrackingIdle();
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
