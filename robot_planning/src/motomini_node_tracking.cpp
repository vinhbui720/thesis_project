/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — TF polling + predictive target + tracking tick.
 *
 * v4 additions:
 *   - EMA velocity estimation in tfPollLoop()
 *   - Predictive target (lead-compensated pose) fed to planner
 *   - tip_velocity passed through to runTrackingPlanner for Jacobian FF
 */

#include <robot_planning/motomini_planning_node.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <thread>

// ---------------------------------------------------------------------------
// TF polling thread — start / stop
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::startTfPolling()
{
    if (tf_poll_running_) return;
    tf_poll_running_ = true;
    tf_poll_thread_  = std::thread(&MotoMiniPlanningNode::tfPollLoop, this);
    RCLCPP_INFO(this->get_logger(), "TF poll thread started (%.0f Hz)", tf_poll_rate_hz_);
}

void MotoMiniPlanningNode::stopTfPolling()
{
    tf_poll_running_ = false;
    if (tf_poll_thread_.joinable()) tf_poll_thread_.join();
}

// ---------------------------------------------------------------------------
// Joint state polling thread — start / stop
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::startJointStatePolling()
{
    if (joint_state_poll_running_) return;
    joint_state_poll_running_ = true;
    joint_state_poll_thread_  = std::thread(&MotoMiniPlanningNode::jointStatePollLoop, this);
    RCLCPP_INFO(this->get_logger(), "Joint state poll thread started (%.0f Hz)",
                joint_state_poll_rate_hz_);
}

void MotoMiniPlanningNode::stopJointStatePolling()
{
    joint_state_poll_running_ = false;
    if (joint_state_poll_thread_.joinable()) joint_state_poll_thread_.join();
}

// ---------------------------------------------------------------------------
// Thread-safe getters
// ---------------------------------------------------------------------------
sensor_msgs::msg::JointState MotoMiniPlanningNode::getLatestJointState() const
{
    std::lock_guard<std::mutex> lock(joint_state_poll_mutex_);
    return latest_polled_joint_state_;
}

Eigen::Isometry3d MotoMiniPlanningNode::getLatestTipPose() const
{
    std::lock_guard<std::mutex> lock(tip_pose_mutex_);
    return latest_working_tip_world_;
}

// NEW — lead-compensated predicted pose
Eigen::Isometry3d MotoMiniPlanningNode::getLatestTipPosePredicted() const
{
    std::lock_guard<std::mutex> lock(tip_pose_mutex_);
    // Lead = one planner period + half a streamer period (IPC latency estimate)
    const double lead = plan_latency_ + exec_latency_;
    Eigen::Isometry3d predicted = latest_working_tip_world_;
    predicted.translation() += tip_velocity_ * lead;
    return predicted;
}

// NEW — smoothed Cartesian velocity of the gantry tip
Eigen::Vector3d MotoMiniPlanningNode::getLatestTipVelocity() const
{
    std::lock_guard<std::mutex> lock(tip_pose_mutex_);
    return tip_vel_initialized_ ? tip_velocity_ : Eigen::Vector3d::Zero();
}

// ---------------------------------------------------------------------------
// jointStatePollLoop
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::jointStatePollLoop()
{
    const auto period = std::chrono::milliseconds(
        static_cast<int64_t>(1000.0 / joint_state_poll_rate_hz_));

    while (joint_state_poll_running_ && rclcpp::ok())
    {
        if (last_joint_state_)
        {
            std::lock_guard<std::mutex> lock(joint_state_poll_mutex_);
            latest_polled_joint_state_ = *last_joint_state_;
            if (!joint_state_poll_initialized_)
                joint_state_poll_initialized_ = true;
        }
        std::this_thread::sleep_for(period);
    }
}

// ---------------------------------------------------------------------------
// tfPollLoop — position EMA + velocity EMA + predictive target
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::tfPollLoop()
{
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / tf_poll_rate_hz_));

    while (tf_poll_running_ && rclcpp::ok())
    {
        try
        {
            const auto world_to_base = tf_buffer_->lookupTransform(
                tracking_world_frame_, tracking_gantry_base_frame_, tf2::TimePointZero);
            const auto base_to_tip = tf_buffer_->lookupTransform(
                tracking_gantry_base_frame_, tracking_tip_frame_, tf2::TimePointZero);

            const Eigen::Isometry3d T_world_base = tf2::transformToEigen(world_to_base.transform);
            const Eigen::Isometry3d T_base_tip   = tf2::transformToEigen(base_to_tip.transform);
            const Eigen::Isometry3d measured      = T_world_base * T_base_tip;

            {
                std::lock_guard<std::mutex> lock(tip_pose_mutex_);

                if (!tracking_pose_initialized_)
                {
                    // Cold start — accept raw measurement directly
                    latest_working_tip_world_  = measured;
                    tip_prev_measured_         = measured;
                    tip_prev_time_             = this->now();
                    tracking_pose_initialized_ = true;
                    tip_vel_initialized_       = false;
                    tip_velocity_.setZero();
                }
                else
                {
                    // ── Position EMA (unchanged) ──────────────────────────────
                    const double alpha = tracking_ema_alpha_;
                    latest_working_tip_world_.translation() =
                        (1.0 - alpha) * latest_working_tip_world_.translation() +
                               alpha  * measured.translation();
                    latest_working_tip_world_.linear() = measured.linear();

                    // ── Velocity EMA ──────────────────────────────────────────
                    // Finite-difference on RAW measurement (not EMA filtered pos)
                    // so we get a responsive velocity estimate. The EMA below
                    // provides noise smoothing on top.
                    const double dt_tf =
                        (this->now() - tip_prev_time_).seconds();

                    if (dt_tf > 1e-4 && dt_tf < 0.05)  // skip stale / bogus TF
                    {
                        const Eigen::Vector3d raw_vel =
                            (measured.translation() -
                             tip_prev_measured_.translation()) / dt_tf;

                        if (!tip_vel_initialized_)
                        {
                            tip_velocity_       = raw_vel;
                            tip_vel_initialized_ = true;
                        }
                        else
                        {
                            // Lower alpha than position — derivative is noisier
                            tip_velocity_ = tf_vel_alpha_ * raw_vel
                                          + (1.0 - tf_vel_alpha_) * tip_velocity_;
                        }
                    }

                    tip_prev_measured_ = measured;
                    tip_prev_time_     = this->now();
                }
            }

            // Debug publish
            if (pub_tracked_pose_)
            {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.stamp    = this->now();
                ps.header.frame_id = tracking_world_frame_;
                ps.pose            = tf2::toMsg(measured);
                pub_tracked_pose_->publish(ps);
            }
        }
        catch (const tf2::TransformException &) {}  // TF not ready — retry

        std::this_thread::sleep_for(period);
    }
}

// ---------------------------------------------------------------------------
// trackingTick — planning tick at tracking_rate_hz_
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingTick()
{
    RCLCPP_DEBUG(this->get_logger(), "[trackingTick] entered.");

    if (!last_joint_state_ || last_joint_state_->position.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "[trackingTick] No joint state — skipping.");
        return;
    }
    if (!tracking_enabled_ || !tracking_pose_initialized_)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "[trackingTick] Guard: enabled=%d pose_init=%d — skipping.",
                     (int)tracking_enabled_, (int)tracking_pose_initialized_);
        return;
    }

    const std::vector<std::string> joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u",
        "joint_4_r", "joint_5_b", "joint_6_t"};

    // ── Predicted target (lead-compensated) ───────────────────────────────
    const Eigen::Isometry3d target   = getLatestTipPosePredicted();  // ← key change
    const Eigen::Vector3d   tip_vel  = getLatestTipVelocity();       // ← for Jac FF

    // ── HW joint velocities (cold-start fallback only) ────────────────────
    Eigen::VectorXd hw_vel;
    {
        const auto js = getLatestJointState();
        if (js.velocity.size() == joint_names.size())
        {
            hw_vel.resize(static_cast<Eigen::Index>(joint_names.size()));
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                auto it = std::find(js.name.begin(), js.name.end(), joint_names[i]);
                hw_vel[static_cast<Eigen::Index>(i)] =
                    (it != js.name.end())
                        ? js.velocity[static_cast<size_t>(
                              std::distance(js.name.begin(), it))]
                        : 0.0;
            }
        }
    }

    // ── Call planner ──────────────────────────────────────────────────────
    if (!planner_->runTrackingPlanner(target, hw_vel, tip_vel))
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: planning failed — skipping.");
        return;
    }

    auto traj_ptr = planner_->getTrajectory();
    if (!traj_ptr || traj_ptr->empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: empty trajectory — skipping.");
        return;
    }

    const std::vector<std::string> traj_joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u",
        "joint_4_r", "joint_5_b", "joint_6_t"};

    publishTrackingTrajectory(*traj_ptr, traj_joint_names);

    RCLCPP_INFO(this->get_logger(),
                "Tracking: sent %.3f s traj (%zu pts), tip_vel=%.3f m/s",
                traj_ptr->back().time, traj_ptr->size(), tip_vel.norm());
}

// ---------------------------------------------------------------------------
// trackingStreamTick — no-op (handled by motomini_traj_streamer)
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingStreamTick() {}