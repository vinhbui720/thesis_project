/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — high-frequency TF polling thread + planning tick.
 *
 * Architecture:
 *   tfPollLoop()   — runs in a dedicated thread at ~200 Hz, continuously caches
 *                    the working_tip pose from TF.  This decouples TF freshness
 *                    from the (slower) planning tick rate.
 *   trackingTick() — called by the tracking timer (default 30 Hz).  Reads the
 *                    cached pose (lock-free fast), syncs env, calls planner.
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
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
    if (tf_poll_running_)
        return;
    tf_poll_running_ = true;
    tf_poll_thread_ = std::thread(&MotoMiniPlanningNode::tfPollLoop, this);
    RCLCPP_INFO(this->get_logger(), "TF poll thread started (%.0f Hz)", tf_poll_rate_hz_);
}

void MotoMiniPlanningNode::stopTfPolling()
{
    tf_poll_running_ = false;
    if (tf_poll_thread_.joinable())
        tf_poll_thread_.join();
}

// ---------------------------------------------------------------------------
// Joint state polling thread — start / stop
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::startJointStatePolling()
{
    if (joint_state_poll_running_)
        return;
    joint_state_poll_running_ = true;
    joint_state_poll_thread_ = std::thread(&MotoMiniPlanningNode::jointStatePollLoop, this);
    RCLCPP_INFO(this->get_logger(), "Joint state poll thread started (%.0f Hz)", joint_state_poll_rate_hz_);
}

void MotoMiniPlanningNode::stopJointStatePolling()
{
    joint_state_poll_running_ = false;
    if (joint_state_poll_thread_.joinable())
        joint_state_poll_thread_.join();
}

// ---------------------------------------------------------------------------
// getLatestJointState — thread-safe read of cached joint state
// ---------------------------------------------------------------------------
sensor_msgs::msg::JointState MotoMiniPlanningNode::getLatestJointState() const
{
    std::lock_guard<std::mutex> lock(joint_state_poll_mutex_);
    return latest_polled_joint_state_;
}

// ---------------------------------------------------------------------------
// getLatestTipPose — thread-safe read of cached pose
// ---------------------------------------------------------------------------
Eigen::Isometry3d MotoMiniPlanningNode::getLatestTipPose() const
{
    std::lock_guard<std::mutex> lock(tip_pose_mutex_);
    return latest_working_tip_world_;
}

// ---------------------------------------------------------------------------
// jointStatePollLoop — runs in its own thread, continuously caches joint state
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::jointStatePollLoop()
{
    const auto period = std::chrono::milliseconds(
        static_cast<int64_t>(1000.0 / joint_state_poll_rate_hz_));

    while (joint_state_poll_running_ && rclcpp::ok())
    {
        // Read the latest joint state from callback cache
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
// tfPollLoop — runs in its own thread, continuously caches the working_tip pose
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
            const Eigen::Isometry3d T_base_tip = tf2::transformToEigen(base_to_tip.transform);
            const Eigen::Isometry3d measured = T_world_base * T_base_tip;

            {
                std::lock_guard<std::mutex> lock(tip_pose_mutex_);
                if (!tracking_pose_initialized_)
                {
                    latest_working_tip_world_ = measured;
                    tracking_pose_initialized_ = true;
                }
                else
                {
                    // EMA low-pass filter — alpha closer to 1.0 = faster response
                    const double alpha = tracking_ema_alpha_;
                    latest_working_tip_world_.translation() =
                        (1.0 - alpha) * latest_working_tip_world_.translation() +
                        alpha * measured.translation();
                    latest_working_tip_world_.linear() = measured.linear();
                }
            }

            // Debug: publish the cached pose as PoseStamped
            if (pub_tracked_pose_)
            {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.stamp = this->now();
                ps.header.frame_id = tracking_world_frame_;
                ps.pose = tf2::toMsg(measured);
                pub_tracked_pose_->publish(ps);
            }
        }
        catch (const tf2::TransformException &)
        {
            // TF not ready yet — silently retry next iteration
        }

        std::this_thread::sleep_for(period);
    }
}

// ---------------------------------------------------------------------------
// trackingTick — called by the tracking timer (optimized continuous tracking)
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingTick()
{
    // --- Use last_joint_state_ directly (available from callback) ---
    if (!last_joint_state_ || last_joint_state_->position.empty())
        return;

    // --- Sync planner env with latest joint states ---
    {
        const std::vector<std::string> &names = last_joint_state_->name;
        Eigen::VectorXd joint_pos(static_cast<Eigen::Index>(last_joint_state_->position.size()));
        for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
            joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[i];
        planner_->updateEnvironmentState(names, joint_pos);
    }

    // --- Tracking DISABLED: yield to planning mode, do nothing ---
    if (!tracking_enabled_)
        return;

    // --- Tracking ENABLED: follow cached working_tip pose ---
    if (!tracking_pose_initialized_)
        return;

    const Eigen::Isometry3d target = getLatestTipPose();

    if (!planner_->runTrackingPlanner(target))
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: planning failed");
        return;
    }

    auto traj_ptr = planner_->getTrajectory();
    if (!traj_ptr || traj_ptr->empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: empty trajectory");
        return;
    }

    // === BASIC TRAJECTORY SANITY CHECKS ===
    // Verify trajectory has valid data
    if (traj_ptr->front().position.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Tracking: trajectory has empty position data");
        return;
    }

    // === PREDICTIVE START POSITION VALIDATION ===
    // For continuous tracking on moving platforms (e.g., gantry), use lenient validation:
    // - Accept trajectories if start is reasonably close to current position within RELAXED tolerance
    // - This prevents rejection of valid trajectories due to platform motion between planning & execution

    const size_t traj_dof = traj_ptr->front().position.size();
    const size_t current_dof = last_joint_state_->position.size();

    // Validate DOF compatibility (trajectory should not exceed available joints)
    if (traj_dof > current_dof)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Tracking: trajectory DOF (%zu) exceeds available joints (%zu), skipping",
                    traj_dof, current_dof);
        return;
    }

    // Compute trajectory start validation error with RELAXED tolerance for continuous motion
    double max_start_error = 0.0;
    for (size_t i = 0; i < traj_dof; ++i)
    {
        double start_error = std::abs(traj_ptr->front().position[i] - last_joint_state_->position[i]);
        max_start_error = std::max(max_start_error, start_error);
    }

    // RELAXED tolerance: 0.35 rad (~20 degrees) for continuous gantry tracking
    // This accounts for motion between planning cycle and controller execution
    // The controller will perform its own stricter validation
    const double CONTINUOUS_TRACKING_TOLERANCE = 0.35;

    if (max_start_error > CONTINUOUS_TRACKING_TOLERANCE)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "Tracking: trajectory start error %.4f rad exceeds tolerance %.4f rad (gantry moving), "
                     "publishing anyway for continuous motion",
                     max_start_error, CONTINUOUS_TRACKING_TOLERANCE);
        // Note: We still publish to maintain continuous motion. The controller will validate stricter constraints.
        // Silently skipping here causes jerky motion (run-stop-run pattern).
        // Better to publish and let control layer handle edge cases.
    }

    // === TRAJECTORY COMPLETION DETECTION ===
    // STRICT continuous tracking: Only publish when PREVIOUS trajectory is complete.
    // DO NOT publish on target movement (causes splicing errors).
    // This ensures robot finishes each segment before next is sent.

    bool should_publish = false;

    // First trajectory → always publish
    if (last_trajectory_end_state_.empty())
    {
        should_publish = true;
        RCLCPP_DEBUG(this->get_logger(), "Tracking: FIRST trajectory, publishing");
    }
    else
    {
        // === STRICT COMPLETION CHECK ===
        // Wait for robot to reach END of previous trajectory before sending next
        double max_end_error = 0.0;
        const size_t end_state_size = last_trajectory_end_state_.size();

        for (size_t i = 0; i < end_state_size && i < last_joint_state_->position.size(); ++i)
        {
            double end_error = std::abs(last_trajectory_end_state_[i] - last_joint_state_->position[i]);
            max_end_error = std::max(max_end_error, end_error);
        }

        // STRICT: 0.30 rad (~17 deg) - robot must be very close to end before next sends
        const double TRAJECTORY_COMPLETION_TOLERANCE = 0.30;

        // Also enforce MINIMUM TIME between publishes to prevent rapid splicing
        const rclcpp::Time now = this->now();
        const double time_since_last = (now - last_tracking_publish_time_).seconds();
        const double MIN_TRAJECTORY_SPACING = 0.5; // At least 500ms between trajectories

        if (max_end_error < TRAJECTORY_COMPLETION_TOLERANCE)
        {
            // Trajectory is complete, but also check minimum spacing
            if (time_since_last >= MIN_TRAJECTORY_SPACING)
            {
                should_publish = true;
                RCLCPP_INFO(this->get_logger(),
                            "Tracking: COMPLETE (error: %.3f rad, gap: %.3f s) → publishing next",
                            max_end_error, time_since_last);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "Tracking: complete but too soon (error: %.3f rad, only %.3f s since last)",
                             max_end_error, time_since_last);
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Tracking: NOT complete yet (error: %.3f rad, threshold: %.3f rad)",
                         max_end_error, TRAJECTORY_COMPLETION_TOLERANCE);
        }
    }

    if (!should_publish)
        return;

    // === PUBLISH THE TRAJECTORY ===
    publishTrackingTrajectory(*traj_ptr, last_joint_state_->name);
    last_tracking_publish_time_ = this->now();
    last_published_target_ = target;

    // === CACHE END STATE FOR NEXT COMPLETION CHECK ===
    if (!last_joint_state_->name.empty() && traj_ptr->size() > 0)
    {
        const auto &last_point = traj_ptr->back();
        last_trajectory_end_state_ = std::vector<double>(last_point.position.data(),
                                                         last_point.position.data() + last_point.position.size());
        RCLCPP_DEBUG(this->get_logger(), "Tracking: cached end state, next check at: [%.4f, %.4f, %.4f, ...]",
                     last_trajectory_end_state_.size() > 0 ? last_trajectory_end_state_[0] : 0.0,
                     last_trajectory_end_state_.size() > 1 ? last_trajectory_end_state_[1] : 0.0,
                     last_trajectory_end_state_.size() > 2 ? last_trajectory_end_state_[2] : 0.0);
    }

    RCLCPP_DEBUG(this->get_logger(), "Tracking: trajectory published (%.3f s horizon)",
                 traj_ptr->empty() ? 0.0 : traj_ptr->back().time);
}
