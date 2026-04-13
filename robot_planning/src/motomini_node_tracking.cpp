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
    // === GUARD: Check preconditions ===
    if (!last_joint_state_ || last_joint_state_->position.empty())
        return;
    if (!tracking_enabled_ || !tracking_pose_initialized_)
        return;

    const std::vector<std::string> joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u",
        "joint_4_r", "joint_5_b", "joint_6_t"};

    // ========================================================================
    // CRITICAL: Sync environment state with actual robot position (like run.cpp)
    // updateEnvironmentState() internally calls setState() to sync the planner's env
    // Filter positions to match manipulator joint_names (skip gantry joints)
    // ========================================================================
    {
        // Construct joint_pos by matching indices with joint_names
        Eigen::VectorXd joint_pos(static_cast<Eigen::Index>(joint_names.size()));

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            // Find the index of joint_names[i] in last_joint_state_->name
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(),
                                joint_names[i]);

            if (it != last_joint_state_->name.end())
            {
                size_t idx = std::distance(last_joint_state_->name.begin(), it);
                joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[idx];
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Tracking: Joint '%s' not found in joint_states",
                            joint_names[i].c_str());
                joint_pos[static_cast<Eigen::Index>(i)] = 0.0;
            }
        }

        planner_->updateEnvironmentState(joint_names, joint_pos);

        RCLCPP_DEBUG(this->get_logger(),
                     "Tracking: synced env state [%.4f, %.4f, %.4f]",
                     joint_pos[0], joint_pos[1], joint_pos[2]);
    }

    // === Get target tracking pose ===
    const Eigen::Isometry3d target = getLatestTipPose();

    // === CALL TRACKING PLANNER ===
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

    // === TRAJECTORY COMPLETION DETECTION (gating logic) ===
    bool should_publish = false;

    // First trajectory → always publish
    if (last_trajectory_end_state_.empty())
    {
        should_publish = true;
        RCLCPP_INFO(this->get_logger(), "Tracking: FIRST trajectory → publishing");
    }
    else
    {
        // Check if robot reached END of previous trajectory
        // CRITICAL: Extract ONLY manipulator joint positions (matching trajectory format)
        double max_end_error = 0.0;

        for (size_t i = 0; i < joint_names.size() && i < last_trajectory_end_state_.size(); ++i)
        {
            // Find current position of manipulator joint i
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(),
                                joint_names[i]);

            if (it != last_joint_state_->name.end())
            {
                size_t idx = std::distance(last_joint_state_->name.begin(), it);
                double current_pos = last_joint_state_->position[idx];
                double err = std::abs(last_trajectory_end_state_[i] - current_pos);
                max_end_error = std::max(max_end_error, err);
            }
        }

        const double COMPLETION_TOLERANCE = 0.15; // 8.6 degrees - tighter for smooth tracking
        const rclcpp::Time now = this->now();
        const double time_since_last = (now - last_tracking_publish_time_).seconds();
        const double MIN_SPACING = 0.3; // 300ms minimum between trajectories

        if (max_end_error < COMPLETION_TOLERANCE && time_since_last >= MIN_SPACING)
        {
            should_publish = true;
            RCLCPP_INFO(this->get_logger(),
                        "Tracking: COMPLETE (error: %.3f rad, gap: %.3f s) → publishing next",
                        max_end_error, time_since_last);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Tracking: waiting (error: %.3f rad < %.3f, gap: %.3f s < %.3f s)",
                         max_end_error, COMPLETION_TOLERANCE,
                         time_since_last, MIN_SPACING);
        }
    }

    // === ENFORCE COMPLETION GATE ===
    if (!should_publish)
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: gate closed - waiting for completion");
        return;
    }

    // === PUBLISH THE TRAJECTORY ===
    publishTrackingTrajectory(*traj_ptr, last_joint_state_->name);
    last_tracking_publish_time_ = this->now();
    last_published_target_ = target;

    // === CACHE END STATE FOR NEXT CHECK ===
    if (!last_joint_state_->name.empty() && traj_ptr->size() > 0)
    {
        const auto &last_point = traj_ptr->back();
        last_trajectory_end_state_ = std::vector<double>(
            last_point.position.data(),
            last_point.position.data() + last_point.position.size());
    }

    RCLCPP_INFO(this->get_logger(),
                "Tracking: trajectory published (%.3f s horizon)",
                traj_ptr->empty() ? 0.0 : traj_ptr->back().time);
}