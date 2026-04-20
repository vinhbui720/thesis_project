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
    RCLCPP_DEBUG(this->get_logger(), "[trackingTick] entered.");

    // === GUARD: Check preconditions ===
    if (!last_joint_state_ || last_joint_state_->position.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "[trackingTick] No joint state yet — skipping.");
        return;
    }
    if (!tracking_enabled_ || !tracking_pose_initialized_)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "[trackingTick] Guard: enabled=%d pose_init=%d — skipping.",
                     (int)tracking_enabled_, (int)tracking_pose_initialized_);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "[trackingTick] Guards passed.");

    // NOTE: DO NOT call updateEnvironmentState() here.
    // The Tesseract environment is continuously updated by the ROSEnvironmentMonitor
    // (via startStateMonitor('/joint_states')). Calling setState() from this thread
    // while the monitor's own update thread is also writing to env_ creates a
    // data race on Tesseract's internal unordered_map, corrupting its memory
    // layout → SIGSEGV -11. The planner reads env_ under its own shared_lock,
    // which is sufficient since the monitor uses a write lock.

    const std::vector<std::string> joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u",
        "joint_4_r", "joint_5_b", "joint_6_t"};

    // === Get target tracking pose ===
    const Eigen::Isometry3d target = getLatestTipPose();

    // === CALL TRACKING PLANNER ===
    if (!planner_->runTrackingPlanner(target))
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: planning failed — skipping");
        return;
    }

    auto traj_ptr = planner_->getTrajectory();
    if (!traj_ptr || traj_ptr->empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Tracking: empty trajectory — skipping");
        return;
    }

    // === PUBLISH FULL TRAJECTORY DIRECTLY → traj_streamer splices it ===
    // The traj_streamer finds the nearest point to the current robot position
    // and smooth-merges into the new trajectory, so there is no need for a
    // completion gate here.  Just send every new plan immediately.
    const std::vector<std::string> traj_joint_names = {
        "joint_1_s", "joint_2_l", "joint_3_u",
        "joint_4_r", "joint_5_b", "joint_6_t"};

    publishTrackingTrajectory(*traj_ptr, traj_joint_names);

    RCLCPP_INFO(this->get_logger(),
                "Tracking: sent %.3f s trajectory (%zu pts) to traj_streamer",
                traj_ptr->back().time, traj_ptr->size());
}

// ---------------------------------------------------------------------------
// trackingStreamTick — 50Hz continuous interpolation and streaming
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingStreamTick()
{
    // No-op: trajectory streaming is now handled entirely by motomini_traj_streamer.
    // The traj_streamer receives the full Ruckig-smoothed trajectory on /joint_path_command,
    // splices it from the nearest point to the current robot state, and publishes
    // interpolated single-point commands to /joint_command at 50 Hz.
}