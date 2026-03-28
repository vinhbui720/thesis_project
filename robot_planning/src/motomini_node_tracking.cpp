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
// getLatestTipPose — thread-safe read of cached pose
// ---------------------------------------------------------------------------
Eigen::Isometry3d MotoMiniPlanningNode::getLatestTipPose() const
{
    std::lock_guard<std::mutex> lock(tip_pose_mutex_);
    return latest_working_tip_world_;
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
// trackingTick — called by the tracking timer
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingTick()
{
    // --- Sync planner env with latest joint states ---
    if (last_joint_state_)
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

    const std::vector<std::string> joint_names =
        last_joint_state_ ? last_joint_state_->name : std::vector<std::string>{};

    publishTrackingTrajectory(*traj_ptr, joint_names);
    RCLCPP_DEBUG(this->get_logger(), "Tracking: new trajectory published");
}
