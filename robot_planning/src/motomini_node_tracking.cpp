/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — TF-based working-tip pose update and tracking tick.
 *
 * trackingTick() is called by the tracking timer (default 5 Hz).
 * When tracking is ENABLED  → looks up working_tip in world frame, filters
 *                             translation with an EMA, calls runTrackingPlanner().
 * When tracking is DISABLED → commands the robot back to its initial pose.
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

// ---------------------------------------------------------------------------
// updateWorkingTipPoseFromTfAndJoints
// ---------------------------------------------------------------------------
bool MotoMiniPlanningNode::updateWorkingTipPoseFromTfAndJoints()
{
    if (!last_joint_state_)
        return false;

    // Keep planner environment in sync with real joint values
    const std::vector<std::string> &names = last_joint_state_->name;
    Eigen::VectorXd joint_pos(static_cast<Eigen::Index>(last_joint_state_->position.size()));
    for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
        joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[i];
    planner_->updateEnvironmentState(names, joint_pos);

    try
    {
        const auto world_to_base = tf_buffer_->lookupTransform(
            tracking_world_frame_, tracking_gantry_base_frame_, tf2::TimePointZero);
        const auto base_to_tip = tf_buffer_->lookupTransform(
            tracking_gantry_base_frame_, tracking_tip_frame_, tf2::TimePointZero);

        const Eigen::Isometry3d T_world_base = tf2::transformToEigen(world_to_base.transform);
        const Eigen::Isometry3d T_base_tip = tf2::transformToEigen(base_to_tip.transform);
        const Eigen::Isometry3d measured = T_world_base * T_base_tip;

        if (!tracking_pose_initialized_)
        {
            latest_working_tip_world_ = measured;
            tracking_pose_initialized_ = true;
        }
        else
        {
            // EMA low-pass filter on translation — suppresses ICP / TF noise.
            // Decrease alpha (toward 0) for more smoothing; increase (toward 1) for faster response.
            constexpr double alpha = 0.25;
            latest_working_tip_world_.translation() =
                (1.0 - alpha) * latest_working_tip_world_.translation() +
                alpha * measured.translation();
            latest_working_tip_world_.linear() = measured.linear();
        }
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Tracking TF lookup failed: %s", ex.what());
        return false;
    }
}

// ---------------------------------------------------------------------------
// trackingTick
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::trackingTick()
{
    if (!tracking_mode_)
        return;

    // --- Tracking DISABLED: return to initial pose ---
    if (!tracking_enabled_)
    {
        if (!last_joint_state_)
            return;

        const std::vector<std::string> &names = last_joint_state_->name;
        Eigen::VectorXd joint_pos(static_cast<Eigen::Index>(last_joint_state_->position.size()));
        for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
            joint_pos[static_cast<Eigen::Index>(i)] = last_joint_state_->position[i];
        planner_->updateEnvironmentState(names, joint_pos);

        if (!planner_->runTrackingPlanner(initial_robot_pose_))
            return;

        auto traj_ptr = planner_->getTrajectory();
        if (!traj_ptr || traj_ptr->empty())
            return;

        publishTrackingTrajectory(*traj_ptr, names);
        publishStatus("Tracking disabled: returning to initial pose");
        return;
    }

    // --- Tracking ENABLED: follow working_tip ---
    if (!updateWorkingTipPoseFromTfAndJoints())
        return;

    if (!planner_->runTrackingPlanner(latest_working_tip_world_))
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
