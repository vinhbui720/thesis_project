/**
 * @file motomini_node_publish.cpp
 * @brief MotoMiniPlanningNode — trajectory and status publishing helpers.
 *
 *   publishStatus()             — string status to /optimization_status
 *   publishTrajectory()         — full JointTrajectory to /joint_path_command
 *   publishTrackingTrajectory() — sends only the forward target point (strips the
 *                                 leading current-state point that causes back-step jerk)
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_common/types.h>

#include <array>
#include <algorithm>

// ---------------------------------------------------------------------------
// publishStatus
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::publishStatus(const std::string &status)
{
    std_msgs::msg::String msg;
    msg.data = status;
    pub_status_->publish(msg);
}

// ---------------------------------------------------------------------------
// publishTrackingTrajectory
//   Strips the leading current-state point from a two-point trajectory and
//   sends only the forward target to the controller.  Sending [current, target]
//   every tick caused the controller to restart each time from 'current',
//   producing the back-step / forward-step jerk pattern.
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::publishTrackingTrajectory(
    const tesseract_common::JointTrajectory &tess_traj,
    const std::vector<std::string> &joint_names)
{
    if (tess_traj.empty())
        return;

    const double tracking_period = 1.0 / std::max(0.1, tracking_rate_hz_);
    const double min_step_dt = std::clamp(0.25 * tracking_period, 0.001, 0.02);
    const double start_delay = std::clamp(0.10 * tracking_period, 0.001, 0.005);
    const double max_horizon = std::max(min_step_dt, 0.95 * tracking_period - start_delay);

    if (tess_traj.size() <= 1)
    {
        publishTrajectory(tess_traj, joint_names, start_delay, min_step_dt);
        return;
    }

    tesseract_common::JointTrajectory forward(tess_traj.begin() + 1, tess_traj.end());

    // Re-base time so the first published point starts at t=0
    const double t_offset = forward.front().time;
    for (auto &state : forward)
        state.time = std::max(0.0, state.time - t_offset);

    const double current_horizon = forward.back().time;
    if (current_horizon > max_horizon && current_horizon > 1e-6)
    {
        const double scale = max_horizon / current_horizon;
        for (auto &state : forward)
        {
            state.time *= scale;
            for (Eigen::Index i = 0; i < state.velocity.size(); ++i)
                state.velocity[i] /= scale;
            for (Eigen::Index i = 0; i < state.acceleration.size(); ++i)
                state.acceleration[i] /= (scale * scale);
        }
    }

    // === ENFORCE VELOCITY LIMITS ===
    // Get velocity limits from the planner's kinematic group
    const Eigen::MatrixX2d vel_limits = planner_->getTrackingVelocityLimits();
    if (vel_limits.rows() > 0)
    {
        for (auto &state : forward)
        {
            for (Eigen::Index i = 0; i < state.velocity.size() && i < vel_limits.rows(); ++i)
            {
                const double max_vel = vel_limits(i, 1);
                const double min_vel = vel_limits(i, 0);
                // Clamp to [-max_abs, +max_abs]
                const double abs_max = std::max(std::abs(min_vel), std::abs(max_vel));
                state.velocity[i] = std::clamp(state.velocity[i], -abs_max, abs_max);
            }
        }
    }

    publishTrajectory(forward, joint_names, start_delay, min_step_dt);
}

// ---------------------------------------------------------------------------
// publishTrajectory
//   Maps a Tesseract JointTrajectory onto the six controller joints by name,
//   enforces a minimum inter-point time spacing, and stamps the message
//   slightly in the future so it never arrives "in the past".
// ---------------------------------------------------------------------------
void MotoMiniPlanningNode::publishTrajectory(
    const tesseract_common::JointTrajectory &tess_traj,
    const std::vector<std::string> &joint_names,
    double start_delay_sec,
    double min_step_dt_sec)
{
    if (tess_traj.empty())
        return;

    // The six joints expected by the Motoman controller (in order)
    static const std::vector<std::string> controlled_joints = {
        "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

    const double min_step_dt = std::max(0.001, min_step_dt_sec);
    const double start_delay = std::max(0.0, start_delay_sec);

    trajectory_msgs::msg::JointTrajectory ros_msg;
    ros_msg.header.stamp = this->now() + rclcpp::Duration::from_seconds(start_delay);
    ros_msg.header.frame_id = "world";
    ros_msg.joint_names = controlled_joints;

    // Build joint-name → source-index mapping
    std::array<int, 6> source_indices{};
    source_indices.fill(-1);

    bool use_name_mapping =
        !joint_names.empty() &&
        (tess_traj.front().position.size() == static_cast<Eigen::Index>(joint_names.size()));

    if (use_name_mapping)
    {
        for (size_t i = 0; i < controlled_joints.size(); ++i)
        {
            auto it = std::find(joint_names.begin(), joint_names.end(), controlled_joints[i]);
            if (it == joint_names.end())
            {
                use_name_mapping = false;
                break;
            }
            source_indices[i] = static_cast<int>(std::distance(joint_names.begin(), it));
        }
    }

    double prev_time = 0.0;
    for (size_t point_idx = 0; point_idx < tess_traj.size(); ++point_idx)
    {
        const auto &state = tess_traj[point_idx];
        trajectory_msgs::msg::JointTrajectoryPoint point;

        double t = state.time;
        t = (point_idx == 0) ? std::max(t, min_step_dt)
                             : std::max(t, prev_time + min_step_dt);
        prev_time = t;
        point.time_from_start = rclcpp::Duration::from_seconds(t);

        for (size_t i = 0; i < controlled_joints.size(); ++i)
        {
            const int src = use_name_mapping ? source_indices[i] : static_cast<int>(i);

            double pos = 0.0, vel = 0.0, acc = 0.0;
            if (src >= 0 && src < state.position.size())
                pos = state.position[src];
            if (src >= 0 && src < state.velocity.size())
                vel = state.velocity[src];
            if (src >= 0 && src < state.acceleration.size())
                acc = state.acceleration[src];

            point.positions.push_back(pos);
            point.velocities.push_back(vel);
            point.accelerations.push_back(acc);
        }

        ros_msg.points.push_back(point);
    }

    pub_trajectory_->publish(ros_msg);
}
