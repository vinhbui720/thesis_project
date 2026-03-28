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
//   Publishes the full trajectory including the current-state point at t=0.
//   The motoman controller validates that trajectories start at the current position,
//   so we must include the first point. The controller will handle motion smoothly
//   from this baseline.
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
    // Expand horizon for tracking: allow longer trajectory (up to 1 second)
    const double max_horizon = std::min(1.0, std::max(min_step_dt, 0.95 * tracking_period - start_delay));

    // Use full trajectory to give controller more lookahead
    // This prevents the "too short" trajectory issue
    tesseract_common::JointTrajectory forward(tess_traj);

    // Ensure first point (current state) has zero velocity and acceleration
    forward.front().velocity.setZero();
    forward.front().acceleration.setZero();

    // Re-base time so the first published point starts at t=0
    const double t_offset = forward.front().time;
    for (auto &state : forward)
        state.time = std::max(0.0, state.time - t_offset);

    // Extend trajectory to use full horizon (don't compress!)
    const double current_horizon = forward.back().time;
    if (current_horizon > 1e-6 && current_horizon < max_horizon * 0.5)
    {
        // Trajectory is too short - expand it by scaling time
        const double scale = max_horizon * 0.7 / current_horizon;
        for (auto &state : forward)
        {
            state.time *= scale;
            // Velocity scales inversely with time (distance/time)
            for (Eigen::Index i = 0; i < state.velocity.size(); ++i)
                state.velocity[i] /= scale;
            // Acceleration scales inversely with time squared
            for (Eigen::Index i = 0; i < state.acceleration.size(); ++i)
                state.acceleration[i] /= (scale * scale);
        }
    }

    // === ENFORCE VELOCITY LIMITS ===
    const Eigen::MatrixX2d vel_limits = planner_->getTrackingVelocityLimits();
    if (vel_limits.rows() > 0)
    {
        for (auto &state : forward)
        {
            for (Eigen::Index i = 0; i < state.velocity.size() && i < vel_limits.rows(); ++i)
            {
                const double max_vel = vel_limits(i, 1);
                const double min_vel = vel_limits(i, 0);
                const double abs_max = std::max(std::abs(min_vel), std::abs(max_vel));
                state.velocity[i] = std::clamp(state.velocity[i], -abs_max, abs_max);
            }
        }
    }

    // === ACCELERATION CLAMPING (no velocity continuity enforcement) ===
    // ISP already produces kinematically feasible trajectories with proper velocities.
    // Just clamp unrealistic accelerations without further degrading velocities.
    const double max_acceleration = 12.0; // rad/s^2 — allow dynamic tracking

    for (size_t i = 0; i < forward.size(); ++i)
    {
        for (Eigen::Index j = 0; j < forward[i].acceleration.size(); ++j)
        {
            // Only clamp acceleration, don't reduce velocities
            forward[i].acceleration[j] = std::clamp(forward[i].acceleration[j],
                                                    -max_acceleration, max_acceleration);
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
