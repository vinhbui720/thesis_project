/**
 * @file motomini_node_publish.cpp
 * @brief MotoMiniPlanningNode — trajectory and status publishing helpers.
 *
 *   publishStatus()      — string status to /optimization_status
 *   publishTrajectory()  — full JointTrajectory to /path_command
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
// publishTrajectory
//   Maps a Tesseract JointTrajectory onto the six controller joints by name,
//   enforces a minimum inter-point time spacing, and stamps the message
//   slightly in the future so it never arrives "in the past".
// ---------------------------------------------------------------------------
bool MotoMiniPlanningNode::publishTrajectory(
    const tesseract_common::JointTrajectory &tess_traj,
    const std::vector<std::string> &joint_names,
    double start_delay_sec,
    double min_step_dt_sec)
{
    if (tess_traj.empty())
        return false;

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

    const bool all_points_staged = ros_msg.points.size() == tess_traj.size();
    if (!all_points_staged)
        return false;

    pub_trajectory_->publish(ros_msg);
    return true;
}
