/**
 * @file motomini_planning_tracking.cpp
 * @brief MotoMiniPlanning::runTrackingPlanner() — lightweight real-time IK tracking.
 *
 * Called at ~5 Hz from the ROS node.  Each tick:
 *   1. Reads current joint state and the filtered working-tip target pose.
 *   2. Runs FK to keep the current end-effector orientation (position-only tracking).
 *   3. Calls calcInvKin() seeded from the previous command for branch continuity.
 *   4. Clamps per-joint step to avoid large jumps in one tick.
 *   5. Stores a two-point trajectory; the node publishes only the forward target point.
 *
 * Tuning knobs (search "TUNE"):
 *   max_joint_step   — maximum per-joint delta per tick (rad)
 *   tracking_horizon — time horizon fed to the controller (s)
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Kinematics
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/types.h>

// Command language (build the two-point trajectory)
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

// Visualization (debug plotting)
#include <tesseract_visualization/visualization.h>

// Full type definitions needed in this TU
#include <tesseract_common/joint_state.h>
#include <tesseract_state_solver/state_solver.h>

// STL
#include <algorithm> // std::clamp
#include <limits>    // std::numeric_limits

using namespace tesseract_kinematics;
using namespace tesseract_planning;

namespace Vinhtesseract_examples
{

    bool MotoMiniPlanning::runTrackingPlanner(const Eigen::Isometry3d &target_pose)
    {
        if (!env_)
            return false;

        CONSOLE_BRIDGE_logDebug("[Tracking] Running lightweight tracking planner...");

        std::shared_lock<std::shared_mutex> lock(env_mutex_);

        const std::vector<std::string> joint_names = {
            "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
        const Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);

        // --- IK seed: prefer last commanded position to avoid branch switching ---
        const Eigen::VectorXd ik_seed =
            (has_last_tracking_command_ && last_tracking_command_.size() == start_pos.size())
                ? last_tracking_command_
                : start_pos;

        KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
        if (!manip)
        {
            CONSOLE_BRIDGE_logError("[Tracking] Could not find kinematic group '%s'",
                                    manipulator_group_.c_str());
            return false;
        }

        // --- FK: keep current orientation (position-only tracking) ---
        const auto fk_map = manip->calcFwdKin(start_pos);
        auto fk_it = fk_map.find(ee_link_);
        if (fk_it == fk_map.end())
        {
            CONSOLE_BRIDGE_logError("[Tracking] FK result did not contain ee link '%s'",
                                    ee_link_.c_str());
            return false;
        }

        Eigen::Isometry3d target_pose_adjusted = target_pose;
        target_pose_adjusted.linear() = fk_it->second.linear();

        // --- IK ---
        const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;
        KinGroupIKInput ik_input(target_pose_adjusted, base_link_, ee_link_);
        IKSolutions solutions = manip->calcInvKin(ik_input, ik_seed);

        if (solutions.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] IK failed for XYZ=(%.4f, %.4f, %.4f)",
                                   target_pose_adjusted.translation().x(),
                                   target_pose_adjusted.translation().y(),
                                   target_pose_adjusted.translation().z());
            return false;
        }

        // --- Pick solution closest to the seed ---
        Eigen::VectorXd best_solution = solutions.front();
        double best_dist = std::numeric_limits<double>::max();
        for (const auto &candidate : solutions)
        {
            const double dist = (candidate - ik_seed).squaredNorm();
            if (dist < best_dist)
            {
                best_dist = dist;
                best_solution = candidate;
            }
        }

        // --- Clamp to joint limits ---
        for (Eigen::Index i = 0; i < best_solution.size() && i < joint_limits.rows(); ++i)
            best_solution[i] = std::clamp(best_solution[i], joint_limits(i, 0), joint_limits(i, 1));

        // --- TUNE: per-tick step limit and time horizon ---
        // Increase max_joint_step to move faster; decrease for smoother but slower tracking.
        const double max_joint_step = 0.08;   // rad per tick
        const double tracking_horizon = 0.18; // seconds fed to the controller

        Eigen::VectorXd commanded_solution = best_solution;
        for (Eigen::Index i = 0; i < commanded_solution.size(); ++i)
        {
            const double delta = best_solution[i] - start_pos[i];
            commanded_solution[i] = start_pos[i] + std::clamp(delta, -max_joint_step, max_joint_step);
        }

        // --- Build two-point trajectory (node publishes only the forward target) ---
        CompositeInstruction ci(
            "DEFAULT", tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));
        ci.push_back(MoveInstruction(StateWaypoint(joint_names, start_pos),
                                     MoveInstructionType::FREESPACE, "FREESPACE"));
        ci.push_back(MoveInstruction(StateWaypoint(joint_names, commanded_solution),
                                     MoveInstructionType::FREESPACE, "FREESPACE"));

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        if (trajectory.empty())
        {
            CONSOLE_BRIDGE_logWarn("[Tracking] Empty trajectory generated from IK solution");
            return false;
        }

        trajectory.front().time = 0.0;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
            trajectory[i].time = trajectory[i - 1].time + tracking_horizon;

        // --- Persist state for next tick ---
        last_tracking_command_ = commanded_solution;
        has_last_tracking_command_ = true;
        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        // --- Debug visualization ---
        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());

        if (toolpath_cb_)
        {
            std::vector<Eigen::Vector3d> ee_path;
            ee_path.reserve(trajectory.size());
            for (const auto &state : trajectory)
            {
                Eigen::Isometry3d tf = manip->calcFwdKin(state.position).at(ee_link_);
                ee_path.push_back(tf.translation());
            }
            toolpath_cb_(ee_path);
        }

        CONSOLE_BRIDGE_logDebug("[Tracking] Trajectory ready (%zu points)", trajectory.size());
        return true;
    }

} // namespace Vinhtesseract_examples
