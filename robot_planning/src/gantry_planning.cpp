#include <robot_planning/gantry_planning.h>

#include <tesseract_common/joint_state.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_environment/environment.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_kinematics/core/kinematic_group.h>

#include <limits>
#include <mutex>

#include <console_bridge/console.h>

using namespace tesseract_planning;

namespace Vinhtesseract_examples
{
    GantryPlanning::GantryPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                                   std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                   std::string manipulator_group,
                                   std::string base_link,
                                   std::string ee_link,
                                   bool debug,
                                   bool use_obstacles)
        : Example(std::move(env), std::move(plotter)), manipulator_group_(std::move(manipulator_group)), base_link_(std::move(base_link)), ee_link_(std::move(ee_link)), debug_(debug), use_obstacles_(use_obstacles)
    {
        last_trajectory_ = nullptr;
    }

    void GantryPlanning::setTargetPoses(const std::vector<Eigen::Isometry3d> &poses)
    {
        target_poses_ = poses;
    }

    std::shared_ptr<tesseract_common::JointTrajectory> GantryPlanning::getTrajectory() const
    {
        return last_trajectory_;
    }

    void GantryPlanning::updateEnvironmentState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_pos)
    {
        std::unique_lock<std::shared_mutex> lock(env_mutex_);
        env_->setState(joint_names, joint_pos);
    }

    bool GantryPlanning::run()
    {
        if (debug_)
            console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
        else
            console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

        if (!env_ || target_poses_.empty())
            return false;

        if (!use_obstacles_)
            CONSOLE_BRIDGE_logInform("Gantry planner running in IK-only mode (obstacle checking disabled).");
        else
            CONSOLE_BRIDGE_logWarn("Gantry planner obstacle flag is enabled, but simplified planner currently performs IK + interpolation without collision optimization.");

        std::shared_lock<std::shared_mutex> lock(env_mutex_);

        const std::vector<std::string> joint_names = {"joint_x", "joint_z"};
        Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);
        env_->setState(joint_names, start_pos);

        tesseract_kinematics::KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
        if (manip == nullptr)
        {
            CONSOLE_BRIDGE_logError("Could not find kinematic group '%s'", manipulator_group_.c_str());
            return false;
        }

        const Eigen::Isometry3d start_ee_tf = manip->calcFwdKin(start_pos).at(ee_link_);
        const Eigen::Matrix3d fixed_orientation = start_ee_tf.linear();

        const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;

        std::vector<Eigen::VectorXd> joint_waypoints;
        joint_waypoints.reserve(target_poses_.size() + 1);
        joint_waypoints.push_back(start_pos);

        for (const auto &target_pose_world : target_poses_)
        {
            Eigen::Isometry3d target_pose = target_pose_world;
            target_pose.linear() = fixed_orientation;

            tesseract_kinematics::KinGroupIKInput ik_input(target_pose, base_link_, ee_link_);
            tesseract_kinematics::IKSolutions solutions = manip->calcInvKin(ik_input, joint_waypoints.back());
            if (solutions.empty())
            {
                CONSOLE_BRIDGE_logError("Gantry IK failed for target pose at XYZ=(%.4f, %.4f, %.4f)",
                                        target_pose.translation().x(),
                                        target_pose.translation().y(),
                                        target_pose.translation().z());
                return false;
            }

            Eigen::VectorXd best_solution = solutions.front();
            double best_dist = std::numeric_limits<double>::max();
            for (const auto &candidate : solutions)
            {
                const double dist = (candidate - joint_waypoints.back()).squaredNorm();
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_solution = candidate;
                }
            }

            for (Eigen::Index i = 0; i < best_solution.size() && i < joint_limits.rows(); ++i)
            {
                const double lower = joint_limits(i, 0);
                const double upper = joint_limits(i, 1);
                if (best_solution[i] < lower || best_solution[i] > upper)
                {
                    CONSOLE_BRIDGE_logError("Gantry IK solution violates limits on joint index %ld (value=%.6f, limits=[%.6f, %.6f])",
                                            static_cast<long>(i),
                                            best_solution[i],
                                            lower,
                                            upper);
                    return false;
                }
            }

            joint_waypoints.push_back(best_solution);
        }

        CompositeInstruction ci("DEFAULT", tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));
        for (const auto &joint_wp : joint_waypoints)
        {
            ci.push_back(MoveInstruction(StateWaypoint(joint_names, joint_wp), MoveInstructionType::FREESPACE, "FREESPACE"));
        }

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        if (trajectory.empty())
        {
            CONSOLE_BRIDGE_logError("Gantry trajectory generation failed: empty trajectory");
            return false;
        }

        const double max_joint_speed = 0.05; // m/s for prismatic axes
        trajectory.front().time = 0.0;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const Eigen::VectorXd delta = (trajectory[i].position - trajectory[i - 1].position).cwiseAbs();
            const double max_delta = delta.maxCoeff();
            const double dt = std::max(0.05, max_delta / max_joint_speed);
            trajectory[i].time = trajectory[i - 1].time + dt;
        }

        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        if (debug_ && plotter_ != nullptr && plotter_->isConnected())
        {
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());
        }

        return true;
    }

} // namespace Vinhtesseract_examples
