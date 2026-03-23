/**
 * @file motomini_planning_base.cpp
 * @brief MotoMiniPlanning — constructor and lightweight shared-state helpers.
 *
 * Kept deliberately thin: no heavy planning headers needed here.
 * All optimizer-specific code lives in motomini_planning_run.cpp and
 * motomini_planning_tracking.cpp.
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <mutex>
#include <shared_mutex>

namespace Vinhtesseract_examples
{

    MotoMiniPlanning::MotoMiniPlanning(
        std::shared_ptr<tesseract_environment::Environment> env,
        std::shared_ptr<tesseract_visualization::Visualization> plotter,
        std::string manipulator_group,
        std::string base_link,
        std::string ee_link,
        bool debug,
        bool ifopt,
        bool use_ompl,
        bool online_mode)
        : Example(std::move(env), std::move(plotter)),
          manipulator_group_(std::move(manipulator_group)),
          base_link_(std::move(base_link)),
          ee_link_(std::move(ee_link)),
          debug_(debug),
          ifopt_(ifopt),
          use_ompl_(use_ompl),
          online_mode_(online_mode)
    {
        last_trajectory_ = nullptr;
    }

    void MotoMiniPlanning::setTargetPoses(const std::vector<Eigen::Isometry3d> &poses)
    {
        target_poses_ = poses;
    }

    std::shared_ptr<tesseract_common::JointTrajectory> MotoMiniPlanning::getTrajectory() const
    {
        return last_trajectory_;
    }

    void MotoMiniPlanning::setCommandCallback(CommandCallback cb)
    {
        command_cb_ = std::move(cb);
    }

    void MotoMiniPlanning::updateEnvironmentState(const std::vector<std::string> &joint_names,
                                                  const Eigen::VectorXd &joint_pos)
    {
        std::unique_lock<std::shared_mutex> lock(env_mutex_);
        env_->setState(joint_names, joint_pos);
    }

    void MotoMiniPlanning::setToolpathCallback(ToolpathCallback cb)
    {
        toolpath_cb_ = std::move(cb);
    }

} // namespace Vinhtesseract_examples
