/**
 * @file motomini_planning.h
 * @brief Planning logic for MotoMini robot using Tesseract
 *
 * @author Bùi Quang Vinh
 * @date January 2026
 */

#ifndef ROBOT_PLANNING_MOTOMINI_PLANNING_H
#define ROBOT_PLANNING_MOTOMINI_PLANNING_H

#include <tesseract_common/macros.h>
#include <tesseract_kinematics/core/kinematic_group.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory> // Required for std::shared_ptr
#include <Eigen/Geometry>
#include <thread>
#include <atomic>
#include <functional>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Include the base class
#include <robot_planning/example.h>
// Forward declare the class so we don't need the header file here
namespace tesseract_common
{
    struct JointTrajectory;
}

namespace Vinhtesseract_examples
{

    class MotoMiniPlanning : public Example
    {
    public:
        using CommandCallback = std::function<void(const Eigen::VectorXd &)>;

        MotoMiniPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                         std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                         std::string manipulator_group = "manipulator",
                         std::string base_link = "world",
                         std::string ee_link = "tool0",
                         bool debug = false,
                         bool ifopt = false,
                         bool use_ompl = false,
                         bool online_mode = false);

        ~MotoMiniPlanning() override = default;
        MotoMiniPlanning(const MotoMiniPlanning &) = default;
        MotoMiniPlanning &operator=(const MotoMiniPlanning &) = default;
        MotoMiniPlanning(MotoMiniPlanning &&) = default;
        MotoMiniPlanning &operator=(MotoMiniPlanning &&) = default;

        bool run() override final;

        void setTargetPoses(const std::vector<Eigen::Isometry3d> &poses);

        // Return a pointer instead of the full object
        std::shared_ptr<tesseract_common::JointTrajectory> getTrajectory() const;
        void setCommandCallback(CommandCallback cb);
        void stopOnlinePlanner()
        {
            is_executing_online_ = false;
            if (online_thread_.joinable())
            {
                online_thread_.join();
            }
        }
        void updateEnvironmentState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_pos);
        using ToolpathCallback = std::function<void(const std::vector<Eigen::Vector3d> &)>;

        void setToolpathCallback(ToolpathCallback cb);

        // Lightweight tracking planner (collision check only, no optimization yet)
        bool runTrackingPlanner(const Eigen::Isometry3d &target_pose);

        // Configure tracking parameters from ROS node
        void configureTracking(bool use_trajopt, bool enable_collision,
                               int num_steps, int trajopt_max_iter,
                               double max_joint_step);

    private:
        std::string manipulator_group_;
        std::string base_link_;
        std::string ee_link_;
        bool online_mode_;
        bool debug_;
        bool ifopt_;
        bool use_ompl_;
        std::vector<Eigen::Isometry3d> target_poses_;

        std::shared_ptr<tesseract_common::JointTrajectory> last_trajectory_;
        std::thread online_thread_;
        std::atomic<bool> is_executing_online_{false};
        CommandCallback command_cb_;
        mutable std::shared_mutex env_mutex_;
        ToolpathCallback toolpath_cb_;
        Eigen::VectorXd last_tracking_command_;
        bool has_last_tracking_command_{false};
        // --- Tracking TrajOpt configuration ---
        bool tracking_use_trajopt_{true};       // TrajOpt smoothing + optional collision
        bool tracking_enable_collision_{false}; // collision avoidance (slower)
        int tracking_num_steps_{5};             // interpolation steps
        int tracking_trajopt_max_iter_{5};      // max SQP iterations
        double tracking_max_joint_step_{0.15};  // rad per tick

        // --- Cached objects for fast repeated tracking calls ---
        tesseract_kinematics::KinematicGroup::ConstPtr tracking_manip_;
        Eigen::MatrixX2d tracking_joint_limits_;
        Eigen::MatrixX2d tracking_velocity_limits_;
        bool tracking_caches_valid_{false};
        void ensureTrackingCaches();
    };

} // namespace Vinhtesseract_examples

#endif // ROBOT_PLANNING_MOTOMINI_PLANNING_H