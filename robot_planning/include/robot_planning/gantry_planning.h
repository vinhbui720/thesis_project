/**
 * @file gantry_planning.h
 * @brief Planning logic for 2-axis gantry using Tesseract
 */

#ifndef ROBOT_PLANNING_GANTRY_PLANNING_H
#define ROBOT_PLANNING_GANTRY_PLANNING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <shared_mutex>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <robot_planning/example.h>

namespace tesseract_common
{
    struct JointTrajectory;
}

namespace Vinhtesseract_examples
{

    class GantryPlanning : public Example
    {
    public:
        GantryPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                       std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                       std::string manipulator_group = "gantry",
                       std::string base_link = "world",
                       std::string ee_link = "gantry_tool_link",
                       bool debug = false,
                       bool use_obstacles = false);

        ~GantryPlanning() override = default;
        GantryPlanning(const GantryPlanning &) = default;
        GantryPlanning &operator=(const GantryPlanning &) = default;
        GantryPlanning(GantryPlanning &&) = default;
        GantryPlanning &operator=(GantryPlanning &&) = default;

        bool run() override final;

        void setTargetPoses(const std::vector<Eigen::Isometry3d> &poses);
        std::shared_ptr<tesseract_common::JointTrajectory> getTrajectory() const;
        void updateEnvironmentState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_pos);

    private:
        std::string manipulator_group_;
        std::string base_link_;
        std::string ee_link_;
        bool debug_;
        bool use_obstacles_;
        std::vector<Eigen::Isometry3d> target_poses_;

        std::shared_ptr<tesseract_common::JointTrajectory> last_trajectory_;
        mutable std::shared_mutex env_mutex_;
    };

} // namespace Vinhtesseract_examples

#endif // ROBOT_PLANNING_GANTRY_PLANNING_H
