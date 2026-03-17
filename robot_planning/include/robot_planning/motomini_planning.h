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
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory> // Required for std::shared_ptr
#include <Eigen/Geometry>
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
        MotoMiniPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                         std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                         std::string manipulator_group = "manipulator",
                         std::string base_link = "world",
                         std::string ee_link = "tool0",
                         bool debug = false,
                         bool ifopt = false,
                         bool use_omp = false,
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

    private:
        std::string manipulator_group_;
        std::string base_link_;
        std::string ee_link_;
        bool online_mode_;
        bool debug_;
        bool ifopt_;
        bool use_ompl_;
        std::vector<Eigen::Isometry3d> target_poses_;

        // CHANGE: Use a shared_ptr to avoid "incomplete type" errors
        std::shared_ptr<tesseract_common::JointTrajectory> last_trajectory_;
    };

} // namespace Vinhtesseract_examples

#endif // ROBOT_PLANNING_MOTOMINI_PLANNING_H