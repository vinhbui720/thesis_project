#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_common/macros.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_motion_planners/simple/simple.h>
#include <tesseract_motion_planners/profile_dictionary.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_motion_planners;
using namespace tesseract_planning;

class RobotMotionPlanningExample
{
public:
    RobotMotionPlanningExample()
    {
        CONSOLE_BRIDGE_logInform("=== Robot Motion Planning Example ===");

        // Create environment
        env_ = std::make_shared<Environment>();

        // Try to load sample robot
        try
        {
            std::string package_share_dir =
                ament_index_cpp::get_package_share_directory("tesseract_support");
            std::string urdf_path = package_share_dir + "/urdf/abb_irb1200_5_90.urdf";
            std::string srdf_path = package_share_dir + "/urdf/abb_irb1200_5_90.srdf";

            if (!env_->loadURDFFile(urdf_path, srdf_path))
            {
                CONSOLE_BRIDGE_logError("Failed to load URDF file");
                return;
            }

            CONSOLE_BRIDGE_logInform("URDF loaded successfully");
            robot_loaded_ = true;
        }
        catch (const std::exception &e)
        {
            CONSOLE_BRIDGE_logWarn("Could not load robot: %s", e.what());
        }
    }

    bool runExample()
    {
        if (!robot_loaded_)
        {
            CONSOLE_BRIDGE_logError("Robot not loaded");
            return false;
        }

        // Get kinematic groups
        auto groups = env_->getGroupNames();
        if (groups.empty())
        {
            CONSOLE_BRIDGE_logError("No kinematic groups found");
            return false;
        }

        std::string group_name = groups[0];
        CONSOLE_BRIDGE_logInform("Using kinematic group: %s", group_name.c_str());

        auto kin_group = env_->getKinematicGroup(group_name);
        if (!kin_group)
        {
            CONSOLE_BRIDGE_logError("Failed to get kinematic group");
            return false;
        }

        // Create start and goal configurations
        Eigen::VectorXd start_config = Eigen::VectorXd::Zero(kin_group->numDOF());
        Eigen::VectorXd goal_config = Eigen::VectorXd::Zero(kin_group->numDOF());

        // Set goal configuration (some random joint values)
        for (int i = 0; i < static_cast<int>(goal_config.size()); ++i)
        {
            goal_config(i) = (i % 2) ? 0.5 : -0.5;
        }

        CONSOLE_BRIDGE_logInform("Start Configuration: %s", start_config.transpose().format(Eigen::IOFormat(2, 0, ", ", "\n", "[", "]")).c_str());
        CONSOLE_BRIDGE_logInform("Goal Configuration: %s", goal_config.transpose().format(Eigen::IOFormat(2, 0, ", ", "\n", "[", "]")).c_str());

        // Test forward kinematics
        testForwardKinematics(kin_group, start_config, "Start");
        testForwardKinematics(kin_group, goal_config, "Goal");

        return true;
    }

private:
    std::shared_ptr<Environment> env_;
    bool robot_loaded_{false};

    void testForwardKinematics(const std::shared_ptr<KinematicGroup> &kin_group,
                               const Eigen::VectorXd &config,
                               const std::string &label)
    {
        auto fk_result = kin_group->calcFwdKin(config);

        if (fk_result.empty())
        {
            CONSOLE_BRIDGE_logError("FK calculation failed for %s configuration", label.c_str());
            return;
        }

        const auto &tcp_transform = fk_result.back();
        const auto &tcp_pos = tcp_transform.translation();
        Eigen::Quaterniond tcp_quat(tcp_transform.rotation());

        CONSOLE_BRIDGE_logInform("%s Configuration FK Result:", label.c_str());
        CONSOLE_BRIDGE_logInform("  Position: [%.4f, %.4f, %.4f]",
                                 tcp_pos(0), tcp_pos(1), tcp_pos(2));
        CONSOLE_BRIDGE_logInform("  Orientation (quat): [%.4f, %.4f, %.4f, %.4f]",
                                 tcp_quat.x(), tcp_quat.y(), tcp_quat.z(), tcp_quat.w());
    }
};

int main()
{
    CONSOLE_BRIDGE_setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_INFO);

    RobotMotionPlanningExample example;
    bool success = example.runExample();

    return success ? 0 : 1;
}
