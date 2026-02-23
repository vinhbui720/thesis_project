#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_common/macros.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_command_language/types.h>

using namespace tesseract_environment;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_kinematics;

class RobotPlanningExample
{
public:
    RobotPlanningExample(std::shared_ptr<Environment> env, std::shared_ptr<Visualization> plotter)
        : env_(env), plotter_(plotter)
    {
    }

    bool run()
    {
        CONSOLE_BRIDGE_logInform("Running Robot Planning Example");

        // Check if environment is initialized
        if (!env_)
        {
            CONSOLE_BRIDGE_logError("Environment not initialized");
            return false;
        }

        // Get the environment's scene graph
        const auto &graph = env_->getSceneGraph();
        CONSOLE_BRIDGE_logInform("Scene Graph has %lu links", graph->getLinks().size());

        // List all links in the robot
        CONSOLE_BRIDGE_logInform("Links in the robot:");
        for (const auto &link : graph->getLinks())
        {
            CONSOLE_BRIDGE_logInform("  - %s", link->getName().c_str());
        }

        // Get root link
        const auto &root_link = graph->getRoots();
        if (!root_link.empty())
        {
            CONSOLE_BRIDGE_logInform("Root link: %s", root_link[0]->getName().c_str());
        }

        // Get all joints
        const auto &joints = graph->getJoints();
        CONSOLE_BRIDGE_logInform("Number of joints: %lu", joints.size());

        CONSOLE_BRIDGE_logInform("Joints in the robot:");
        for (const auto &joint : joints)
        {
            CONSOLE_BRIDGE_logInform("  - %s (type: %d)", joint->getName().c_str(), static_cast<int>(joint->type));
        }

        // Get kinematic groups
        std::vector<std::string> groups = env_->getGroupNames();
        CONSOLE_BRIDGE_logInform("Number of kinematic groups: %lu", groups.size());

        CONSOLE_BRIDGE_logInform("Kinematic groups:");
        for (const auto &group_name : groups)
        {
            CONSOLE_BRIDGE_logInform("  - %s", group_name.c_str());
        }

        // Test getting a kinematic group
        if (!groups.empty())
        {
            const auto &first_group = groups[0];
            auto kin_group = env_->getKinematicGroup(first_group);

            if (kin_group)
            {
                CONSOLE_BRIDGE_logInform("Kinematic group '%s' details:", first_group.c_str());
                CONSOLE_BRIDGE_logInform("  Number of joints: %lu", kin_group->numJoints());
                CONSOLE_BRIDGE_logInform("  Number of DOF: %lu", kin_group->numDOF());

                // Get joint names
                auto joint_names = kin_group->getJointNames();
                CONSOLE_BRIDGE_logInform("  Joint names:");
                for (const auto &name : joint_names)
                {
                    CONSOLE_BRIDGE_logInform("    - %s", name.c_str());
                }

                // Test forward kinematics with zero configuration
                Eigen::VectorXd zero_config = Eigen::VectorXd::Zero(kin_group->numDOF());
                auto fk_result = kin_group->calcFwdKin(zero_config);

                if (fk_result.size() == kin_group->numJoints())
                {
                    CONSOLE_BRIDGE_logInform("Forward kinematics calculated successfully for zero configuration");

                    // Print TCP pose
                    auto tcp_transform = fk_result[kin_group->numJoints() - 1];
                    auto tcp_position = tcp_transform.translation();
                    CONSOLE_BRIDGE_logInform("  TCP Position: [%.4f, %.4f, %.4f]",
                                             tcp_position(0), tcp_position(1), tcp_position(2));
                }
            }
        }

        // Visualize the environment
        if (plotter_)
        {
            CONSOLE_BRIDGE_logInform("Plotting environment");
            plotter_->plot(env_);

            // Wait for user input
            std::cout << "\nPress Enter to continue..." << std::endl;
            std::cin.get();
        }

        CONSOLE_BRIDGE_logInform("Robot Planning Example completed successfully!");
        return true;
    }

private:
    std::shared_ptr<Environment> env_;
    std::shared_ptr<Visualization> plotter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create or load environment
    auto env = std::make_shared<Environment>();

    // Try to load a basic URDF or SRDF
    // For now, we'll just demonstrate the API
    CONSOLE_BRIDGE_logInform("Tesseract Environment initialized");

    // Create visualization (optional)
    std::shared_ptr<Visualization> plotter = nullptr;
    try
    {
        plotter = std::make_shared<Visualization>();
        CONSOLE_BRIDGE_logInform("Visualization initialized");
    }
    catch (const std::exception &e)
    {
        CONSOLE_BRIDGE_logWarn("Could not initialize visualization: %s", e.what());
    }

    // Run the example
    RobotPlanningExample example(env, plotter);
    bool success = example.run();

    rclcpp::shutdown();

    return success ? 0 : 1;
}
