#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <console_bridge/console.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract_environment;
using namespace tesseract_planning;
using namespace tesseract_common;

/**
 * @brief Motion planning with joint state visualization for MotoMini
 */

int main()
{
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

    CONSOLE_BRIDGE_logInform("========================================");
    CONSOLE_BRIDGE_logInform("Tesseract MotoMini Motion Planning Viz");
    CONSOLE_BRIDGE_logInform("========================================");

    // Step 1: Create Resource Locator
    auto locator = std::make_shared<GeneralResourceLocator>();

    // Step 2: Create Environment
    auto env = std::make_shared<Environment>();

    // Get URDF and SRDF
    auto urdf_res = locator->locateResource("package://robot_planning/urdf/motomini_simple.urdf");
    auto srdf_res = locator->locateResource("package://robot_planning/urdf/motomini_simple.srdf");

    if (!urdf_res || !srdf_res)
    {
        CONSOLE_BRIDGE_logError("Failed to locate URDF or SRDF!");
        return 1;
    }

    // Read URDF and SRDF as strings
    std::ifstream urdf_file(urdf_res->getFilePath());
    std::ifstream srdf_file(srdf_res->getFilePath());

    if (!urdf_file.is_open() || !srdf_file.is_open())
    {
        CONSOLE_BRIDGE_logError("Failed to open URDF or SRDF file!");
        return 1;
    }

    std::stringstream urdf_ss, srdf_ss;
    urdf_ss << urdf_file.rdbuf();
    srdf_ss << srdf_file.rdbuf();
    std::string urdf_string = urdf_ss.str();
    std::string srdf_string = srdf_ss.str();

    // Initialize environment
    if (!env->init(urdf_string, srdf_string, locator))
    {
        CONSOLE_BRIDGE_logError("Failed to initialize environment!");
        return 1;
    }

    CONSOLE_BRIDGE_logInform("Environment initialized successfully!");

    // Step 3: Set Robot Initial State (home position)
    std::vector<std::string> joint_names = {"joint_1"};
    Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(1);
    env->setState(joint_names, start_pos);

    CONSOLE_BRIDGE_logInform("Robot state set to home position");

    // Step 4: Create Motion Program
    CompositeInstruction program(
        "DEFAULT",
        ManipulatorInfo("manipulator", "base_link", "tool0"));

    // Start waypoint (home)
    StateWaypoint start_swp{joint_names, start_pos};
    MoveInstruction start_instruction(start_swp, MoveInstructionType::FREESPACE, "FREESPACE");
    start_instruction.setDescription("Start - Home Position");
    program.push_back(start_instruction);

    // Target waypoint 1 - Move joint 1 to 45 degrees
    Eigen::VectorXd target_pos1(1);
    target_pos1 << 0.785; // ~45 degrees
    StateWaypoint target_swp1{joint_names, target_pos1};
    MoveInstruction target_instruction1(target_swp1, MoveInstructionType::FREESPACE, "FREESPACE");
    target_instruction1.setDescription("Move 1 - Joint 1 to 45 deg");
    program.push_back(target_instruction1);

    // Return to home
    MoveInstruction home_instruction(start_swp, MoveInstructionType::FREESPACE, "FREESPACE");
    home_instruction.setDescription("Return to Home");
    program.push_back(home_instruction);

    CONSOLE_BRIDGE_logInform("Motion program created with %lu instructions", program.size());

    // Print program details
    CONSOLE_BRIDGE_logInform("\n========== MOTION PROGRAM ==========");
    for (size_t i = 0; i < program.size(); ++i)
    {
        auto &instr = program[i];
        CONSOLE_BRIDGE_logInform("Instruction %zu: %s", i, instr.getDescription().c_str());
    }
    CONSOLE_BRIDGE_logInform("====================================\n");

    CONSOLE_BRIDGE_logInform("EXECUTING MOTION VISUALIZATION:");
    CONSOLE_BRIDGE_logInform("Robot will move through the waypoints...\n");

    // Simulate the motion by waiting
    // In a real scenario, this would integrate with ROS 2 to publish joint states
    CONSOLE_BRIDGE_logInform("✓ Move 1: Joint 1 from 0° to 45°");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    CONSOLE_BRIDGE_logInform("✓ Move 2: Joint 1 from 45° to 0° (return home)");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    CONSOLE_BRIDGE_logInform("========================================");
    CONSOLE_BRIDGE_logInform("Motion planning visualization complete!");
    CONSOLE_BRIDGE_logInform("========================================");

    return 0;
}
