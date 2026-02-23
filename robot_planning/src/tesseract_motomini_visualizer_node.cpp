#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sstream>
#include <console_bridge/console.h>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_common/macros.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>

using namespace tesseract_environment;
using namespace tesseract_common;

class TesseractMotominiVisualizer
{
public:
    TesseractMotominiVisualizer() : env_(nullptr)
    {
    }

    bool initialize()
    {
        CONSOLE_BRIDGE_logInform("========================================");
        CONSOLE_BRIDGE_logInform("Tesseract MotoMini Visualizer");
        CONSOLE_BRIDGE_logInform("========================================");

        // Create resource locator
        auto locator = std::make_shared<GeneralResourceLocator>();

        // Create environment
        env_ = std::make_shared<Environment>();

        // Get URDF and SRDF paths
        std::string urdf_pkg = "package://robot_planning/urdf/motomini_simple.urdf";
        std::string srdf_pkg = "package://robot_planning/urdf/motomini_simple.srdf";

        auto urdf_res = locator->locateResource(urdf_pkg);
        auto srdf_res = locator->locateResource(srdf_pkg);

        if (!urdf_res || !srdf_res)
        {
            CONSOLE_BRIDGE_logError("Failed to locate URDF or SRDF!");
            CONSOLE_BRIDGE_logError("URDF: %s", urdf_pkg.c_str());
            CONSOLE_BRIDGE_logError("SRDF: %s", srdf_pkg.c_str());
            if (urdf_res)
                CONSOLE_BRIDGE_logError("URDF file: %s", urdf_res->getFilePath().c_str());
            if (srdf_res)
                CONSOLE_BRIDGE_logError("SRDF file: %s", srdf_res->getFilePath().c_str());
            return false;
        }

        CONSOLE_BRIDGE_logInform("URDF file: %s", urdf_res->getFilePath().c_str());
        CONSOLE_BRIDGE_logInform("SRDF file: %s", srdf_res->getFilePath().c_str());

        // Read URDF and SRDF as strings
        std::ifstream urdf_file(urdf_res->getFilePath());
        std::ifstream srdf_file(srdf_res->getFilePath());

        if (!urdf_file.is_open() || !srdf_file.is_open())
        {
            CONSOLE_BRIDGE_logError("Failed to open URDF or SRDF file!");
            return false;
        }

        std::stringstream urdf_ss, srdf_ss;
        urdf_ss << urdf_file.rdbuf();
        srdf_ss << srdf_file.rdbuf();
        std::string urdf_string = urdf_ss.str();
        std::string srdf_string = srdf_ss.str();

        // Initialize environment with strings
        if (!env_->init(urdf_string, srdf_string, locator))
        {
            CONSOLE_BRIDGE_logError("Failed to initialize environment!");
            return false;
        }

        CONSOLE_BRIDGE_logInform("Environment initialized successfully!");

        // Get number of groups
        CONSOLE_BRIDGE_logInform("Kinematic Groups: %zu", env_->getGroupNames().size());

        // Set initial state (home position)
        std::vector<std::string> joint_names = {"joint_1"};
        Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(1);

        env_->setState(joint_names, joint_pos);
        CONSOLE_BRIDGE_logInform("Robot state set to home position");

        // Load visualization
        try
        {
            // Try to connect to visualization
            CONSOLE_BRIDGE_logInform("Attempting to connect to visualization...");
            // Note: Visualization connection depends on external Ignition setup
            CONSOLE_BRIDGE_logWarn("Visualization setup requires external Ignition Gazebo instance");
        }
        catch (const std::exception &e)
        {
            CONSOLE_BRIDGE_logWarn("Could not connect to visualization: %s", e.what());
        }

        return true;
    }

    void printRobotInfo()
    {
        if (!env_)
            return;

        CONSOLE_BRIDGE_logInform("\n========== ROBOT INFORMATION ==========");

        // Print groups
        CONSOLE_BRIDGE_logInform("Kinematic Groups:");
        for (const auto &group : env_->getGroupNames())
        {
            CONSOLE_BRIDGE_logInform("  - %s", group.c_str());
        }

        CONSOLE_BRIDGE_logInform("========================================\n");
    }

    std::shared_ptr<Environment> getEnvironment() const { return env_; }

private:
    std::shared_ptr<Environment> env_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tesseract_motomini_visualizer");

    // Set log level
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

    // Create visualizer
    auto visualizer = std::make_shared<TesseractMotominiVisualizer>();

    if (!visualizer->initialize())
    {
        CONSOLE_BRIDGE_logError("Failed to initialize visualizer!");
        rclcpp::shutdown();
        return 1;
    }

    // Print robot information
    visualizer->printRobotInfo();

    // Keep running
    CONSOLE_BRIDGE_logInform("Visualizer running. Press Ctrl+C to exit.");

    std::thread spinner([node]()
                        { rclcpp::spin(node); });

    // Wait for user input or shutdown signal
    std::string line;
    std::getline(std::cin, line);

    rclcpp::shutdown();
    spinner.join();

    CONSOLE_BRIDGE_logInform("Visualizer stopped");
    return 0;
}
