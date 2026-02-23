#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_common/macros.h>
#include <tesseract_environment/environment.h>

using namespace tesseract_environment;

class RobotPlanningTestNode : public rclcpp::Node
{
public:
    RobotPlanningTestNode() : Node("robot_planning_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "================================");
        RCLCPP_INFO(this->get_logger(), "Robot Planning Test Node Started");
        RCLCPP_INFO(this->get_logger(), "================================");

        // Initialize tesseract environment
        env_ = std::make_shared<Environment>();
        RCLCPP_INFO(this->get_logger(), "✓ Tesseract Environment created");

        // Test environment object
        testEnvironmentBasics();

        RCLCPP_INFO(this->get_logger(), "================================");
        RCLCPP_INFO(this->get_logger(), "Test Complete!");
        RCLCPP_INFO(this->get_logger(), "================================");
    }

private:
    std::shared_ptr<Environment> env_;

    void testEnvironmentBasics()
    {
        if (!env_)
        {
            RCLCPP_ERROR(this->get_logger(), "Environment is nullptr!");
            return;
        }

        try
        {
            RCLCPP_INFO(this->get_logger(), "\n--- Environment Information ---");
            RCLCPP_INFO(this->get_logger(), "✓ Environment object created successfully");

            // Get kinematic groups (this should work even with empty env)
            auto groups = env_->getGroupNames();
            RCLCPP_INFO(this->get_logger(), "Number of Kinematic Groups: %zu", groups.size());

            if (groups.size() > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Groups found:");
                for (const auto &group : groups)
                {
                    RCLCPP_INFO(this->get_logger(), "  - %s", group.c_str());
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "No kinematic groups loaded (environment is empty)");
            }

            RCLCPP_INFO(this->get_logger(), "\n--- Test Results ---");
            RCLCPP_INFO(this->get_logger(), "✓ Tesseract Environment working!");
            RCLCPP_INFO(this->get_logger(), "✓ Successfully integrated with your ROS 2 package!");

            RCLCPP_INFO(this->get_logger(), "\nYou can now:");
            RCLCPP_INFO(this->get_logger(), "1. Load URDF/SRDF files");
            RCLCPP_INFO(this->get_logger(), "2. Perform forward/inverse kinematics");
            RCLCPP_INFO(this->get_logger(), "3. Setup collision detection");
            RCLCPP_INFO(this->get_logger(), "4. Create motion planning tasks");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPlanningTestNode>();
    rclcpp::shutdown();
    return 0;
}
