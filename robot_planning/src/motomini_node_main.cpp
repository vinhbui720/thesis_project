/**
 * @file motomini_node_main.cpp
 * @brief Entry point for the motomini_planning_node executable.
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotoMiniPlanningNode>();
    node->postInit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
