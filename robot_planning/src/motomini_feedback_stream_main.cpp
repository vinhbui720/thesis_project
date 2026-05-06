#include "robot_planning/motomini_feedback_stream_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_planning::MotoMiniFeedbackStreamNode>());
  rclcpp::shutdown();
  return 0;
}
