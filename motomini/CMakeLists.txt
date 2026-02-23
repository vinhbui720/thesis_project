cmake_minimum_required(VERSION 3.8)
project(motomini)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IGNITION REQUIRED ignition-transport11 ignition-msgs8)

add_executable(object_controller src/object_controller.cpp)
ament_target_dependencies(object_controller rclcpp geometry_msgs std_msgs  visualization_msgs)

add_executable(object_pose src/object_pose.cpp)

ament_target_dependencies(object_pose
  rclcpp 
  geometry_msgs 
  visualization_msgs
)

target_include_directories(object_pose PUBLIC ${IGNITION_INCLUDE_DIRS})
target_link_directories(object_pose PUBLIC ${IGNITION_LIBRARY_DIRS})
target_link_libraries(object_pose ${IGNITION_LIBRARIES})

install(TARGETS object_controller object_pose DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch meshes rviz urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
