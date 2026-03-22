cmake_minimum_required(VERSION 3.8)
project(motomini)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

pkg_check_modules(IGNITION REQUIRED ignition-transport11 ignition-msgs8)

add_executable(object_controller src/object_controller.cpp)
ament_target_dependencies(object_controller rclcpp geometry_msgs std_msgs  visualization_msgs tf2 tf2_geometry_msgs)

add_executable(object_pose src/object_pose.cpp)

ament_target_dependencies(object_pose
  rclcpp 
  geometry_msgs 
  visualization_msgs
)

target_include_directories(object_pose PUBLIC ${IGNITION_INCLUDE_DIRS})
target_link_directories(object_pose PUBLIC ${IGNITION_LIBRARY_DIRS})
target_link_libraries(object_pose ${IGNITION_LIBRARIES})


add_executable(object_tf_broadcaster src/object_tf_broadcaster.cpp)
ament_target_dependencies(object_tf_broadcaster
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  std_srvs
  tf2_geometry_msgs
)
add_executable(env_tf_broadcaster
  src/env_tf_broadcaster.cpp
)

ament_target_dependencies(env_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(env_tf_broadcaster yaml-cpp)

add_executable(command_node src/command_node.cpp)

# Require C++17 (Good practice for modern ROS 2)
target_compile_features(command_node PUBLIC c_std_99 cxx_std_17)

# 3. Link the dependencies to the executable
ament_target_dependencies(command_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)


install(TARGETS object_controller object_pose object_tf_broadcaster env_tf_broadcaster command_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  src/manual_controller.py
  src/2axis_controller.py
  src/2axis_gui.py
  DESTINATION lib/${PROJECT_NAME}
)
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
