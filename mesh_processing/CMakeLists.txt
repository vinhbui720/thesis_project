cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(mesh_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------
# Executable 1: STL to Point Cloud
# ------------------------------------------------------------
add_executable(model_manager_node src/stl_to_cloud_node.cpp)

target_link_libraries(model_manager_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(model_manager_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  pcl_conversions
  ament_index_cpp
  tf2_ros
)
# ------------------------------------------------------------
# Executable 2: Scene Preprocessor (Step 1)
# ------------------------------------------------------------
add_executable(scene_preprocessor src/scene_preprocessor.cpp)
target_link_libraries(scene_preprocessor ${PCL_LIBRARIES})

ament_target_dependencies(scene_preprocessor
  rclcpp
  sensor_msgs
  pcl_conversions
  rcl_interfaces
)
# ------------------------------------------------------------
# Executable 3: Global Registration (Step 2)
# ------------------------------------------------------------
add_executable(pose_estimator src/pose_estimator.cpp)
target_link_libraries(pose_estimator ${PCL_LIBRARIES})

ament_target_dependencies(pose_estimator
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
)

# ------------------------------------------------------------
# Executable 4: Trajectory Visualizer (RViz)
# ------------------------------------------------------------
add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
target_link_libraries(trajectory_visualizer
  jsoncpp
  Eigen3::Eigen
)

target_include_directories(trajectory_visualizer PRIVATE
  /usr/include/jsoncpp
)

ament_target_dependencies(trajectory_visualizer
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  ament_index_cpp
  tf2_ros
  tf2
  Eigen3
)

# Executable 5: Trajectory Debug
# ------------------------------------------------------------
add_executable(trajectory_debug src/trajectory_debug.cpp)
target_link_libraries(trajectory_debug
  ${PCL_LIBRARIES}
)

ament_target_dependencies(trajectory_debug
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
)


# ------------------------------------------------------------
# Install Directives
# ------------------------------------------------------------
install(TARGETS
  model_manager_node
  scene_preprocessor
  pose_estimator
  trajectory_visualizer
  trajectory_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  mesh
  launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
