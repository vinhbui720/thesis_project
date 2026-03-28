cmake_minimum_required(VERSION 3.8)

project(robot_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(tesseract_urdf REQUIRED)

find_package(Taskflow REQUIRED HINTS /home/vinbui/vinh_ws/install/Taskflow/lib/cmake/Taskflow)

find_package(
  tesseract_motion_planners REQUIRED
  COMPONENTS core simple
  OPTIONAL_COMPONENTS
    descartes
    ompl
    trajopt
    trajopt_ifopt)
find_package(
  tesseract_time_parameterization REQUIRED COMPONENTS core isp)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

tesseract_variables()

add_library(
  ${PROJECT_NAME}
  # Planning library — split into 3 logical units for clarity and scalability:
  src/motomini_planning_base.cpp      # Constructor + shared-state helpers
  src/motomini_planning_run.cpp       # Full offline TrajOpt/OMPL pipeline (run())
  src/motomini_planning_tracking.cpp  # Lightweight real-time IK tracking
  src/gantry_planning.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  tesseract::tesseract_environment
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_trajopt_ifopt
  trajopt::trajopt_sqp
  trajopt::trajopt_ifopt
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_planning_nodes

  tesseract::tesseract_time_parameterization_core
  tesseract::tesseract_time_parameterization_isp

  Taskflow::Taskflow
  console_bridge::console_bridge)

target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

# Node executable — split into logical files for easier debugging and extension:
add_executable(motomini_planning_node
  src/motomini_node_setup.cpp       # Constructor, postInit, env init
  src/motomini_node_callbacks.cpp   # ROS subscription callbacks + monitor
  src/motomini_node_tracking.cpp    # TF lookup + tracking tick
  src/motomini_node_publish.cpp     # Trajectory & status publishers
  src/motomini_node_main.cpp        # main()
)
target_link_libraries(motomini_planning_node ${PROJECT_NAME})
ament_target_dependencies(motomini_planning_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  trajectory_msgs
  visualization_msgs
  tesseract_environment
  tesseract_common
  tesseract_visualization
  tesseract_monitoring  
  tesseract_rosutils
  tesseract_scene_graph
)

add_executable(gantry_planning_node src/gantry_planning_node.cpp)
target_link_libraries(gantry_planning_node ${PROJECT_NAME})
ament_target_dependencies(gantry_planning_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tesseract_environment
  tesseract_common
  tesseract_visualization
  tesseract_rosutils
  tesseract_scene_graph
)

add_executable(gantry_loop_controller_node src/gantry_loop_controller_node.cpp)
ament_target_dependencies(gantry_loop_controller_node
  rclcpp
  trajectory_msgs
)

add_executable(online_collision_debugger src/collision_debugger_node.cpp)
target_link_libraries(online_collision_debugger
  tesseract::tesseract_environment
  tesseract::tesseract_collision_core
)
ament_target_dependencies(online_collision_debugger
  rclcpp
  sensor_msgs
  visualization_msgs
)

add_executable(grinding_task src/grinding_task.cpp)
target_link_libraries(grinding_task ${PROJECT_NAME})
ament_target_dependencies(grinding_task
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tesseract_environment
  tesseract_common
  tesseract_rosutils
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY
  urdf
  meshes
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME}
  motomini_planning_node
  gantry_planning_node
  gantry_loop_controller_node
  online_collision_debugger
  grinding_task
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()