cmake_minimum_required(VERSION 3.8)

# FIX: Suppress the CMP0074 warning about PCL_ROOT
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(robot_planning)

# 1. FIND DEPENDENCIES
# --------------------
find_package(ament_cmake REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)       
find_package(geometry_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tesseract_task_composer REQUIRED)
# find_package(tesseract_rosutils REQUIRED) # Commented out to fix build error
find_package(tesseract_common REQUIRED)
find_package(tesseract_visualization REQUIRED)
# find_package(tesseract_msgs) # Commented out to fix build error
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(
  tesseract_motion_planners REQUIRED
  COMPONENTS core simple
  OPTIONAL_COMPONENTS
    descartes
    ompl
    trajopt
    trajopt_ifopt)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(
  PCL REQUIRED
  COMPONENTS core
             features
             filters
             io
             segmentation
             surface)

# 2. CONSOLE BRIDGE CONFIGURATION
# -------------------------------
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

# Load Tesseract variables
tesseract_variables()

# 3. BUILD THE MAIN LIBRARY
# -------------------------
add_library(
  ${PROJECT_NAME}
  src/vinh_pick_and_place_example.cpp
  src/motomini_planning.cpp  # CORRECT: The Logic class (no main function)
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
  console_bridge::console_bridge
  ${PCL_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

# 4. PCL DEFINITIONS LOOP
# -----------------------
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if(NOT "${DEF}" STREQUAL "")
    string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
    if("${DEF_PREFIX}" STREQUAL "-m")
      string(REPLACE " " ";" DEF ${DEF})
      foreach(OPTION_DEF ${DEF})
        target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
      endforeach()
    else()
      target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
    endif()
  endif()
endforeach()

# 5. CREATE THE EXECUTABLES
# -------------------------

# A. Original Example Node
add_executable(vinh_pick_place_node src/vinh_pick_and_place_example_node.cpp)
target_link_libraries(vinh_pick_place_node ${PROJECT_NAME} rclcpp::rclcpp)
target_compile_options(vinh_pick_place_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_include_directories(vinh_pick_place_node PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

# B. MotoMini Planning Node
# CORRECT: The Node file (has main function and rclcpp includes)
add_executable(motomini_planning_node src/motomini_planning_node.cpp)

# Link to the library logic
target_link_libraries(motomini_planning_node ${PROJECT_NAME})

# Add ROS dependencies
ament_target_dependencies(motomini_planning_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# C. Bridge Nodes
# add_executable(tesseract_bridge src/tesseract_bridge.cpp)
# ament_target_dependencies(tesseract_bridge
#   rclcpp
#   trajectory_msgs
#   tesseract_msgs
# )

add_executable(hardware_bridge_node src/hardware_bridge.cpp)
ament_target_dependencies(hardware_bridge_node
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
)

add_executable(trajectory_dispatcher_node src/trajectory_dispatcher_node.cpp)
ament_target_dependencies(trajectory_dispatcher_node
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs)

# D. Test Executables
add_executable(test_header_build src/test_header_build.cpp)
target_include_directories(test_header_build PUBLIC 
    include
    ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(test_header_build
    tesseract::tesseract_common
    tesseract::tesseract_environment
    tesseract::tesseract_visualization
)

# 6. INSTALLATION SETUP
# ---------------------
# Install headers
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
# Install the library and the executable
install(TARGETS 
  ${PROJECT_NAME} 
  vinh_pick_place_node
  motomini_planning_node
  trajectory_dispatcher_node
  # tesseract_bridge
  test_header_build
  hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()