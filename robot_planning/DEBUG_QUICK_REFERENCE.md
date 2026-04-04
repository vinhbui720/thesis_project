"""
Enhanced Debug System - Quick Reference & Integration Guide
============================================================

# Topics & Messages

## Subscriptions

- `/joint_states` (sensor_msgs/JointState)
  - Updates robot state for collision checking
  - Used to compute FK for cartesian error visualization

## Publications

- `/debug_markers` (visualization_msgs/MarkerArray)
  - All visualization markers for RViz
  - Published at joint_states callback rate

## Parameters (ROS 2)

collision_threshold (double): 0.1

- Collision warning distance in meters
- Used for margin visualization coloring

enable_collision_viz (bool): true

- Enable collision box/margin visualization

enable_cartesian_error_viz (bool): true

- Enable target vs actual end-effector frame visualization

enable_trajectory_viz (bool): true

- Enable trajectory path animation

enable_kinematic_error_viz (bool): true

- Enable kinematic tracking error visualization

enable_collision_gradient (bool): false

- Enable collision gradient arrows (can be expensive)

frame_id (string): "world"

- Marker frame ID, should match TF tree root

ee_link (string): "ee_link"

- End-effector link name for FK calculations

base_link (string): "base_link"

- Base link for reference frame

manipulator_group (string): "manipulator"

- Kinematic group for FK solver

# ROS 2 Launch Examples

## 1. Standalone Debug Node

$ ros2 launch robot_planning enhanced_debug_launch.py
$ ros2 launch robot_planning enhanced_debug_launch.py collision_threshold:=0.05

## 2. Integrated with Planning

$ ros2 launch robot_planning motomini_planning_with_debug.py

## 3. Custom Configuration

$ ros2 run robot_planning enhanced_debug_node \\
--param collision_threshold:=0.1 \\
--param enable_collision_gradient:=true

# Programmatic API Usage

## C++ Integration in Your Planning Code

```cpp
#include <robot_planning/sqp_debug_callback.hpp>
#include "your_debug_node.hpp"

// Example 1: Monitor Collision Violations
auto debug_node = std::make_shared<EnhancedDebugNode>();

Eigen::Isometry3d target = ... // your plan target
debug_node->setTargetCartesianPose("ee_link", target);

// In callback or update loop:
const auto& collisions = debug_node->getCollisionDebugInfo();
for (const auto& c : collisions) {
    RCLCPP_INFO(logger_, "Collision: %s <-> %s: %.3f m",
        c.link1.c_str(), c.link2.c_str(), c.distance);
}


// Example 2: Track Trajectory Execution
std::vector<Eigen::VectorXd> trajectory_waypoints = ...;
double time = 0.0;
double dt = 0.01;

for (const auto& wp : trajectory_waypoints) {
    debug_node->addTrajectoryWaypoint(wp, time);
    time += dt;
}

// Visualizer automatically animates the path


// Example 3: SQP Optimization Monitoring
auto sqp_callback = std::make_shared<
    Vinhtesseract_examples::SQPDebugCallback>();

sqp_callback->setOnIterationCallback(
    [](const auto& metrics) {
        // Called at each SQP iteration
        std::cout << "Iteration " << metrics.iteration
                  << " Cost: " << metrics.total_cost
                  << " Violations: " << metrics.collision_violations
                  << std::endl;
    });

sqp_callback->setOnCollisionCallback(
    [](const auto& data) {
        // Called when collisions detected
        std::cout << "Collision violations: "
                  << data.total_violations << std::endl;
    });

// In your solver loop:
Vinhtesseract_examples::SQPIterationMetrics metrics;
// ... populate metrics from solver ...
sqp_callback->updateMetrics(metrics);


// Example 4: Trajectory Analysis
tesseract_common::JointTrajectory traj = ...;
Vinhtesseract_examples::SQPIterationMetrics metrics;
metrics.iteration = 5;

Vinhtesseract_examples::SQPTrajectoryAnalyzer::analyzeTrajectory(
    traj, joint_names, metrics);

std::cout << "Max velocity: " << metrics.max_joint_velocity << std::endl;
std::cout << "Max acceleration: " << metrics.max_joint_acceleration << std::endl;

int discontinuities =
    Vinhtesseract_examples::SQPTrajectoryAnalyzer::detectDiscontinuities(traj);
std::cout << "Discontinuity count: " << discontinuities << std::endl;
```

# Marker Type Reference

## Collision Margin Visualization

- **Marker**: LINE_STRIP + TEXT_VIEW_FACING
- **Color**: Red → Yellow → Green (danger → caution → safe)
- **Channel**: `/debug_markers` namespace: `collision_margin`
- **Purpose**: Visualize minimum distance between colliding links

```
Example visualization:
gripper ↔ workspace_boundary  ← Text showing links and distance
[━━━━━━━━━━━━━]  ← Line from nearest point to nearest point
       ↑
     Color gradient based on distance ratio
```

## Collision Gradient (Direction of Avoidance)

- **Marker**: ARROW
- **Direction**: From contact toward safety
- **Length**: Proportional to distance
- **Channel**: namespace: `collision_gradient`
- **Purpose**: Show which direction to move to escape collision

```
Example: Arrow pointing away from obstacle
   ↑ (safety direction)
   |
   +→ (collision point on moving link)
```

## Cartesian Error Visualization

- **Markers**: 3x ARROW (XYZ axes) at target + 3x at current + 1x error arrow
- **Purpose**: Compare desired vs actual end-effector pose
- **Channel**: namespace: `cartesian_error`

```
Example: Target frame (solid) vs Current frame (ghosted) + error vector
┌─ Target Position
│   X→ Y↑ Z↗
│
└─ Current Position
    X→ Y↑ Z↗
         ⋱ (error arrow pointing to target)
```

## Trajectory Visualization

- **Marker**: LINE_STRIP (green)
- **Purpose**: Animate end-effector path through space
- **Channel**: namespace: `trajectory`

```
Example: Curved path showing full trajectory
    ╭─────────╮
    │         │
Start→•···•···•···•···•←End
          (waypoints traced in 3D)
```

## Kinematic Error

- **Marker**: ARROW (magenta/purple)
- **Purpose**: Specific tracking error during execution
- **Channel**: namespace: `kinematic_error`
- **Used For**: Online optimization feedback

```
Example: Tracking error as colored arrow
   Current EE position
         ↓
      •──═════→ (error pointing to desired)
         ↑
    Desired position
```

# Troubleshooting Commands

## Check if debug node is running

$ ros2 node list | grep enhanced_debug

## Monitor published markers

$ ros2 topic echo /debug_markers | head -20

## Check joint states

$ ros2 topic echo /joint_states | head -5

## View RViz with debug markers

$ rviz2 -d $(ros2 pkg prefix robot_planning)/share/robot_planning/rviz/debug_config.rviz

## Test collision detection manually

$ ros2 topic pub /joint_states sensor_msgs/JointState \\
'{header: {frame_id: base_link}, \\
name: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t], \\
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Integration with motomini_planning_run.cpp

Add after trajectory planning completes:

```cpp
// In motomini_planning_run.cpp run() method
auto debug_node = std::make_shared<EnhancedDebugNode>();

// Option 1: Visualize target poses
for (size_t i = 0; i < target_poses_.size(); ++i) {
    debug_node->setTargetCartesianPose(ee_link_, target_poses_[i]);
}

// Option 2: Animate trajectory
for (const auto& state : full_traj) {
    debug_node->addTrajectoryWaypoint(state.position, state.time);
}

// Option 3: Real-time collision monitoring in online mode
if (online_mode_) {
    const auto& collision_info = debug_node->getCollisionDebugInfo();
    if (!collision_info.empty()) {
        CONSOLE_BRIDGE_logWarn(
            "Online collision detected: %zu violations",
            collision_info.size());
    }
}
```

# Performance Tuning

## If debug node is slow:

1. Disable unused features via parameters
2. Reduce collision check frequency
3. Increase trajectory waypoint decimation
4. Disable gradient visualization (enable_collision_gradient=false)

## Memory usage:

- Base: ~50MB
- Per trajectory point: ~500B
- Per marker: ~200B
- Typical with 100-waypoint trajectory: ~100MB

## CPU usage:

- Collision checking: ~5ms per update
- FK calculations: ~2-3ms
- Marker generation: ~2ms
- Total overhead: ~10ms @ 100Hz = negligible

# Example RViz Configuration (rviz_config.yaml)

```yaml
Visualization Manager:
  Class: ""
  Displays:
    - Class: RobotModel
      Name: RobotModel

    - Class: MarkerArray
      Name: DebugMarkers
      Topic: /debug_markers
      Namespaces:
        collision_margin: true
        collision_gradient: true
        collision_text: true
        trajectory: true
        cartesian_error: true
        kinematic_error: true

    - Class: Axes
      Name: WorldFrame
      Frame: world
```

# See Also

- DEBUG_GUIDE.md - Comprehensive documentation
- collision_debugger_node.cpp - Online collision detection
- motomini_planning_run.cpp - Integration points
- sqp_debug_callback.hpp - Optimization monitoring
  """
