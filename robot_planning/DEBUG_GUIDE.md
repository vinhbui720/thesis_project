# Enhanced Debug System - User Guide

## Overview

This enhanced debugging system provides comprehensive visualization of planning and optimization processes in Tesseract. It integrates multiple marker types and visualization strategies to monitor:

1. **Collision Box/Margin** - Contact distance visualization
2. **Collision Gradient** - Direction of collision avoidance
3. **Cartesian Error** - End-effector position/orientation tracking
4. **Joint Trajectory** - Complete path animation
5. **Kinematic Error** - Tracking errors during execution
6. **SQP Collision Debug** - Real-time optimization monitoring

---

## Architecture

### Node Structure

```
enhanced_debug_node.cpp
├── OnlineCollisionDebugger (existing)
│   └── Real-time collision checking on joint state updates
│
└── EnhancedDebugNode (new)
    ├── 1. Collision Detection & Visualization
    │   ├── ContactResultsMarker (line + text labels)
    │   └── ArrowMarker (gradient direction)
    │
    ├── 2. Cartesian Error Visualization
    │   ├── AxisMarker at target (expected)
    │   ├── AxisMarker at current (actual)
    │   └── ArrowMarker (error vector)
    │
    ├── 3. Trajectory Visualization
    │   ├── LineStrip showing end-effector path
    │   └── Time-indexed waypoint tracking
    │
    ├── 4. Kinematic Error Visualization
    │   └── Orientation error indication
    │
    └── 5. SQP Debug Callback Framework
        ├── Metrics collection at each iteration
        ├── Collision violation detection
        └── Constraint satisfaction monitoring
```

---

## Usage Guide

### 1. Basic Launch

```bash
ros2 launch robot_planning enhanced_debug_launch.py
```

This will:

- Start the debug node
- Subscribe to `/joint_states`
- Publish visualization markers to `/debug_markers`
- Display in RViz

### 2. Configure Debug Features

Edit `enhanced_debug_node.cpp` parameters or pass via ROS parameters:

```bash
ros2 run robot_planning enhanced_debug_node \
    --param collision_threshold:=0.1 \
    --param enable_collision_viz:=true \
    --param enable_cartesian_error_viz:=true \
    --param enable_trajectory_viz:=true \
    --param enable_kinematic_error_viz:=true
```

### 3. Programmatic API Usage

#### A. Basic Setup

```cpp
#include "robot_planning/enhanced_debug_node.cpp"

auto debug_node = std::make_shared<EnhancedDebugNode>();

// Get collision debug info
const auto& collision_info = debug_node->getCollisionDebugInfo();
for (const auto& info : collision_info)
{
    std::cout << "Collision: " << info.link1 << " <-> " << info.link2
              << " Distance: " << info.distance << std::endl;
}
```

#### B. Set Cartesian Targets for Error Visualization

```cpp
Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
target_pose.translation() = Eigen::Vector3d(0.5, 0.0, 0.5);

debug_node->setTargetCartesianPose("ee_link", target_pose);
// Now cartesian error will be visualized in RViz
```

#### C. Add Trajectory for Animation

```cpp
tesseract_common::JointTrajectory traj = ... // your planned trajectory

for (size_t i = 0; i < traj.size(); ++i)
{
    debug_node->addTrajectoryWaypoint(traj[i].position, traj[i].time);
}

// Later, clear when done
debug_node->clearTrajectory();
```

#### D. Integration with motomini_planning_run.cpp

Add to `MotoMiniPlanning::run()`:

```cpp
// After planning chunk, visualize trajectory
auto debug_node = std::make_shared<EnhancedDebugNode>();

// Set target cartesian poses
for (size_t i = 0; i < target_poses_.size(); ++i)
{
    debug_node->setTargetCartesianPose(ee_link_, target_poses_[i]);
}

// Add planned trajectory waypoints
for (const auto& state : res.traj)
{
    debug_node->addTrajectoryWaypoint(state.position, state.time);
}

// Joint callbacks get collision data
const auto& collision_info = debug_node->getCollisionDebugInfo();
```

---

## Visualization Details

### 1. Collision Box/Margin Visualization

**Marker Type**: `LINE_STRIP` + `TEXT_VIEW_FACING`

**Color Coding**:

- 🔴 Red: Collision (distance < 0% of threshold)
- 🟡 Yellow: Caution (50% of threshold)
- 🟢 Green: Safe (100% of threshold)

**Information**:

```
Link1_Name ↔ Link2_Name
(distance in cm)
```

**Example Output**:

```
gripper ↔ part
(2.50 cm)  # Yellow warning
```

---

### 2. Collision Gradient Visualization

**Marker Type**: `ARROW` (pointing away from collision)

**Purpose**: Shows direction of collision avoidance

- Points from contact point towards safety
- Length proportional to collision distance
- Color matches collision margin visualization

**Use Case**: Understand which direction links should move to avoid collision

---

### 3. Cartesian Error Visualization

**Marker Components**:

- AxisMarker (Red/Green/Blue) at target position
- AxisMarker (Red/Green/Blue) at current position
- ArrowMarker pointing from actual to desired

**Color Coding**:

- 🟢 Green Arrows: Small error (<5cm)
- 🟡 Yellow Arrows: Medium error
- 🔴 Red Arrows: Large error (>5cm)

**Use Case**: Monitor end-effector tracking during execution

---

### 4. Joint Trajectory Visualization

**Marker Type**: `LINE_STRIP` (green)

**Features**:

- Traces end-effector path through 3D space
- Waypoint-indexed for temporal tracking
- Useful for verifying planned trajectory smoothness

**Use Case**: Visualize complete path before execution

---

### 5. Kinematic Error Visualization

**Marker Type**: `ARROW` (magenta/purple)

**Purpose**: Specific to end-effector tracking errors

- Highlights remaining convergence issues
- Separate from cartesian error for clarity
- Used during online SQP optimization

---

## SQP Optimization Debugging

### Integration with motomini_planning_run.cpp

Add SQP callback monitoring for real-time optimization visualization:

```cpp
#include "robot_planning/sqp_debug_callback.hpp"

// During online thread setup in motomini_planning_run.cpp
auto debug_callback = std::make_shared<Vinhtesseract_examples::SQPDebugCallback>();

// Register iteration callback
debug_callback->setOnIterationCallback(
    [&](const Vinhtesseract_examples::SQPIterationMetrics& metrics)
    {
        CONSOLE_BRIDGE_logInform(
            "SQP Iter %d: Cost=%.4f | Collision Viol=%d | Cart Viol=%d",
            metrics.iteration, metrics.total_cost,
            metrics.collision_violations, metrics.cartesian_violations);
    });

// Register collision callback
debug_callback->setOnCollisionCallback(
    [&](const Vinhtesseract_examples::SQPCollisionDebugData& data)
    {
        CONSOLE_BRIDGE_logWarn("Collision violations detected: %d",
            data.total_violations);
    });

// In the solver loop
// ... solver.stepSQPSolver();
// SQPIterationMetrics metrics = extractMetricsFromSolver();
// debug_callback->updateMetrics(metrics);
```

---

## RViz Configuration

### Add Debug Markers Display

1. In RViz: `Add` → `By Topic`
2. Select `/debug_markers` (MarkerArray)
3. Configure visualization options:

```yaml
MarkerArray:
  Namespaces:
    - collision_margin: Show/Hide collision distance lines
    - collision_gradient: Show/Hide avoidance direction arrows
    - collision_text: Show/Hide distance labels
    - trajectory: Show/Hide path animation
    - cartesian_error: Show/Hide target/actual frames
    - kinematic_error: Show/Hide tracking errors
```

### Suggested RViz Layout

```
┌─────────────────────────────────────────┐
│          3D Robot Visualization         │
│                                         │
│  • Robot Model (with links)             │
│  • Planning Scene (collision objects)   │
│  • Planned Trajectory (green line)      │
│  • Collision Warnings (red/yellow)      │
│  • Cartesian Targets (axis frames)      │
│                                         │
└─────────────────────────────────────────┘
```

---

## Troubleshooting

### Markers Not Appearing

**Issue**: Debug markers don't show in RViz
**Solutions**:

1. Check frame_id matches TF tree
2. Verify marker topic is subscribed: `ros2 topic echo /debug_markers`
3. Check RViz MarkerArray configuration
4. Ensure joint_states are being published

### Performance Issues

**Issue**: Debug node causing slowdown
**Solutions**:

1. Disable unused visualization features via parameters
2. Increase marker publication throttle
3. Reduce trajectory waypoint density
4. Use `clear()` frequently to limit marker accumulation

### Collision Visualization Wrong

**Issue**: Collision margins not matching expectations
**Solutions**:

1. Verify collision_threshold parameter matches planner config
2. Check Allowed Collision Matrix (SRDF) is properly loaded
3. Confirm link transform updates are correct
4. Validate contact manager configuration

---

## Advanced: Custom Debug Extensions

### Adding New Marker Types

Extend `EnhancedDebugNode::jointStateCallback()`:

```cpp
// Add new visualization function
int visualizeCustomFeature(
    visualization_msgs::msg::MarkerArray& marker_array,
    const CustomData& data,
    int id_counter,
    const std::string& frame_id)
{
    // Implement custom marker logic here
    visualization_msgs::msg::Marker marker;
    // ... configure marker ...
    marker_array.markers.push_back(marker);
    return 1;  // Return number of markers added
}

// Call from jointStateCallback
id_counter += visualizeCustomFeature(
    marker_array, custom_data, id_counter, frame_id_);
```

### SQP Metrics Export

Use `SQPTrajectoryAnalyzer` for metrics collection:

```cpp
Vinhtesseract_examples::SQPIterationMetrics metrics;
metrics.iteration = iter;
metrics.collision_cost = computed_collision_cost;

// Analyze trajectory
Vinhtesseract_examples::SQPTrajectoryAnalyzer::analyzeTrajectory(
    trajectory, joint_names, metrics);

// Check for discontinuities
int discontinuities =
    Vinhtesseract_examples::SQPTrajectoryAnalyzer::detectDiscontinuities(
        trajectory);

debug_callback->updateMetrics(metrics);
```

---

## Performance Metrics

| Feature         | CPU Impact | GPU Impact | Memory                             |
| --------------- | ---------- | ---------- | ---------------------------------- |
| Collision Viz   | ~5ms       | Light      | ~1MB                               |
| Cartesian Error | ~2ms       | Light      | ~500KB                             |
| Trajectory Viz  | ~3ms       | Medium     | ~2MB (scales with trajectory size) |
| Kinematic Error | ~1ms       | Light      | ~100KB                             |
| SQP Callback    | <1ms       | None       | ~100KB                             |

**Total**: ~12ms per cycle (at 100Hz = negligible overhead)

---

## References

- Tesseract Visualization API: `/tesseract_visualization/visualization.h`
- Marker Types: `/tesseract_visualization/markers/`
- SQP Callbacks: `/trajopt_sqp/callbacks/`
- Collision API: `/tesseract_collision/`

---

## See Also

- [OnlineCollisionDebugger](./collision_debugger_node.cpp) - Real-time collision checking
- [MotoMiniPlanning](./motomini_planning_run.cpp) - Integration points for trajectory planning
- [MotominoPlanning Header](../../include/robot_planning/motomini_planning.h) - Main API
