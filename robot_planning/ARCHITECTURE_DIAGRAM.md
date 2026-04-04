# Enhanced Debug System - Visual Architecture & Quick Start

## 🎯 System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Ecosystem                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  /joint_states  (sensor_msgs/JointState)                           │
│      ↓                                                              │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │     EnhancedDebugNode (Main Processing Node)              │   │
│  │  ┌─────────────────────────────────────────────────────┐  │   │
│  │  │ jointStateCallback()                                │  │   │
│  │  │ ┌──────────────────────────────────┐               │  │   │
│  │  │ │ 1. Update Environment State      │               │  │   │
│  │  │ │    └─→ setState(joint_names)     │               │  │   │
│  │  │ └──────────────────────────────────┘               │  │   │
│  │  │ ┌──────────────────────────────────┐               │  │   │
│  │  │ │ 2. Collision Detection           │               │  │   │
│  │  │ │    └─→ contactTest()            │               │  │   │
│  │  │ │    ✅ Collision Margin           │               │  │   │
│  │  │ │    ✅ Collision Gradient         │               │  │   │
│  │  │ └──────────────────────────────────┘               │  │   │
│  │  │ ┌──────────────────────────────────┐               │  │   │
│  │  │ │ 3. FK Calculations               │               │  │   │
│  │  │ │    └─→ calcFwdKin()              │               │  │   │
│  │  │ │    ✅ Cartesian Error            │               │  │   │
│  │  │ │    ✅ Kinematic Error            │               │  │   │
│  │  │ └──────────────────────────────────┘               │  │   │
│  │  │ ┌──────────────────────────────────┐               │  │   │
│  │  │ │ 4. Trajectory Animation          │               │  │   │
│  │  │ │    └─→ visualizeTrajectory()     │               │  │   │
│  │  │ │    ✅ Joint Trajectory           │               │  │   │
│  │  │ └──────────────────────────────────┘               │  │   │
│  │  │ ┌──────────────────────────────────┐               │  │   │
│  │  │ │ 5. Generate Markers              │               │  │   │
│  │  │ │    └─→ MarkerArray               │               │  │   │
│  │  │ └──────────────────────────────────┘               │  │   │
│  │  └─────────────────────────────────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────┘   │
│      ↓                                                              │
│  /debug_markers (visualization_msgs/MarkerArray)                   │
│      ↓                                                              │
│  ┌──────────────┐                                                  │
│  │    RViz      │                                                  │
│  │              │                                                  │
│  │  • Red Lines (Collisions)                                      │
│  │  • Green Lines (Trajectory)                                    │
│  │  • Colored Arrows (Errors)                                     │
│  │  • Frame Axes (Cartesian)                                      │
│  │                                                                 │
│  └──────────────┘                                                  │
│                                                                     │
│  ╔══════════════════════════════════════════════════════════╗     │
│  ║         Optional: SQP Optimization Monitoring           ║     │
│  ║  ┌────────────────────────────────────────────────────┐ ║     │
│  ║  │ SQPDebugCallback                                   │ ║     │
│  ║  │  • on_iteration_cb_  → Metrics per iteration       │ ║     │
│  ║  │  • on_collision_cb_  → Violation detection         │ ║     │
│  ║  │  • on_cartesian_cb_  → Constraint satisfaction     │ ║     │
│  ║  └────────────────────────────────────────────────────┘ ║     │
│  ╚══════════════════════════════════════════════════════════╝     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 📊 Data Flow Diagram

```
Planning System
      ↓
  Trajectory
      ↓
┌─────────────────────┐
│  add to debug_node  │ ← setTargetCartesianPose()
│                     │ ← addTrajectoryWaypoint()
│  cartesian_targets_ │
│  trajectory_waypts_ │
└─────────────────────┘
      ↓
Joint States Published
      ↓
/joint_states
      ↓
EnhancedDebugNode
      ↓
┌──────────────────────────────────┐
│ Compute All Visualizations       │
├──────────────────────────────────┤
│ • Contact results                │
│ • FK positions                   │
│ • Error calculations             │
│ • Trajectory points              │
│ • Marker generation              │
└──────────────────────────────────┘
      ↓
/debug_markers
      ↓
RViz
      ↓
┌──────────────────────────────────┐
│   3D Visualization               │
│   ✅ Collision lines + text      │
│   ✅ Gradient arrows             │
│   ✅ Frame axes                  │
│   ✅ Error arrows                │
│   ✅ Trajectory path             │
└──────────────────────────────────┘
```

---

## 🚀 Quick Start Flow

```
STEP 1: Installation
├─ Build:  colcon build --packages-select robot_planning
└─ Success: Module installed

STEP 2: Launch Debug Node
├─ Command: ros2 launch robot_planning enhanced_debug_launch.py
├─ Node starts
├─ Subscribes to /joint_states
└─ Publishes to /debug_markers

STEP 3: Configure RViz
├─ Open RViz: ros2 run rviz2 rviz2
├─ Add Display: MarkerArray
├─ Select Topic: /debug_markers
├─ Set Frame: world
└─ View: 3D robot with markers

STEP 4: See Results
├─ Move robot via joint_states
├─ Collision warnings appear (RED/YELLOW)
├─ Trajectory path animates (GREEN)
├─ Error vectors show (ARROWS)
└─ Success! System working
```

---

## 🔄 Component Interaction Diagram

```
┌─────────────────┐
│  Your Planning  │
│    Code         │
└────────┬────────┘
         │
         │ Reports trajectory
         ↓
┌─────────────────────────────┐
│ EnhancedDebugNode::API      │
│ - setTargetCartesianPose()  │
│ - addTrajectoryWaypoint()   │
│ - getCollisionDebugInfo()   │
└────────┬────────────────────┘
         │
         │ Stores targets &
         │ waypoints
         ↓
┌─────────────────────────────┐
│ Private Data Structures     │
│ - cartesian_targets_        │
│ - trajectory_waypoints_     │
│ - collision_debug_info_     │
└────────┬────────────────────┘
         │
         │ Updated by
         ↓
/joint_states  (from robot drivers)
         │
         ↓
jointStateCallback()
         │
         ├─→ Current State
         │   (determines FK,
         │    contact checks)
         │
         ├─→ Merge with targets
         │   (compute errors)
         │
         ├─→ Generate Markers
         │   (visualizations)
         │
         └─→ Publish
             /debug_markers
```

---

## 🎨 Visualization Flowchart

```
COLLISION DETECTION
    ↓
contactTest()
    ↓
FOR each contact pair:
    ├─ IS ALLOWED? (check ACM)
    │   └─ YES → Skip
    ├─ DISTANCE < THRESHOLD?
    │   ├─ NO → Skip
    │   └─ YES → Continue
    ├─ CREATE LINE MARKER
    │   ├─ Start: nearest_points[0]
    │   ├─ End: nearest_points[1]
    │   └─ Color: Based on distance ratio
    └─ CREATE TEXT MARKER
        └─ Label: "link1 ↔ link2 (distance cm)"

CARTESIAN ERROR VISUALIZATION
    ↓
FOR each target in cartesian_targets_:
    ├─ Calculate FK for link_name
    ├─ Get current position
    ├─ Create AXIS markers
    │   ├─ One at target (expected)
    │   └─ One at current (actual)
    ├─ Calculate error vector
    └─ Create ARROW marker
        └─ Points from current to target

TRAJECTORY ANIMATION
    ↓
FOR each waypoint in trajectory_waypoints_:
    ├─ Calculate FK for all waypoints
    └─ Create LINE_STRIP
        ├─ Color: Green
        └─ Points: All FK results
```

---

## 📈 Feature Activation Matrix

```
               │ Collision │ Cartesian │ Trajectory │ Kinematic │ Gradient
               │  Margin   │   Error   │   Viz      │   Error   │Advanced
────────────────┼───────────┼───────────┼────────────┼───────────┼─────────
Default Auto   │    ✅     │    ✅     │     ✅     │    ✅     │    ❌
────────────────┼───────────┼───────────┼────────────┼───────────┼─────────
Standalone     │    ✅     │    ✅     │     ✅     │    ✅     │    ✅*
Mode           │           │           │            │           │  (*param)
────────────────┼───────────┼───────────┼────────────┼───────────┼─────────
API-based      │    ✅     │  On-Req   │   On-Req   │    ✅     │    ⚠️
Integration    │           │  (param)  │   (param)  │           │  (manual)
────────────────┼───────────┼───────────┼────────────┼───────────┼─────────
SQP Callback   │    ✅     │    ✅     │     ✅     │    ✅     │    ✅
Mode           │  (auto)   │ (callback)│ (optional) │ (callback)│ (optimal)
```

---

## 🔧 Parameter Configuration Pyramid

```
                    ┌──────────────────┐
                    │  System Enabled  │
                    │ (base, always on)│
                    └────────┬─────────┘
                             │
                    ┌────────▼─────────┐
                    │ Collision Margin │
                    │      (default)   │
                    └────────┬─────────┘
                             │
           ┌─────────────────┼─────────────────┐
           │                 │                 │
    ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
    │  Cartesian  │  │ Trajectory  │  │ Kinematic   │
    │    Error    │  │     Viz     │  │    Error    │
    │  (optional) │  │  (optional) │  │  (optional) │
    └─────────────┘  └─────────────┘  └─────────────┘
           │                 │                 │
           └─────────────────┼─────────────────┘
                             │
                    ┌────────▼─────────┐
                    │ Advanced Feature │
                    │ Collision Gradient
                    │  (enable:false)  │
                    └──────────────────┘
```

---

## 💾 Data Structure Relationships

```
EnhancedDebugNode
├── Environment
│   ├── env_
│   ├── contact_manager_
│   ├── manipulator_
│   └── state_solver_
│
├── Debug Data (Protected by mutex)
│   ├── cartesian_targets_
│   │   └── CartesianTarget
│   │       ├── expected_pose
│   │       ├── ee_position
│   │       └── position_error
│   │
│   ├── trajectory_waypoints_
│   │   └── TrajectoryWaypoint
│   │       ├── joint_positions
│   │       └── time
│   │
│   └── collision_debug_info_
│       └── CollisionDebugInfo
│           ├── link1, link2
│           ├── distance
│           ├── margin
│           └── nearest_pt1/pt2
│
└── ROS 2 Components
    ├── joint_sub_
    └── marker_pub_
```

---

## 🎯 Usage Scenarios

### Scenario 1: Real-Time Monitoring

```
User launches debug node
    ↓ (automatic)
Subscribes to /joint_states
    ↓ (every 100ms)
Processes joint state
    ↓
Detects collisions
    ↓
Publishes markers
    ↓
RViz displays warnings
    ↓
User sees real-time feedback
```

### Scenario 2: Planning Integration

```
Plan trajectory
    ↓
Call: addTrajectoryWaypoint(...) for each point
    ↓
Call: setTargetCartesianPose(...)
    ↓
Execute robot
    ↓
Debug node visualization shows:
  • Target frame
  • Current frame
  • Error arrow
  • Trajectory path
  • Collision warnings
```

### Scenario 3: SQP Debugging

```
Start optimization
    ↓
Register SQPDebugCallback
    ↓
Each iteration:
  • updateMetrics()
  • onIterationCallback()
  • updateCollisionData()
  • onCollisionCallback()
    ↓
Real-time metrics printed
    ↓
See convergence progress
```

---

## 📊 Timeline: Feature Integration

```
Week 1: Initial Implementation
├─ Enhanced debug node structure
├─ Collision visualization
└─ Testing

Week 2: Advanced Features
├─ Cartesian error tracking
├─ Trajectory animation
└─ Kinematic error

Week 3: Integration
├─ SQP callback framework
├─ Launch files
└─ API refinement

Week 4: Documentation
├─ User guides
├─ Code examples
└─ Quick reference

Result: Production-ready system
```

---

## ✅ Verification Checklist

- [x] 5 visualization features working
- [x] Standalone node operational
- [x] API methods functional
- [x] SQP callback framework complete
- [x] Launch files configured
- [x] Documentation comprehensive
- [x] Code examples provided
- [x] Performance optimized
- [x] Error handling implemented
- [x] Thread safety verified
