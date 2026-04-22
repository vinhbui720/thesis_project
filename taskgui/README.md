# TaskGUI: Dual-Robot Task Sequence Manager

This package provides a modern graphical interface to record, edit, and playback task sequences for a dual-robot ROS 2 system.

## Features
- **Live Monitor**: Visual status of Robot 1 and Robot 2 poses and ROS parameters.
- **Recording**: Captured waypoints, service calls, and parameter updates with millisecond precision.
- **Editing**: Built-in tabular editor to adjust timestamps and coordinates.
- **Playback**: Automated execution engine that respects recorded timing.

## Installation & Running

### 1. Environment Setup
The GUI is designed to run using the project virtual environment.

```bash
# From the workspace root
source .venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(pwd)/src/thesis_project/taskgui
```

### 2. Run the GUI
```bash
python3 src/thesis_project/taskgui/main.py
```

### 3. File Structure
- `core/`: ROS handling and Data processing logic.
- `ui/`: PyQt6 layout and styling.
- `models/`: Data schemas.

## YAML Format Example
Recorded tasks are saved in the following format:
```yaml
metadata:
  created_at: "2024-04-22 08:55:00"
  total_actions: 2
sequence:
  - timestamp: 0.0
    type: "waypoint"
    robot: "robot1"
    data: {x: 0.1, y: 0.2, z: 0.3}
  - timestamp: 1.5
    type: "param_update"
    node: "planner"
    param: "speed"
    value: 0.5
```

---
*Developed for Master's Thesis Project*
