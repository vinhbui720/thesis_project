# Thesis Project — Robotic Pick-and-Place with Motion Planning

> **Author:** Vinh Bui Quang Vinh · HCMUT · `vinh.buiquangvinh@hcmut.edu.vn`
> **ROS Distro:** ROS 2 Humble (Ubuntu 22.04)

A ROS 2 workspace for a robotic pick-and-place thesis using the **Tesseract** motion-planning framework with a Motoman MotoMini robot arm and a custom 2-axis gantry. The project includes 3-D mesh processing, trajectory planning, robot simulation and a PyQt6 GUI operator interface.

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [System Requirements](#system-requirements)
3. [Step 1 — System Dependencies](#step-1--system-dependencies)
4. [Step 2 — Tesseract (vendor libraries)](#step-2--tesseract-vendor-libraries)
5. [Step 3 — Clone This Repository](#step-3--clone-this-repository)
6. [Step 4 — Python GUI Environment](#step-4--python-gui-environment)
7. [Step 5 — Build the ROS 2 Workspace](#step-5--build-the-ros-2-workspace)
8. [Step 6 — Running the System](#step-6--running-the-system)
9. [Package Overview](#package-overview)
10. [Troubleshooting](#troubleshooting)

---

## Project Structure

```
vinh_ws/
└── src/
    └── thesis_project/          ← this repository
        ├── gantry_controller/   # C++ ROS 2 node – Modbus TCP gantry driver
        ├── main_gui/            # PyQt6 operator GUI (standalone Python app)
        ├── mesh_processing/     # C++ point-cloud & mesh processing node
        ├── motomini/            # MotoMini URDF/xacro + launch files
        ├── robot_model/         # Shared robot description package
        ├── robot_planning/      # C++ Tesseract motion-planning node
        └── tesseract/           # ← NOT in this repo (see Step 2 below)
```

> **`tesseract/`** is a collection of third-party vendor libraries
> (`tesseract`, `tesseract_planning`, `tesseract_qt`, `trajopt`, etc.).
> They are **excluded from version control** and must be cloned separately.

---

## System Requirements

| Requirement | Version / Notes |
|---|---|
| Ubuntu | 22.04 LTS (Jammy) |
| ROS 2 | Humble Hawksbill |
| CMake | ≥ 3.22 |
| GCC / Clang | GCC 11+ (default on 22.04) |
| Python | 3.10 (system, bundled with 22.04) |
| Eigen3 | ≥ 3.4 |
| Open3D | 0.18 (Python, via pip) |

---

## Step 1 — System Dependencies

```bash
# 1a. ROS 2 Humble (if not already installed)
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-venv

# Source ROS 2 (add to ~/.bashrc for convenience)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 1b. rosdep initialisation (first time only)
sudo rosdep init
rosdep update

# 1c. Eigen, Boost, console_bridge and other C++ deps
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libconsole-bridge-dev \
  libomp-dev \
  libgtest-dev \
  liburdfdom-dev \
  liboctomap-dev \
  libfcl-dev \
  libbullet-dev \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-visualization-msgs \
  ros-humble-trajectory-msgs

# 1d. PyQt6 system-level requirement
sudo apt install -y \
  libgl1-mesa-glx \
  libglib2.0-0
```

---

## Step 2 — Tesseract (vendor libraries)

These libraries are **not** included in this repo. Clone them into the
**same** `src/thesis_project/tesseract/` directory your workspace expects.

```bash
# Create the directory inside the workspace
mkdir -p ~/vinh_ws/src/thesis_project/tesseract
cd ~/vinh_ws/src/thesis_project/tesseract

# Clone each vendor repo
git clone --depth 1 https://github.com/tesseract-robotics/tesseract.git
git clone --depth 1 https://github.com/tesseract-robotics/tesseract_planning.git
git clone --depth 1 https://github.com/tesseract-robotics/tesseract_qt.git
git clone --depth 1 https://github.com/ros-industrial/trajopt.git
git clone --depth 1 https://github.com/ros-industrial/ifopt.git
git clone --depth 1 https://github.com/ros-industrial/descartes_light.git
git clone --depth 1 https://github.com/ros-industrial/opw_kinematics.git
git clone --depth 1 https://github.com/taskflow/taskflow.git
git clone --depth 1 https://github.com/Ros-Industrial/ros_industrial_cmake_boilerplate.git
git clone --depth 1 https://github.com/unr-arl/boost_plugin_loader.git
```

> **Tip:** You can also use `vcs` (the ROS VCS tool) with a `.repos` file.
> Ask the project maintainer for the pinned `.repos` file to reproduce the
> exact commit hashes used during development.

Install rosdep dependencies for all cloned packages:

```bash
cd ~/vinh_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 3 — Clone This Repository

```bash
cd ~/vinh_ws/src
git clone https://github.com/vinhbui720/thesis_project.git
```

If you already have the repo and just need to update:

```bash
cd ~/vinh_ws/src/thesis_project
git pull origin master
```

---

## Step 4 — Python GUI Environment

The `main_gui` package is a standalone PyQt6 application and uses its own
Python virtual environment to avoid conflicts with system packages.

```bash
cd ~/vinh_ws/src/thesis_project/main_gui

# Create a virtual environment (name it .venv — already in .gitignore)
python3 -m venv .venv

# Activate it
source .venv/bin/activate

# Upgrade pip and install all GUI dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Deactivate when done (the launch script handles activation automatically)
deactivate
```

> **Note on rclpy:** Do **not** install `rclpy` via pip. The ROS 2 system
> Python packages are used directly. The `run_gui.sh` script sets
> `PYTHONPATH` correctly so both can coexist.

---

## Step 5 — Build the ROS 2 Workspace

```bash
cd ~/vinh_ws

# Optional: build only the thesis packages first to catch dependency errors fast
colcon build --packages-select \
  gantry_controller \
  mesh_processing \
  motomini \
  robot_model \
  robot_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Full workspace build (includes Tesseract vendor libs – takes ~10–20 min)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the install overlay
source ~/vinh_ws/install/setup.bash
# Add to ~/.bashrc for convenience:
echo "source ~/vinh_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 6 — Running the System

### Launch the full system (planning + GUI + simulation)

```bash
# Terminal 1 — Robot planning node (Tesseract)
ros2 launch robot_planning robot_planning.launch.py

# Terminal 2 — Gantry controller node
ros2 launch gantry_controller gantry_controller.launch.py

# Terminal 3 — Mesh processing node
ros2 launch mesh_processing mesh_processing.launch.py

# Terminal 4 — PyQt6 GUI
cd ~/vinh_ws/src/thesis_project/main_gui
bash run_gui.sh
```

### Launch robot description / RViz only

```bash
ros2 launch motomini description.launch.py
```

---

## Package Overview

| Package | Language | Description |
|---|---|---|
| `gantry_controller` | C++ | ROS 2 hardware bridge for the 2-axis gantry over **Modbus TCP** |
| `main_gui` | Python / PyQt6 | Operator GUI — calibration, mesh loading, pick pose, trajectory approval |
| `mesh_processing` | C++ | Point-cloud filtering, surface reconstruction, pick-point extraction |
| `motomini` | URDF/xacro | Motoman MotoMini robot description + Gazebo/RViz launch files |
| `robot_model` | URDF/xacro | Shared robot model (arm + gantry combined workcell) |
| `robot_planning` | C++ | **Tesseract**-based motion planner (TrajOpt / Descartes) + ROS 2 action server |

---

## Troubleshooting

### `tesseract_*` packages not found during build

Ensure you have cloned all vendor repos into `tesseract/` (see Step 2) and
ran `rosdep install` before building.

### PyQt6 `libEGL` error on headless machine

```bash
sudo apt install -y libegl1-mesa
export QT_QPA_PLATFORM=offscreen   # for headless testing only
```

### `open3d` import error inside ROS 2 node

`open3d` must be installed in the **system** Python (or the venv activated
before running the GUI). It should never be imported inside a `colcon`-built
C++ node.

### colcon build fails with `Eigen3` not found

```bash
sudo apt install -y libeigen3-dev
# If cmake still cannot find it, set the hint:
colcon build --cmake-args -DEigen3_DIR=/usr/lib/cmake/eigen3
```

### Modbus TCP connection refused

Check that the gantry PLC IP is reachable and that the correct IP/port are
set in `gantry_controller/config/params.yaml`.

---

## Remote Repositories

| Remote | URL |
|---|---|
| `origin` (personal fork) | https://github.com/vinhbui720/thesis_project |
| `upstream` (team repo) | https://github.com/Vinhktn720/thesis_project |

```bash
# Sync with team upstream
git fetch upstream
git merge upstream/master
```

---

*Generated: April 2026 · HCMUT Robotics Lab*
