# Thesis Project — Robotic Pick-and-Place with Tesseract Motion Planning

> **Author:** Vinh Bui Quang Vinh · HCMUT · `vinh.buiquangvinh@hcmut.edu.vn`
> **ROS Distro:** ROS 2 Humble (Ubuntu 22.04 LTS)
> **Tesseract Suite:** v0.33.0

A ROS 2 workspace for a robotic pick-and-place thesis using the **Tesseract** motion-planning framework with a Motoman MotoMini robot arm and a custom 2-axis gantry. The system performs 3-D mesh processing, collision-aware trajectory planning, robot simulation and exposes a PyQt6 GUI operator interface.

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Package Overview](#package-overview)
3. [System Requirements](#system-requirements)
4. [Step 1 — Install ROS 2 Humble](#step-1--install-ros-2-humble)
5. [Step 2 — Install System Dependencies](#step-2--install-system-dependencies)
6. [Step 3 — Clone This Repository](#step-3--clone-this-repository)
7. [Step 4 — Clone Tesseract Dependencies (vcs)](#step-4--clone-tesseract-dependencies-vcs)
8. [Step 5 — rosdep Install](#step-5--rosdep-install)
9. [Step 6 — Python GUI Virtual Environment](#step-6--python-gui-virtual-environment)
10. [Step 7 — Build the Workspace](#step-7--build-the-workspace)
11. [Step 8 — Running the System](#step-8--running-the-system)
12. [Troubleshooting](#troubleshooting)
13. [Remote Repositories](#remote-repositories)

---

## Project Structure

```
vinh_ws/
└── src/
    └── thesis_project/              ← this repository
        ├── dependencies.repos       ← VCS manifest (Tesseract vendor repos)
        ├── gantry_controller/       # C++ ROS 2 node  — Modbus TCP gantry driver
        ├── main_gui/                # Python / PyQt6  — operator GUI
        ├── mesh_processing/         # C++ ROS 2 node  — point-cloud & mesh pipeline
        ├── motomini/                # URDF/xacro      — MotoMini robot description
        ├── robot_model/             # URDF/xacro      — combined workcell model
        ├── robot_planning/          # C++ ROS 2 node  — Tesseract motion planner
        └── tesseract/               ← NOT in this repo — cloned via vcs (see Step 4)
            ├── tesseract/
            ├── tesseract_planning/
            ├── tesseract_qt/
            ├── trajopt/
            ├── boost_plugin_loader/
            ├── descartes_light/
            ├── ifopt/
            ├── opw_kinematics/
            ├── ros_industrial_cmake_boilerplate/
            ├── taskflow/
            └── src/
                └── tesseract_ros2/  # ROS 2 wrappers (msgs, rosutils, rviz, monitoring)
```

> **`tesseract/`** is a collection of third-party vendor libraries excluded from version
> control. The `dependencies.repos` file pins every repo to the exact version used.

---

## Package Overview

| Package             | Lang           | Description                                                                                                  |
| ------------------- | -------------- | ------------------------------------------------------------------------------------------------------------ |
| `gantry_controller` | C++            | ROS 2 hardware bridge for the 2-axis gantry over **Modbus TCP** (`rclcpp`, `sensor_msgs`, `trajectory_msgs`) |
| `main_gui`          | Python / PyQt6 | Operator GUI — calibration, mesh loading, pick-pose editing, trajectory approval                             |
| `mesh_processing`   | C++            | Point-cloud filtering, surface reconstruction, pick-point extraction (Open3D, PCL)                           |
| `motomini`          | URDF/xacro     | Motoman MotoMini robot description + Gazebo / RViz launch files                                              |
| `robot_model`       | URDF/xacro     | Shared robot model combining arm + gantry workcell                                                           |
| `robot_planning`    | C++            | **Tesseract**-based motion planner (TrajOpt / Descartes) exposed as a ROS 2 action server                    |

### Tesseract Vendor Library Map (v0.33.0)

| Repo                                                                                                     | Version | Provides                                                                                                                                                                                                                                                      |
| -------------------------------------------------------------------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`tesseract`](https://github.com/tesseract-robotics/tesseract)                                           | 0.33.0  | Core: `tesseract_common`, `tesseract_collision`, `tesseract_environment`, `tesseract_kinematics`, `tesseract_scene_graph`, `tesseract_srdf`, `tesseract_urdf`, `tesseract_visualization`, `tesseract_geometry`, `tesseract_state_solver`, `tesseract_support` |
| [`tesseract_planning`](https://github.com/tesseract-robotics/tesseract_planning)                         | 0.33.0  | `tesseract_command_language`, `tesseract_motion_planners`, `tesseract_task_composer`, `tesseract_time_parameterization`, `tesseract_examples`                                                                                                                 |
| [`tesseract_qt`](https://github.com/tesseract-robotics/tesseract_qt)                                     | 0.33.0  | Qt5-based visualisation widgets                                                                                                                                                                                                                               |
| [`tesseract_ros2`](https://github.com/tesseract-robotics/tesseract_ros2)                                 | 0.33.0  | ROS 2 wrappers: `tesseract_msgs`, `tesseract_rosutils`, `tesseract_rviz`, `tesseract_monitoring`, `tesseract_planning_server`, `tesseract_qt_ros`, `tesseract_ros_examples`                                                                                   |
| [`trajopt`](https://github.com/tesseract-robotics/trajopt)                                               | 0.33.0  | `trajopt`, `trajopt_ifopt`, `trajopt_sco`, `trajopt_sqp`, `trajopt_common`                                                                                                                                                                                    |
| [`boost_plugin_loader`](https://github.com/tesseract-robotics/boost_plugin_loader)                       | 0.4.2   | Plugin-loader utility                                                                                                                                                                                                                                         |
| [`descartes_light`](https://github.com/swri-robotics/descartes_light)                                    | 0.4.9   | Descartes Cartesian planner                                                                                                                                                                                                                                   |
| [`opw_kinematics`](https://github.com/Jmeyer1292/opw_kinematics)                                         | 0.5.2   | Analytical IK for 6-DOF industrial robots                                                                                                                                                                                                                     |
| [`ifopt`](https://github.com/ethz-adrl/ifopt)                                                            | 2.1.4   | Interface to Ipopt / OSQP NLP solvers                                                                                                                                                                                                                         |
| [`taskflow`](https://github.com/taskflow/taskflow)                                                       | v3.5.0  | Parallel task-graph execution                                                                                                                                                                                                                                 |
| [`ros_industrial_cmake_boilerplate`](https://github.com/ros-industrial/ros_industrial_cmake_boilerplate) | 0.7.4   | Shared CMake macros                                                                                                                                                                                                                                           |

---

## System Requirements

| Item   | Version                  |
| ------ | ------------------------ |
| OS     | Ubuntu 22.04 LTS (Jammy) |
| ROS 2  | Humble Hawksbill         |
| CMake  | ≥ 3.22                   |
| GCC    | 11+ (default on 22.04)   |
| Python | 3.10 (system)            |

---

## Step 1 — Install ROS 2 Humble

Skip if already installed.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  python3-venv

# Source ROS 2 (also add to ~/.bashrc)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## Step 2 — Install System Dependencies

These are apt packages required by the Tesseract suite and the project packages.

### 2a — Core build tools & Eigen / Boost

```bash
sudo apt install -y \
  build-essential \
  cmake \
  libeigen3-dev \
  libboost-all-dev \
  libconsole-bridge-dev \
  libomp-dev \
  libgtest-dev \
  libtinyxml2-dev \
  libyaml-cpp-dev \
  libjsoncpp-dev \
  libgraphviz-dev \
  libpcl-all-dev \
  libassimp-dev
```

### 2b — Collision & kinematics backends

```bash
sudo apt install -y \
  libfcl-dev \
  liboctomap-dev \
  libbullet-dev \
  liborocos-kdl-dev \
  libompl-dev
```

### 2c — Optimisation solvers (TrajOpt / ifopt)

```bash
sudo apt install -y \
  coinor-libipopt-dev \
  libqpoases-dev

# OSQP and its Eigen wrapper (required by trajopt_sqp)
sudo apt install -y ros-humble-osqp-vendor ros-humble-osqp-eigen-vendor || \
  pip3 install osqp  # fallback
```

### 2d — Qt5 + visualisation (tesseract_qt)

```bash
sudo apt install -y \
  qtbase5-dev \
  qttools5-dev \
  libqt5svg5-dev \
  libqwt-qt5-dev \
  qt5-qmake
```

### 2e — Velocity profiling (ruckig)

```bash
sudo apt install -y ros-humble-ruckig-vendor 2>/dev/null || \
  pip3 install ruckig
```

### 2f — ROS 2 packages (non-desktop extras)

```bash
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-visualization-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  ros-humble-resource-retriever \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-rviz-rendering \
  ros-humble-rviz-ogre-vendor \
  ros-humble-octomap-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui
```

### 2g — PyQt6 runtime requirement

```bash
sudo apt install -y \
  libgl1-mesa-glx \
  libglib2.0-0 \
  libegl1-mesa
```

---

## Step 3 — Clone This Repository

```bash
mkdir -p ~/vinh_ws/src
cd ~/vinh_ws/src
git clone https://github.com/vinhbui720/thesis_project.git
```

---

## Step 4 — Clone Tesseract Dependencies (vcs)

This project uses **vcstool** (`vcs`) to manage all third-party Tesseract vendor repos.
The `dependencies.repos` file pins every dependency to the exact version tested.

```bash
# Make sure vcstool is installed (done in Step 1, or install separately)
sudo apt install -y python3-vcstool

# Navigate into the thesis_project directory
cd ~/vinh_ws/src/thesis_project

# Import all Tesseract dependencies in one command
# This creates ~/vinh_ws/src/thesis_project/tesseract/* automatically
vcs import < dependencies.repos
```

> This clones the following repos (all relative to `thesis_project/`):
>
> | Cloned path                                  | Repo                                            | Version |
> | -------------------------------------------- | ----------------------------------------------- | ------- |
> | `tesseract/tesseract`                        | tesseract-robotics/tesseract                    | 0.33.0  |
> | `tesseract/tesseract_planning`               | tesseract-robotics/tesseract_planning           | 0.33.0  |
> | `tesseract/tesseract_qt`                     | tesseract-robotics/tesseract_qt                 | 0.33.0  |
> | `tesseract/trajopt`                          | tesseract-robotics/trajopt                      | 0.33.0  |
> | `tesseract/src/tesseract_ros2`               | tesseract-robotics/tesseract_ros2               | 0.33.0  |
> | `tesseract/boost_plugin_loader`              | tesseract-robotics/boost_plugin_loader          | 0.4.2   |
> | `tesseract/descartes_light`                  | swri-robotics/descartes_light                   | 0.4.9   |
> | `tesseract/opw_kinematics`                   | Jmeyer1292/opw_kinematics                       | 0.5.2   |
> | `tesseract/ifopt`                            | ethz-adrl/ifopt                                 | 2.1.4   |
> | `tesseract/taskflow`                         | taskflow/taskflow                               | v3.5.0  |
> | `tesseract/ros_industrial_cmake_boilerplate` | ros-industrial/ros_industrial_cmake_boilerplate | 0.7.4   |

### Update to Latest Pinned Versions

If you already have the repos cloned and want to sync them to the pinned versions:

```bash
cd ~/vinh_ws/src/thesis_project
vcs pull < dependencies.repos
```

---

## Step 5 — rosdep Install

```bash
# Initialise rosdep (first time only)
sudo rosdep init 2>/dev/null || true
rosdep update

# Install all remaining system deps declared in package.xml files
cd ~/vinh_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 6 — Python GUI Virtual Environment

The `main_gui` package is a **standalone PyQt6 app** that runs outside colcon.
Use a virtual environment to isolate its Python dependencies.

```bash
cd ~/vinh_ws/src/thesis_project/main_gui

# Create venv (named .venv — already in .gitignore)
python3 -m venv .venv

# Activate
source .venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Deactivate when done (run_gui.sh handles activation at launch)
deactivate
```

**Dependencies installed** (`requirements.txt`):

| Package          | Purpose                           |
| ---------------- | --------------------------------- |
| `PyQt6==6.6.1`   | GUI framework                     |
| `open3d==0.18.0` | 3-D point-cloud & mesh processing |
| `numpy>=1.24`    | Numerical arrays                  |
| `scipy`          | Scientific computing              |
| `trimesh`        | Mesh loading & manipulation       |
| `pyvista`        | 3-D visualisation                 |
| `pyqtgraph`      | Embedded plot widgets             |
| `pyyaml`         | YAML config parsing               |
| `psutil`         | System resource monitoring        |

> **Do not** `pip install rclpy` — use the system ROS 2 Python packages.
> `run_gui.sh` adds the ROS 2 Python path automatically.

---

## Step 7 — Build the Workspace

```bash
cd ~/vinh_ws

# Full build (Release mode — recommended)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# OR — build only the project packages first to verify them quickly
colcon build \
  --packages-select \
    gantry_controller \
    mesh_processing \
    motomini \
    robot_model \
    robot_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the install overlay
source ~/vinh_ws/install/setup.bash
# Add to ~/.bashrc for convenience:
echo "source ~/vinh_ws/install/setup.bash" >> ~/.bashrc
```

> **Build time:** First full build (including Tesseract) typically takes **15–30 minutes**
> depending on CPU core count. Use `--parallel-workers $(nproc)` to maximise parallelism.

---

## Step 8 — Running the System

### Visualise robot in RViz only

```bash
ros2 launch motomini description.launch.py
```

### Full pick-and-place pipeline

```bash
# Terminal 1 — Motion planner (Tesseract)
ros2 launch robot_planning robot_planning.launch.py

# Terminal 2 — Gantry controller
ros2 launch gantry_controller gantry_controller.launch.py

# Terminal 3 — Mesh processing
ros2 launch mesh_processing mesh_processing.launch.py

# Terminal 4 — PyQt6 operator GUI
cd ~/vinh_ws/src/thesis_project/main_gui
bash run_gui.sh
```

---

## Troubleshooting

### `tesseract_*` packages not found during build

Ensure you ran **Step 4** (`vcs import`) and **Step 5** (`rosdep install`) completely.

```bash
cd ~/vinh_ws/src/thesis_project
vcs import < dependencies.repos
cd ~/vinh_ws && rosdep install --from-paths src --ignore-src -r -y
```

### PyQt6 `libEGL` / OpenGL error

```bash
sudo apt install -y libegl1-mesa libgl1-mesa-glx
```

### `coinor-libipopt-dev` not found

```bash
sudo apt install -y coinor-libipopt-dev
# If unavailable, build Ipopt from source or use the OSQP solver branch
```

### `qpoases` not found by CMake

```bash
sudo apt install -y ros-humble-qpoases-vendor 2>/dev/null || \
  sudo apt install -y libqpoases-dev
```

### Colcon build fails: `Eigen3 not found`

```bash
sudo apt install -y libeigen3-dev
# Force cmake hint if needed:
colcon build --cmake-args -DEigen3_DIR=/usr/lib/cmake/eigen3
```

### Modbus TCP connection refused (gantry)

Verify the gantry PLC IP/port in `gantry_controller/config/params.yaml` and that the
device is reachable on the network (`ping <PLC_IP>`).

### `open3d` import error inside a ROS 2 node

`open3d` is only for the **GUI** (installed in the venv). It must not be imported
within any `colcon`-built C++ or Python node.

---

PYTHONPATH=/home/vinbui/vinh_ws/src

## Remote Repositories

| Remote            | URL                                          |
| ----------------- | -------------------------------------------- |
| `origin`          | https://github.com/vinhbui720/thesis_project |
| `upstream` (team) | https://github.com/Vinhktn720/thesis_project |

```bash
# Sync with team upstream
git fetch upstream
git merge upstream/master
```

---

_HCMUT Robotics Lab · April 2026_
