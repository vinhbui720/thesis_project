#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox
import subprocess
import yaml
import os

NODE_NAME = "/scene_preprocessor"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "..", "config", "scene_param.yaml")

# -------------------------
# ROS PARAM SETTER
# -------------------------
def set_ros_param(param_name, value):
    subprocess.Popen(
        ["ros2", "param", "set", NODE_NAME, param_name, str(value)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def create_slider_callback(param_name, slider):
    def callback(event):
        set_ros_param(param_name, slider.get())
    return callback

def create_checkbox_callback(param_name, var):
    def callback():
        set_ros_param(param_name, var.get())
    return callback

# -------------------------
# DEFAULT PARAMETERS
# -------------------------
default_params = {
    # Core
    "voxel_size": 0.005,
    "z_min": 0.2, "z_max": 1.0,
    "x_min": -0.5, "x_max": 0.5,
    "y_min": -0.5, "y_max": 0.5,
    "plane_thresh": 0.015,

    # Perpendicular plane
    "plane_axis_x": 0.0,
    "plane_axis_y": 0.0,
    "plane_axis_z": 1.0,
    "plane_eps_angle": 0.15,

    # SOR
    "use_sor": True,
    "sor_mean_k": 30,
    "sor_stddev": 1.0,

    # Normals
    "use_normals": True,
    "normal_radius": 0.02,
}

# Load YAML if exists
if os.path.exists(CONFIG_PATH):
    try:
        with open(CONFIG_PATH, "r") as f:
            yaml_data = yaml.safe_load(f)
            loaded = yaml_data.get("scene_preprocessor", {}).get("ros__parameters", {})
            default_params.update(loaded)
    except Exception as e:
        print("YAML load error:", e)

# -------------------------
# GUI SETUP
# -------------------------
root = tk.Tk()
root.title("ROS2 Scene Preprocessor Live Tuner")
root.geometry("450x900")

canvas = tk.Canvas(root)
scrollbar = tk.Scrollbar(root, orient="vertical", command=canvas.yview)
scrollable_frame = tk.Frame(canvas)

scrollable_frame.bind(
    "<Configure>",
    lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
)

canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
canvas.configure(yscrollcommand=scrollbar.set)

canvas.pack(side="left", fill="both", expand=True)
scrollbar.pack(side="right", fill="y")

# -------------------------
# Helper functions
# -------------------------
def make_slider(parent, label, key, from_val, to_val, res):
    tk.Label(parent, text=label).pack()
    slider = tk.Scale(parent, from_=from_val, to=to_val,
                      resolution=res, orient="horizontal")
    slider.set(default_params[key])
    slider.bind("<ButtonRelease-1>", create_slider_callback(key, slider))
    slider.pack(fill="x")
    return slider

def make_checkbox(parent, label, key):
    var = tk.BooleanVar()
    var.set(default_params[key])
    cb = tk.Checkbutton(parent, text=label,
                        variable=var,
                        command=create_checkbox_callback(key, var))
    cb.pack(anchor="w")
    return var

# -------------------------
# CORE PARAMETERS
# -------------------------
tk.Label(scrollable_frame, text="=== Downsampling ===", font=("Arial", 12, "bold")).pack()
voxel_slider = make_slider(scrollable_frame, "Voxel Size (m)", "voxel_size", 0.001, 0.05, 0.001)

tk.Label(scrollable_frame, text="=== ROI Filtering ===", font=("Arial", 12, "bold")).pack()
z_min_slider = make_slider(scrollable_frame, "Z Min", "z_min", 0.0, 2.0, 0.01)
z_max_slider = make_slider(scrollable_frame, "Z Max", "z_max", 0.0, 3.0, 0.01)
x_min_slider = make_slider(scrollable_frame, "X Min", "x_min", -1.0, 0.0, 0.01)
x_max_slider = make_slider(scrollable_frame, "X Max", "x_max", 0.0, 1.0, 0.01)
y_min_slider = make_slider(scrollable_frame, "Y Min", "y_min", -1.0, 0.0, 0.01)
y_max_slider = make_slider(scrollable_frame, "Y Max", "y_max", 0.0, 1.0, 0.01)

tk.Label(scrollable_frame, text="=== Plane Removal ===", font=("Arial", 12, "bold")).pack()
plane_slider = make_slider(scrollable_frame, "Plane Threshold", "plane_thresh", 0.001, 0.05, 0.001)
axis_x_slider = make_slider(scrollable_frame, "Plane Axis X", "plane_axis_x", -1.0, 1.0, 0.1)
axis_y_slider = make_slider(scrollable_frame, "Plane Axis Y", "plane_axis_y", -1.0, 1.0, 0.1)
axis_z_slider = make_slider(scrollable_frame, "Plane Axis Z", "plane_axis_z", -1.0, 1.0, 0.1)
eps_slider = make_slider(scrollable_frame, "Plane Epsilon Angle (rad)", "plane_eps_angle", 0.01, 0.5, 0.01)

tk.Label(scrollable_frame, text="=== Statistical Outlier Removal ===", font=("Arial", 12, "bold")).pack()
use_sor_var = make_checkbox(scrollable_frame, "Enable SOR", "use_sor")
sor_mean_slider = make_slider(scrollable_frame, "SOR Mean K", "sor_mean_k", 5, 100, 1)
sor_std_slider = make_slider(scrollable_frame, "SOR StdDev", "sor_stddev", 0.1, 3.0, 0.1)

tk.Label(scrollable_frame, text="=== Normals ===", font=("Arial", 12, "bold")).pack()
use_normals_var = make_checkbox(scrollable_frame, "Enable Normals", "use_normals")
normal_radius_slider = make_slider(scrollable_frame, "Normal Radius", "normal_radius", 0.005, 0.1, 0.005)

# -------------------------
# SAVE FUNCTION
# -------------------------
def save_to_yaml():
    data = {
        "scene_preprocessor": {
            "ros__parameters": {
                "voxel_size": voxel_slider.get(),
                "z_min": z_min_slider.get(),
                "z_max": z_max_slider.get(),
                "x_min": x_min_slider.get(),
                "x_max": x_max_slider.get(),
                "y_min": y_min_slider.get(),
                "y_max": y_max_slider.get(),
                "plane_thresh": plane_slider.get(),
                "plane_axis_x": axis_x_slider.get(),
                "plane_axis_y": axis_y_slider.get(),
                "plane_axis_z": axis_z_slider.get(),
                "plane_eps_angle": eps_slider.get(),
                "use_sor": use_sor_var.get(),
                "sor_mean_k": sor_mean_slider.get(),
                "sor_stddev": sor_std_slider.get(),
                "use_normals": use_normals_var.get(),
                "normal_radius": normal_radius_slider.get(),
            }
        }
    }

    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        yaml.dump(data, f, default_flow_style=False)

    messagebox.showinfo("Saved", f"Parameters saved to:\n{CONFIG_PATH}")

tk.Button(scrollable_frame, text="Save to scene_param.yaml",
          command=save_to_yaml, bg="green", fg="white").pack(pady=20)

root.mainloop()