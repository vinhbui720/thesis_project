#!/usr/bin/env python3
import tkinter as tk
from tkinter import filedialog, messagebox
import open3d as o3d
import numpy as np
import os
import csv
import copy

class STLPointCloudGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("3D STL Partial Scan Simulator")
        self.root.geometry("550x650")
        self.root.configure(padx=20, pady=20)

        self.mesh = None
        self.pcd = None
        self.picked_indices = []
        self.aligned_pcd = None
        # --- ADDED: Variable to store the aligned mesh ---
        self.aligned_mesh = None 
        # -----------------------------------------------
        self.coord_frame = None
        
        # New State Variables
        self.pick_point = None
        self.trajectory_points = []

        self.setup_ui()

    def setup_ui(self):
        # 1. File Selection Section
        tk.Label(self.root, text="Step 1: Load 3D Object (STL)", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 2))
        self.file_btn = tk.Button(self.root, text="Browse STL File", command=self.load_stl, width=20)
        self.file_btn.pack(anchor="w")
        self.file_label = tk.Label(self.root, text="No file selected", fg="gray")
        self.file_label.pack(anchor="w", pady=(0, 10))

        # 2. Point Picking Section
        tk.Label(self.root, text="Step 2: Define Ground & Origin", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 2))
        tk.Label(self.root, text="Shift + Left Click 1 point on the bottom surface.", fg="blue").pack(anchor="w")
        self.pick_btn = tk.Button(self.root, text="Open Visualizer to Pick Ground Point", command=self.pick_ground, state=tk.DISABLED, width=35)
        self.pick_btn.pack(anchor="w", pady=(2, 10))

        # 3. Camera Height Section
        tk.Label(self.root, text="Step 3: Camera Configuration", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 2))
        cam_frame = tk.Frame(self.root)
        cam_frame.pack(anchor="w", pady=(0, 10))
        tk.Label(cam_frame, text="Camera Z-Height: ").pack(side=tk.LEFT)
        self.height_entry = tk.Entry(cam_frame, width=10)
        self.height_entry.insert(0, "500") # Default value
        self.height_entry.pack(side=tk.LEFT)
        tk.Label(cam_frame, text=" mm").pack(side=tk.LEFT)

        # 4. Pick Start Point
        tk.Label(self.root, text="Step 4: Pick Start Point", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 2))
        tk.Label(self.root, text="Shift + Left Click exactly 1 target start point.", fg="blue").pack(anchor="w")
        self.start_pt_btn = tk.Button(self.root, text="Pick Start Point (Target)", command=self.pick_start_point, state=tk.DISABLED, width=35)
        self.start_pt_btn.pack(anchor="w", pady=(2, 10))

        # 5. Pick Trajectory
        tk.Label(self.root, text="Step 5: Pick Trajectory", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 2))
        tk.Label(self.root, text="Shift + Left Click multiple points for the path.", fg="blue").pack(anchor="w")
        self.traj_btn = tk.Button(self.root, text="Pick Trajectory Waypoints", command=self.pick_trajectory, state=tk.DISABLED, width=35)
        self.traj_btn.pack(anchor="w", pady=(2, 15))

        # 6. Generate Section
        tk.Label(self.root, text="Step 6: Visualize & Export", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 5))
        self.generate_btn = tk.Button(self.root, text="Show Result & Export Files (Meters)", command=self.generate_and_save, state=tk.DISABLED, width=40, bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.generate_btn.pack(anchor="w")

    def load_stl(self):
        file_path = filedialog.askopenfilename(filetypes=[("STL files", "*.stl"), ("All files", "*.*")])
        if not file_path: return

        try:
            self.mesh = o3d.io.read_triangle_mesh(file_path)
            self.mesh.compute_vertex_normals()
            self.pcd = self.mesh.sample_points_poisson_disk(100000)
            self.pcd.paint_uniform_color([0.7, 0.7, 0.7])

            self.file_label.config(text=os.path.basename(file_path), fg="black")
            self.pick_btn.config(state=tk.NORMAL)
            messagebox.showinfo("Success", "STL loaded successfully. Please proceed to Step 2.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load STL: {e}")

    def pick_ground(self):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick 1 point on the ground face (Shift + Click)", width=800, height=600)
        vis.add_geometry(self.pcd)
        vis.run()
        vis.destroy_window()

        self.picked_indices = vis.get_picked_points()
        if len(self.picked_indices) < 1:
            messagebox.showwarning("Warning", "No point picked.")
            return
        
        self.picked_indices = self.picked_indices[:1]
        self.align_object()
        self.start_pt_btn.config(state=tk.NORMAL)
        messagebox.showinfo("Aligned", "Object aligned to origin! Move to Step 4.")

    def align_object(self):
        idx = self.picked_indices[0]
        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        [k, idxs, _] = pcd_tree.search_knn_vector_3d(self.pcd.points[idx], 50)
        
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        normals = np.asarray(self.pcd.normals)[idxs]
        normal = np.mean(normals, axis=0)
        normal = normal / np.linalg.norm(normal)

        target_normal = np.array([0, 0, -1])
        v = np.cross(normal, target_normal)
        c = np.dot(normal, target_normal)
        s = np.linalg.norm(v)

        if s < 1e-6:
            R = np.eye(3) if c > 0 else np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        else:
            v_skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            R = np.eye(3) + v_skew + np.dot(v_skew, v_skew) * ((1 - c) / (s ** 2))

        self.aligned_pcd = self.pcd.rotate(R, center=(0, 0, 0))

        points = np.asarray(self.aligned_pcd.points)
        min_z = np.min(points[:, 2])
        bbox_height = np.max(points[:, 2]) - min_z
        threshold = bbox_height * 0.01 
        bottom_indices = np.where(points[:, 2] <= min_z + threshold)[0]
        bottom_points = points[bottom_indices]
        
        centroid_x = np.mean(bottom_points[:, 0])
        centroid_y = np.mean(bottom_points[:, 1])
        centroid_z = min_z

        translation = np.array([-centroid_x, -centroid_y, -centroid_z])
        self.aligned_pcd.translate(translation)

        # --- ADDED: Align the original STL mesh using the exact same transformations ---
        self.aligned_mesh = copy.deepcopy(self.mesh)
        self.aligned_mesh.rotate(R, center=(0, 0, 0))
        self.aligned_mesh.translate(translation)
        # -----------------------------------------------------------------------------

        size = np.linalg.norm(self.aligned_pcd.get_max_bound() - self.aligned_pcd.get_min_bound()) * 0.2
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])

    def pick_start_point(self):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick Start Point (Shift + Left Click 1 Point)", width=800, height=600)
        vis.add_geometry(self.aligned_pcd)
        vis.run()
        picked = vis.get_picked_points()
        vis.destroy_window()

        if len(picked) == 0:
            messagebox.showwarning("Warning", "No start point selected.")
            return

        points_np = np.asarray(self.aligned_pcd.points)
        self.pick_point = points_np[picked[-1]]
        
        self.traj_btn.config(state=tk.NORMAL)
        messagebox.showinfo("Success", "Start point saved! Move to Step 5.")

    def pick_trajectory(self):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick Trajectory Points (Shift + Left Click multiple)", width=800, height=600)
        vis.add_geometry(self.aligned_pcd)
        vis.run()
        picked_traj = vis.get_picked_points()
        vis.destroy_window()

        if len(picked_traj) < 2:
            messagebox.showwarning("Warning", "Please pick at least 2 trajectory points.")
            return

        points_np = np.asarray(self.aligned_pcd.points)
        self.trajectory_points = points_np[picked_traj]

        self.generate_btn.config(state=tk.NORMAL)
        messagebox.showinfo("Success", f"Saved {len(self.trajectory_points)} trajectory points! Ready to Export.")

    def generate_and_save(self):
        try:
            cam_height = float(self.height_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid camera height.")
            return

        # --- 1. Calculate Camera View (HPR) ---
        camera_pos = np.array([0.0, 0.0, cam_height])
        diameter = np.linalg.norm(self.aligned_pcd.get_max_bound() - self.aligned_pcd.get_min_bound())
        radius = diameter * 100 
        
        _, pt_map = self.aligned_pcd.hidden_point_removal(camera_pos.tolist(), radius)
        visible_pcd = self.aligned_pcd.select_by_index(pt_map)
        visible_pcd.paint_uniform_color([1.0, 0.5, 0.0]) # Orange

        # --- 2. Calculate Pose Matrix ---
        partial_points = np.asarray(visible_pcd.points)
        cloud_center = np.mean(partial_points, axis=0)
        
        z_axis = camera_pos - cloud_center
        z_axis = z_axis / np.linalg.norm(z_axis)

        global_y = np.array([0, 1, 0])
        if np.abs(np.dot(z_axis, global_y)) > 0.99:
            global_y = np.array([1, 0, 0])
            
        x_axis = np.cross(global_y, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        pose_matrix = np.eye(4)
        pose_matrix[:3, 0] = x_axis
        pose_matrix[:3, 1] = y_axis
        pose_matrix[:3, 2] = z_axis
        pose_matrix[:3, 3] = cloud_center

        pose_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=diameter*0.1)
        pose_frame.transform(pose_matrix)

        # --- 3. Build Final Visualization Geometries ---
        geometries = [self.aligned_pcd, visible_pcd, self.coord_frame, pose_frame]

        # Add Pick Point (Big Red Sphere)
        sphere_pick = o3d.geometry.TriangleMesh.create_sphere(radius=diameter * 0.02)
        sphere_pick.compute_vertex_normals()
        sphere_pick.paint_uniform_color([1, 0, 0])
        sphere_pick.translate(self.pick_point)
        geometries.append(sphere_pick)

        # Add Trajectory (Green Spheres & Lines)
        for pt in self.trajectory_points:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=diameter * 0.01)
            sphere.compute_vertex_normals()
            sphere.paint_uniform_color([0, 1, 0])
            sphere.translate(pt)
            geometries.append(sphere)

        lines = [[i, i + 1] for i in range(len(self.trajectory_points) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(self.trajectory_points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0, 1, 0])
        geometries.append(line_set)

        # Show visualization
        messagebox.showinfo("Review", "Showing final result. Close the 3D window to save all 4 files.")
        o3d.visualization.draw_geometries(geometries, window_name="Final Result: Mesh, Pose, Start Point & Trajectory")

        # --- 4. Export Sequence (Scale to Meters) ---
        save_base_path = filedialog.asksaveasfilename(title="Save As (Provide Base Name)", defaultextension="")
        if not save_base_path: return

        # File paths
        pcd_path = f"{save_base_path}_cam_view.pcd"
        traj_path = f"{save_base_path}_trajectory.csv"
        master_path = f"{save_base_path}_master.csv"
        # --- ADDED: Path for the new aligned STL ---
        stl_path = f"{save_base_path}_aligned.stl"
        # -------------------------------------------

        # 4a. Save PCD in meters
        pcd_to_save = copy.deepcopy(visible_pcd)
        pcd_to_save.scale(0.001, center=(0, 0, 0))
        o3d.io.write_point_cloud(pcd_path, pcd_to_save)

        # 4b. Save Trajectory in meters
        trajectory_meters = self.trajectory_points * 0.001
        with open(traj_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Waypoint X (m)", "Waypoint Y (m)", "Waypoint Z (m)"])
            for pt in trajectory_meters:
                writer.writerow([pt[0], pt[1], pt[2]])

        # 4c. Save Master CSV
        pick_point_meters = self.pick_point * 0.001
        
        pose_matrix_meters = pose_matrix.copy()
        pose_matrix_meters[:3, 3] = cloud_center / 1000.0 # Scale Translation

        with open(master_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            writer.writerow(["--- REQUIRED FILES ---"])
            # --- ADDED: Track the new STL file in the master document ---
            writer.writerow(["Aligned STL Path", stl_path])
            # ------------------------------------------------------------
            writer.writerow(["Camera View PCD Path", pcd_path])
            writer.writerow(["Trajectory Waypoints Path", traj_path])
            writer.writerow([])
            
            writer.writerow(["--- PICK POINT COORDINATES ---"])
            writer.writerow(["Pick X (m)", "Pick Y (m)", "Pick Z (m)"])
            writer.writerow([pick_point_meters[0], pick_point_meters[1], pick_point_meters[2]])
            writer.writerow([])
            
            writer.writerow(["--- CAMERA VIEW POSE MATRIX ---"])
            writer.writerow(["R11", "R12", "R13", "Tx (m)"])
            writer.writerow(["R21", "R22", "R23", "Ty (m)"])
            writer.writerow(["R31", "R32", "R33", "Tz (m)"])
            writer.writerow(["0", "0", "0", "1"])
            for row in pose_matrix_meters:
                writer.writerow(row)
                
        # --- ADDED: 4d. Save Aligned STL in meters ---
        mesh_to_save = copy.deepcopy(self.aligned_mesh)
        mesh_to_save.scale(0.001, center=(0, 0, 0))
        o3d.io.write_triangle_mesh(stl_path, mesh_to_save)
        # ---------------------------------------------
        
        # --- MODIFIED: Added the 4th file to the success message popup ---
        messagebox.showinfo("Success", f"Data exported successfully!\n\n1. {pcd_path}\n2. {traj_path}\n3. {master_path}\n4. {stl_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = STLPointCloudGUI(root)
    root.mainloop()