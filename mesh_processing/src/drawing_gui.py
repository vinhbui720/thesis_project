import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from scipy.spatial import cKDTree
import json
import tkinter as tk
from tkinter import messagebox
import os
import sys

# Get mesh_processing package share directory
try:
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory("mesh_processing")
    config_dir = os.path.join(pkg_share, "config")
except:
    config_dir = os.path.expanduser("~/vinh_ws/src/thesis_project/mesh_processing/config")


# ============================================
# INTERACTIVE GUI MANAGER
# ============================================

class SurfaceEditor:

    def __init__(self):
        self.pick_coord = None
        self.trajectory_coords = []
        self.picked_waypoints = []  # Store picked points for visualization
        self.paint_dist_threshold = None  # Will be set based on mesh scale
        
        # Load STL Mesh directly (no ROS subscription needed)
        self.mesh_path = "/home/vinbui/vinh_ws/src/thesis_project/mesh_processing/mesh/puzzel.stl"
        self.use_mesh = False
        self.mesh = None
        self.mesh_vertices = None
        self.cloud_np = None
        self.kdtree = None
        
        if not os.path.exists(self.mesh_path):
            print(f"[ERROR] Mesh file not found: {self.mesh_path}")
            sys.exit(1)
        
        try:
            print(f"Loading STL Mesh: {self.mesh_path}")
            self.mesh = o3d.io.read_triangle_mesh(self.mesh_path)
            
            # Apply 0.001 scale to match model_cloud from stl_to_cloud_node
            self.mesh.scale(0.001, center=(0, 0, 0))
            print("Mesh scaled and loaded successfully")
            
            # Create point cloud from mesh for visualization and interaction
            mesh_pcd = self.mesh.sample_points_uniformly(number_of_points=150000)
            self.cloud_np = np.asarray(mesh_pcd.points)
            self.kdtree = cKDTree(self.cloud_np)
            
            # Initialize Open3D point cloud
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(self.cloud_np)
            self.pcd.colors = o3d.utility.Vector3dVector(np.ones((len(self.cloud_np), 3)))  # White
            
            self.bbox = self.pcd.get_axis_aligned_bounding_box()
            self.scale = max(self.bbox.get_extent())
            self.paint_dist_threshold = self.scale * 0.015
            
            # Prepare mesh for visualization
            self.mesh.compute_vertex_normals()
            self.mesh.paint_uniform_color([0.3, 0.5, 0.8])  # Blueish gray
            self.mesh_vertices = np.asarray(self.mesh.vertices)
            
            self.use_mesh = True
            print(f"[OK] Mesh loaded: {len(self.cloud_np)} points")
            
        except Exception as e:
            print(f"[ERROR] Failed to load mesh: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)

        # Initialize Tkinter UI
        print("[DEBUG] Creating Tkinter root window...")
        try:
            self.root = tk.Tk()
        except Exception as e:
            print(f"[ERROR] Failed to create Tkinter window: {e}")
            print("[ERROR] This usually means no display is available.")
            print("[ERROR] Set DISPLAY variable or use remote X forwarding.")
            import traceback
            traceback.print_exc()
            raise
        
        print("[DEBUG] Tkinter root window created successfully")
        self.root.title("Point Cloud Logic Manager")
        self.root.geometry("450x450")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        tk.Label(self.root, text=f"Interaction Mode: {'STL MESH GRAPH' if self.use_mesh else 'POINT CLOUD'}", fg="green" if self.use_mesh else "red", font=("Arial", 10, "italic")).pack(pady=(10, 0))

        tk.Label(self.root, text="STEP 1: Pick Start Point", font=("Arial", 11, "bold")).pack(pady=(15, 0))
        tk.Label(self.root, text="• Shift + Left Click ONE point in 3D", fg="blue").pack()
        tk.Button(self.root, text="Open Step 1", command=self.step1_pick, width=20).pack(pady=5)

        tk.Label(self.root, text="STEP 2: Advanced 3D Paint Brush", font=("Arial", 11, "bold")).pack(pady=(20, 0))
        tk.Label(self.root, text="• Watch the Yellow 3D Cursor hug the surface!\n• Press and HOLD 'P' key while moving mouse\nto paint a trajectory along the mesh.", fg="blue").pack()
        tk.Button(self.root, text="Open Paint Brush Tool", command=self.step2_draw, width=25, bg="#e0e0e0").pack(pady=5)

        tk.Label(self.root, text="ALTERNATIVE: Pick Points on Cloud", font=("Arial", 11, "bold")).pack(pady=(20, 0))
        tk.Label(self.root, text="• Shift + Click to select multiple points\n• Auto-connect them into a smooth trajectory.", fg="blue").pack()
        tk.Button(self.root, text="Open Point Picking Tool", command=self.step2_pick_trajectory, width=25, bg="#f0e0e0").pack(pady=5)

        tk.Button(self.root, text="Preview & Save Coordinates", command=self.save_data, bg="green", fg="white", font=("Arial", 10, "bold"), width=30).pack(pady=20)

        if not self.use_mesh:
            messagebox.showwarning("Mesh Not Found", "Could not load STL. Falling back to simple point cloud picking.")

        self.root.mainloop()

    def on_closing(self):
        self.root.quit()
        self.root.destroy()
        sys.exit(0)

    # ---------------------------------------------------------
    # STEP 1: 3D Pick
    # ---------------------------------------------------------
    def step1_pick(self):
        self.root.withdraw()
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Step 1: Pick Start Point (Shift+LClick)", width=1280, height=720)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        
        # Always show the uniformly sampled cloud_np for picking across entire surface
        vis.add_geometry(self.pcd)
            
        vis.run() 
        picked = vis.get_picked_points()
        vis.destroy_window()
        self.root.deiconify() 
        
        if picked:
            # picked indices are directly into cloud_np
            self.pick_coord = self.cloud_np[picked[-1]]
            messagebox.showinfo("Success", "Pick point selected and snapped to exact cloud position!")

    # ---------------------------------------------------------
    # STEP 2: PICK POINTS ON CLOUD & CREATE TRAJECTORY
    # ---------------------------------------------------------
    def step2_pick_trajectory(self):
        """Pick multiple points on the point cloud to create a trajectory"""
        self.root.withdraw()
        
        # Create visualization for picking
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Step 2: Pick Points on Cloud (Shift+LClick)", width=1280, height=720)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        opt.point_size = 5.0
        
        vis.add_geometry(self.pcd)
        
        vis.run()
        picked_indices = vis.get_picked_points()
        vis.destroy_window()
        
        if len(picked_indices) < 2:
            self.root.deiconify()
            messagebox.showwarning("Need More Points", "Please select at least 2 points to create a trajectory.")
            return
        
        # Convert picked indices to actual cloud coordinates
        picked_points = self.cloud_np[picked_indices]
        
        # Store picked waypoints for later visualization
        self.picked_waypoints = picked_points
        
        # Create smooth trajectory connecting all picked points
        self.trajectory_coords = self._create_trajectory_from_points(picked_points)
        
        # Show preview with red picked points and green trajectory
        self._preview_trajectory(picked_points)
        
        self.root.deiconify()
        messagebox.showinfo("Success", 
            f"Selected {len(picked_indices)} points!\n"
            f"Generated smooth trajectory with {len(self.trajectory_coords)} waypoints.\n\n"
            f"Click 'Preview & Save Coordinates' to finalize.")
    
    def _preview_trajectory(self, picked_points):
        """Preview the trajectory with red picked points and green path"""
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Trajectory Preview (Red=Picked, Green=Path)", width=1280, height=720)
        
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        opt.point_size = 3.0
        
        # Add point cloud (white background)
        vis.add_geometry(self.pcd)
        
        # Add picked points as red spheres
        for pt in picked_points:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=self.scale * 0.015)
            sphere.compute_vertex_normals()
            sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red
            sphere.translate(pt)
            vis.add_geometry(sphere)
        
        # Add trajectory as green line
        if len(self.trajectory_coords) > 1:
            lineset = o3d.geometry.LineSet()
            lineset.points = o3d.utility.Vector3dVector(self.trajectory_coords)
            lines = [[i, i+1] for i in range(len(self.trajectory_coords)-1)]
            lineset.lines = o3d.utility.Vector2iVector(lines)
            lineset.paint_uniform_color([0.0, 1.0, 0.0])  # Green
            vis.add_geometry(lineset)
        
        vis.get_view_control().set_up([0, -1, 0])
        vis.get_view_control().set_front([0, 0, -1])
        vis.get_view_control().set_lookat(self.bbox.get_center())
        
        vis.run()
        vis.destroy_window()
    
    def _create_trajectory_from_points(self, picked_points):
        """Create a smooth trajectory connecting picked points"""
        if len(picked_points) < 2:
            return []
        
        # Connect all picked points in order
        all_pts = [picked_points[0]]
        
        for i in range(1, len(picked_points)):
            # Linear interpolation between consecutive picked points
            start = picked_points[i-1]
            end = picked_points[i]
            dist = np.linalg.norm(end - start)
            steps = max(int(dist / (self.paint_dist_threshold * 2)), 5)
            
            for j in range(1, steps + 1):
                t = j / steps
                interp_pt = start * (1 - t) + end * t
                # Snap to nearest cloud point
                _, idx = self.kdtree.query(interp_pt)
                all_pts.append(self.cloud_np[idx])
        
        # Smooth and resample
        traj = np.array(all_pts)
        
        # Simple laplacian smoothing
        for _ in range(3):
            for i in range(1, len(traj) - 1):
                traj[i] = 0.5 * traj[i] + 0.25 * (traj[i-1] + traj[i+1])
        
        # Remove consecutive duplicates
        final_traj = [traj[0]]
        for pt in traj[1:]:
            if not np.allclose(pt, final_traj[-1], atol=1e-5):
                final_traj.append(pt)
        
        return final_traj

    # ---------------------------------------------------------
    # STEP 2: ADVANCED 3D PAINT BRUSH
    # ---------------------------------------------------------
    def step2_draw(self):
        if not self.use_mesh:
            messagebox.showerror("Error", "STL Mesh is required for Advanced 3D Painting.")
            return

        self.root.withdraw()

        try:
            gui.Application.instance.initialize()
        except:
            pass

        paint_window = gui.Application.instance.create_window(
            "Advanced Painter (Hold SHIFT and DRAG to paint)", 1280, 720)

        widget = gui.SceneWidget()
        widget.scene = rendering.Open3DScene(paint_window.renderer)
        widget.scene.set_background([0, 0, 0, 1])
        paint_window.add_child(widget)

        # Add mesh
        mat_mesh = rendering.MaterialRecord()
        mat_mesh.shader = "defaultLit"
        widget.scene.add_geometry("mesh", self.mesh, mat_mesh)

        # Raycasting
        ray_scene = o3d.t.geometry.RaycastingScene()
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)
        ray_scene.add_triangles(mesh_t)

        # Mesh KDTree (for surface projection)
        mesh_vertices = np.asarray(self.mesh.vertices)
        mesh_normals = np.asarray(self.mesh.vertex_normals)
        mesh_kdtree = cKDTree(mesh_vertices)

        self.painted_coords = []
        self.is_painting = False  # Flag to track if P key is held

        # Cursor
        cursor_radius = max(self.scale * 0.015, 0.01)
        cursor_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=cursor_radius)
        cursor_sphere.compute_vertex_normals()

        mat_cursor = rendering.MaterialRecord()
        mat_cursor.shader = "defaultUnlit"
        mat_cursor.base_color = [1.0, 1.0, 0.0, 0.8]
        widget.scene.add_geometry("cursor", cursor_sphere, mat_cursor)

        mat_line = rendering.MaterialRecord()
        mat_line.shader = "unlitLine"
        mat_line.line_width = 8.0

        def on_key(event):
            """Handle key press/release for painting mode"""
            if event.key == gui.KeyName.P:
                if event.type == gui.KeyEvent.Type.DOWN:
                    self.is_painting = True
                else:  # KeyEvent.Type.UP
                    self.is_painting = False
            return gui.Widget.EventCallbackResult.HANDLED

        def get_safe_raycast(x, y):
            w, h = widget.frame.width, widget.frame.height
            if w <= 0 or h <= 0:
                return None

            p0 = np.array(widget.scene.camera.unproject(x, y, 0.0, w, h))
            p1 = np.array(widget.scene.camera.unproject(x, y, 1.0, w, h))

            # Check for NaN or invalid values
            if not (np.isfinite(p0).all() and np.isfinite(p1).all()):
                return None

            ray_dir = p1 - p0
            norm = np.linalg.norm(ray_dir)
            if norm < 1e-6:
                return None
            ray_dir /= norm

            ray = np.concatenate([p0, ray_dir]).astype(np.float32)
            ans = ray_scene.cast_rays(o3d.core.Tensor([ray]))
            t_hit = ans['t_hit'][0].item()

            if np.isinf(t_hit):
                return None

            return p0 + t_hit * ray_dir

        def laplacian_smooth(points, iterations=2):
            pts = points.copy()
            for _ in range(iterations):
                for i in range(1, len(pts) - 1):
                    pts[i] = 0.5 * pts[i] + 0.25 * (pts[i - 1] + pts[i + 1])
            return pts

        def arc_length_resample(points, ds):
            if len(points) < 2:
                return points

            d = np.linalg.norm(np.diff(points, axis=0), axis=1)
            s = np.concatenate([[0], np.cumsum(d)])
            total = s[-1]
            new_s = np.arange(0, total, ds)

            new_pts = []
            for val in new_s:
                idx = np.searchsorted(s, val)
                if idx == 0:
                    new_pts.append(points[0])
                elif idx < len(points):
                    t = (val - s[idx - 1]) / (s[idx] - s[idx - 1] + 1e-8)
                    interp = points[idx - 1] + t * (points[idx] - points[idx - 1])
                    new_pts.append(interp)

            return np.array(new_pts)

        def get_translation_matrix(vec3):
            T = np.eye(4)
            T[:3, 3] = vec3
            return T

        def on_mouse(event):

            hit = get_safe_raycast(event.x, event.y)
            if hit is None:
                return gui.Widget.EventCallbackResult.IGNORED

            # Project to nearest mesh vertex (surface constraint)
            _, vidx = mesh_kdtree.query(hit)
            hit = mesh_vertices[vidx]
            normal = mesh_normals[vidx]

            # Move cursor
            widget.scene.set_geometry_transform("cursor",
                get_translation_matrix(hit))

            if self.is_painting:

                if not self.painted_coords:
                    self.painted_coords.append(hit)
                else:
                    prev = self.painted_coords[-1]
                    move_vec = hit - prev

                    # Tangential constraint (remove normal component)
                    move_vec -= np.dot(move_vec, normal) * normal
                    new_pt = prev + move_vec

                    if np.linalg.norm(new_pt - prev) > self.paint_dist_threshold:
                        # Project constrained point back onto mesh surface
                        _, vidx_final = mesh_kdtree.query(new_pt)
                        new_pt = mesh_vertices[vidx_final]
                        self.painted_coords.append(new_pt)

                if len(self.painted_coords) >= 2:
                    pts = np.array(self.painted_coords)
                    pts = laplacian_smooth(pts, 2)
                    pts = arc_length_resample(pts, self.paint_dist_threshold)

                    lines = [[i, i + 1] for i in range(len(pts) - 1)]
                    lineset = o3d.geometry.LineSet()
                    lineset.points = o3d.utility.Vector3dVector(pts)
                    lineset.lines = o3d.utility.Vector2iVector(lines)

                    if widget.scene.has_geometry("paint"):
                        widget.scene.remove_geometry("paint")

                    widget.scene.add_geometry("paint", lineset, mat_line)

                return gui.Widget.EventCallbackResult.HANDLED

            return gui.Widget.EventCallbackResult.IGNORED

        widget.set_on_mouse(on_mouse)
        widget.set_on_key(on_key)

        btn = gui.Button("Finish Painting")
        btn.set_on_clicked(lambda: gui.Application.instance.quit())
        paint_window.add_child(btn)

        def on_layout(layout_context):
            r = paint_window.content_rect
            widget.frame = r
            btn.frame = gui.Rect(r.x + 20, r.y + 20, 150, 40)

        paint_window.set_on_layout(on_layout)

        widget.setup_camera(60.0, self.bbox, self.bbox.get_center())
        gui.Application.instance.run()

        self.root.deiconify()

        if len(self.painted_coords) > 1:
            raw = np.array(self.painted_coords)
            raw = laplacian_smooth(raw, 4)
            raw = arc_length_resample(raw, self.paint_dist_threshold)
            self.trajectory_coords = self.project_optimal_path(raw, self.pick_coord)

            messagebox.showinfo("Success",
                f"Surface-constrained trajectory with {len(self.trajectory_coords)} waypoints generated.")
    def project_optimal_path(self, raw_coords, start_coord):
        """Maps the already distance-sampled path directly to the exact Point Cloud"""
        # Snap ALL path points directly to the nearest real ROS Point Cloud dots
        _, indices = self.kdtree.query(raw_coords)
        snapped_points = self.cloud_np[indices]
        
        # Filter perfect duplicates to keep path mathematically clean
        final_path = []
        if start_coord is not None:
            final_path.append(start_coord)
            
        for pt in snapped_points:
            if len(final_path) == 0 or not np.allclose(pt, final_path[-1], atol=1e-5):
                final_path.append(pt)

        return final_path

    # ---------------------------------------------------------
    # FINAL PREVIEW & DIAGNOSTICS
    # ---------------------------------------------------------
    def save_data(self):
        self.root.withdraw()
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Preview (Large Red=Start Point, Small Green=Trajectory)", width=1280, height=720)
        
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        opt.point_size = 3.0
        
        vis.add_geometry(self.pcd)

        if self.use_mesh:
            wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(self.mesh)
            wireframe.paint_uniform_color([0.0, 0.4, 0.8])
            vis.add_geometry(wireframe)

        # Show single large RED sphere for pick_cord (start point)
        if self.pick_coord is not None:
            radius = self.scale * 0.05  # Large sphere
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            sphere.compute_vertex_normals()
            sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red - START POINT
            sphere.translate(self.pick_coord)
            vis.add_geometry(sphere)

        # Show trajectory points as small GREEN spheres
        if len(self.trajectory_coords) > 0:
            for pt in self.trajectory_coords:
                radius = self.scale * 0.01  # Small spheres
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                sphere.compute_vertex_normals()
                sphere.paint_uniform_color([0.0, 1.0, 0.0])  # Green - TRAJECTORY
                sphere.translate(pt)
                vis.add_geometry(sphere)

        vis.get_view_control().set_up([0, -1, 0])
        vis.get_view_control().set_front([0, 0, -1])
        vis.get_view_control().set_lookat(self.bbox.get_center())

        vis.run()
        vis.destroy_window()

        # Save Logic
        # Coordinates are in model_frame (same as stl_to_cloud_node output)
        frame_id = "model_frame"
        data = {"frame_id": frame_id, "pick_point": None, "trajectory": []}
        print("\n" + "="*50)
        print("FINAL EXACT MODEL COORDINATES")
        print("="*50)
        print(f"Frame ID: {frame_id}\n")

        if self.pick_coord is not None:
            data["pick_point"] = self.pick_coord.tolist()
            print(f"[RED START]: X={self.pick_coord[0]:.4f}, Y={self.pick_coord[1]:.4f}, Z={self.pick_coord[2]:.4f}")

        if len(self.trajectory_coords) > 0:
            print(f"\n[GREEN PATH] ({len(self.trajectory_coords)} waypoints):")
            for i, coords in enumerate(self.trajectory_coords):
                data["trajectory"].append(coords.tolist())
                if i < 5 or i >= len(self.trajectory_coords) - 5:
                    print(f"  Point {i+1:03d}: X={coords[0]:.4f}, Y={coords[1]:.4f}, Z={coords[2]:.4f}")
                elif i == 5:
                    print("  ... (intermediate waypoints) ...")

        output_file = os.path.join(config_dir, "surface_plan.json")
        with open(output_file, "w") as f:
            json.dump(data, f, indent=4)

        print(f"\n=> Data saved to '{output_file}'\n" + "="*50 + "\n")
        
        self.root.deiconify()
        messagebox.showinfo("Saved", f"Data printed to terminal and saved to:\n{output_file}\nExiting program.")
        self.on_closing()

# ============================================
# MAIN
# ============================================

def main():
    try:
        print("[INFO] Starting Drawing GUI (no ROS required)...")
        editor = SurfaceEditor()
        print("[INFO] GUI closed successfully")
        
    except Exception as e:
        print(f"[ERROR] Exception in main: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()