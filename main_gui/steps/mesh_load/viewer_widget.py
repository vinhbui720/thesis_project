from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QMessageBox
from PyQt6.QtCore import Qt
import numpy as np
import os

# Try to import visualization libraries
try:
    import matplotlib
    matplotlib.use('Qt5Agg')  # Use Qt5Agg backend
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class MeshViewerWidget(QWidget):
    """
    3D Visualization widget for mesh and point cloud display.
    Uses matplotlib for stable rendering with PyQt6.
    """
    
    def __init__(self):
        super().__init__()
        
        layout = QVBoxLayout()
        self.setLayout(layout)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Info label
        self.info_label = QLabel("3D Viewer - Load STL to begin")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.info_label.setStyleSheet(
            "background-color: #f0f0f0; padding: 10px; border-radius: 5px; font-weight: bold;"
        )
        layout.addWidget(self.info_label)
        
        # Statistics display
        self.stats_label = QLabel("")
        self.stats_label.setStyleSheet("background-color: white; padding: 10px; font-family: monospace;")
        layout.addWidget(self.stats_label)
        
        # Create matplotlib figure if available
        if MATPLOTLIB_AVAILABLE:
            self.figure = Figure(figsize=(8, 6), dpi=100)
            self.canvas = FigureCanvasQTAgg(self.figure)
            layout.addWidget(self.canvas, 1)
            print("[INFO] Using Matplotlib for 3D visualization")
        else:
            warning = QLabel("⚠ Matplotlib not available - Using text-only visualization")
            warning.setStyleSheet("color: #ff9800; padding: 10px;")
            layout.addWidget(warning)
            print("[WARNING] Matplotlib not available")
        
        # Geometry storage
        self.current_mesh = None
        self.current_cloud = None
        self.mesh_vertices = None
        self.cloud_points = None

    def load_mesh(self, path):
        """Load and display STL mesh"""
        try:
            if not os.path.exists(path):
                raise FileNotFoundError(f"File not found: {path}")
            
            # Try to read with trimesh first (more reliable)
            try:
                import trimesh
                mesh = trimesh.load(path)
                vertices = np.array(mesh.vertices)
                print(f"[INFO] Loaded mesh with trimesh: {len(vertices)} vertices")
            except:
                # Fallback to open3d
                try:
                    import open3d as o3d
                    mesh = o3d.io.read_triangle_mesh(path)
                    vertices = np.array(mesh.vertices)
                    print(f"[INFO] Loaded mesh with Open3D: {len(vertices)} vertices")
                except Exception as e:
                    raise Exception(f"Could not load mesh: {e}")
            
            self.mesh_vertices = vertices
            self.current_mesh = {
                'path': path,
                'vertices': len(vertices),
                'bounds': [vertices.min(axis=0), vertices.max(axis=0)]
            }
            
            self.update_stats()
            self.draw_visualization()
            
            self.info_label.setText(
                f"✓ STL Loaded: {len(vertices)} vertices | Waiting for point cloud..."
            )
            self.info_label.setStyleSheet(
                "background-color: #c8e6c9; padding: 10px; border-radius: 5px; font-weight: bold;"
            )
                
        except Exception as e:
            self.info_label.setText(f"✗ Error loading mesh: {str(e)}")
            self.info_label.setStyleSheet(
                "background-color: #ffcdd2; padding: 10px; border-radius: 5px; font-weight: bold;"
            )
            print(f"[ERROR] Failed to load mesh: {e}")

    def show_cloud(self, points):
        """Display point cloud"""
        try:
            if not isinstance(points, np.ndarray):
                points = np.array(points)
            
            if points.shape[1] != 3:
                points = points[:, :3]
            
            self.cloud_points = points
            self.current_cloud = {
                'points': len(points),
                'bounds': [points.min(axis=0), points.max(axis=0)]
            }
            
            self.update_stats()
            self.draw_visualization()
            
            mesh_info = f" | {self.current_mesh['vertices']} mesh vertices" if self.current_mesh else ""
            self.info_label.setText(
                f"✓ Point Cloud: {len(points)} points{mesh_info}"
            )
            self.info_label.setStyleSheet(
                "background-color: #c8e6c9; padding: 10px; border-radius: 5px; font-weight: bold;"
            )
            print(f"[INFO] Displayed point cloud: {len(points)} points")
                
        except Exception as e:
            self.info_label.setText(f"✗ Error displaying cloud: {str(e)}")
            self.info_label.setStyleSheet(
                "background-color: #ffcdd2; padding: 10px; border-radius: 5px; font-weight: bold;"
            )
            print(f"[ERROR] Failed to show cloud: {e}")

    def update_stats(self):
        """Update statistics display"""
        stats = "📊 Visualization Statistics:\n"
        stats += "=" * 50 + "\n"
        
        if self.current_mesh:
            vertices = self.current_mesh['vertices']
            bounds = self.current_mesh['bounds']
            stats += f"Mesh:\n"
            stats += f"  • Vertices: {vertices:,}\n"
            stats += f"  • Bounds X: [{bounds[0][0]:.3f}, {bounds[1][0]:.3f}]\n"
            stats += f"  • Bounds Y: [{bounds[0][1]:.3f}, {bounds[1][1]:.3f}]\n"
            stats += f"  • Bounds Z: [{bounds[0][2]:.3f}, {bounds[1][2]:.3f}]\n\n"
        
        if self.current_cloud:
            points = self.current_cloud['points']
            bounds = self.current_cloud['bounds']
            stats += f"Point Cloud:\n"
            stats += f"  • Points: {points:,}\n"
            stats += f"  • Bounds X: [{bounds[0][0]:.3f}, {bounds[1][0]:.3f}]\n"
            stats += f"  • Bounds Y: [{bounds[0][1]:.3f}, {bounds[1][1]:.3f}]\n"
            stats += f"  • Bounds Z: [{bounds[0][2]:.3f}, {bounds[1][2]:.3f}]\n"
        
        self.stats_label.setText(stats)

    def draw_visualization(self):
        """Draw 3D visualization"""
        if not MATPLOTLIB_AVAILABLE:
            return
        
        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d')
        
        # Plot mesh
        if self.mesh_vertices is not None:
            ax.scatter(
                self.mesh_vertices[:, 0],
                self.mesh_vertices[:, 1],
                self.mesh_vertices[:, 2],
                c='gray',
                s=1,
                alpha=0.3,
                label='Mesh'
            )
        
        # Plot cloud
        if self.cloud_points is not None:
            ax.scatter(
                self.cloud_points[:, 0],
                self.cloud_points[:, 1],
                self.cloud_points[:, 2],
                c='green',
                s=5,
                alpha=0.8,
                label='Point Cloud'
            )
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Mesh & Point Cloud')
        if self.mesh_vertices is not None or self.cloud_points is not None:
            ax.legend()
        
        self.figure.tight_layout()
        self.canvas.draw()

    def clear_geometries(self):
        """Clear all visualizations"""
        self.mesh_vertices = None
        self.cloud_points = None
        self.current_mesh = None
        self.current_cloud = None
        self.stats_label.setText("")
        self.info_label.setText("3D Viewer - Load STL to begin")
        self.info_label.setStyleSheet(
            "background-color: #f0f0f0; padding: 10px; border-radius: 5px; font-weight: bold;"
        )
        if MATPLOTLIB_AVAILABLE:
            self.figure.clear()
            self.canvas.draw()

    def closeEvent(self, event):
        """Cleanup when widget closes"""
        self.clear_geometries()
        super().closeEvent(event)