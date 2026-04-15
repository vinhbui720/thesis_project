import rclpy
from rclpy.node import Node
import tf2_ros
import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R, Slerp
import time
import open3d as o3d
import sys

class RealSenseCamPoseEstimator(Node):
    def __init__(self):
        super().__init__('rgbd_cam_pose_estimator')

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estimation State (CAMERA IN WORLD)
        self.est_T_W_C = None
        self.alpha = 0.01  # Continuous long-term learning
        self.iteration_count = 0
        
        # Motion gating
        self.prev_tf_translation = None
        self.prev_tf_time = None
        self.velocity_threshold = 0.001
        self.current_velocity = 0.0

        # ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # RealSense Setup
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            self.profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            # --- CAMERA WARM-UP ---
            self.get_logger().info("Warming up RealSense sensor...")
            for _ in range(30):
                self.pipeline.wait_for_frames()
            self.get_logger().info("Warm-up complete.")
            
            self.pc = rs.pointcloud()
            color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.intrinsics = color_stream.get_intrinsics()
            self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        except Exception as e:
            self.get_logger().error(f"RealSense Init Error: {e}")
            sys.exit(1)

        # Open3D Setup
        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='3D Point Cloud', width=800, height=600)
        self.first_frame = True

    def get_tag_center_3d(self, corners, depth_data):
        c = corners[0]
        sample_pixels = [
            (int(c[0][0]), int(c[0][1])), (int(c[1][0]), int(c[1][1])),
            (int(c[2][0]), int(c[2][1])), (int(c[3][0]), int(c[3][1])),
            (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))
        ]
        points_3d = []
        for u, v in sample_pixels:
            if 0 <= u < 640 and 0 <= v < 480:
                dist = depth_data[v, u] * self.depth_scale
                if 0.1 < dist < 3.0:
                    points_3d.append(rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], dist))
        return np.mean(points_3d, axis=0) if len(points_3d) >= 3 else None

    def is_axis_visible(self, rvec, tvec, K, dist, axis_length=0.05):
        """Checks if the 3D axis endpoints are within the 2D image frame."""
        axis_points = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length]
        ], dtype=np.float32)
        
        img_pts, _ = cv2.projectPoints(axis_points, rvec, tvec, K, dist)
        img_pts = img_pts.reshape(-1, 2)
        
        for p in img_pts:
            if not (0 <= p[0] < 640 and 0 <= p[1] < 480):
                return False
        return True

    def run_once(self):
        """Single iteration of the processing loop, called from main thread."""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame: return

            depth_data = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            pc_color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            hud_status, hud_color = "SEARCHING", (0, 165, 255)

            if ids is not None:
                ids = ids.flatten()
                for i in range(len(ids)):
                    cv2.fillPoly(pc_color_image, corners[i].astype(np.int32), (255, 0, 0))
                    t_cam_tag = self.get_tag_center_3d(corners[i], depth_data)
                    if t_cam_tag is None: continue

                    camera_matrix = np.array([[self.intrinsics.fx, 0, self.intrinsics.ppx],
                                              [0, self.intrinsics.fy, self.intrinsics.ppy],
                                              [0, 0, 1]], dtype=np.float32)
                    dist_coeffs = np.array(self.intrinsics.coeffs, dtype=np.float32)
                    marker_points_3d = np.array([[-0.05,0.05,0],[0.05,0.05,0],[0.05,-0.05,0],[-0.05,-0.05,0]], dtype=np.float32)
                    
                    success, rvec, _ = cv2.solvePnP(marker_points_3d, corners[i], camera_matrix, dist_coeffs)
                    if not success: continue
                    
                    if self.is_axis_visible(rvec, t_cam_tag, camera_matrix, dist_coeffs):
                        cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, t_cam_tag, 0.05)
                    
                    T_C_T = np.eye(4); T_C_T[:3, :3] = cv2.Rodrigues(rvec)[0]; T_C_T[:3, 3] = t_cam_tag
                    T_T_C = np.linalg.inv(T_C_T)

                    try:
                        tf = self.tf_buffer.lookup_transform('world', 'calib_link', rclpy.time.Time())
                        tag_w_t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                        tag_w_q = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
                        T_W_T = np.eye(4); T_W_T[:3, :3] = R.from_quat(tag_w_q).as_matrix(); T_W_T[:3, 3] = tag_w_t
                        T_W_C_instant = T_W_T @ T_T_C

                        now = time.time()
                        is_learning = False
                        if self.prev_tf_translation is not None:
                            if (np.linalg.norm(tag_w_t - self.prev_tf_translation) / (now - self.prev_tf_time)) < self.velocity_threshold:
                                is_learning, hud_status, hud_color = True, "ONLINE LEARNING", (0, 255, 0)
                            else: hud_status, hud_color = "TAG MOVING", (0, 0, 255)
                        self.prev_tf_translation, self.prev_tf_time = tag_w_t, now

                        if self.est_T_W_C is None: self.est_T_W_C = T_W_C_instant
                        elif is_learning:
                            self.est_T_W_C[:3, 3] += self.alpha * (T_W_C_instant[:3, 3] - self.est_T_W_C[:3, 3])
                            slerp = Slerp([0, 1], R.concatenate([R.from_matrix(self.est_T_W_C[:3, :3]), R.from_matrix(T_W_C_instant[:3, :3])]))
                            self.est_T_W_C[:3, :3] = slerp([self.alpha])[0].as_matrix()
                            self.iteration_count += 1
                    except: pass

            # --- SAFE OPEN3D UPDATE ---
            self.pc.map_to(color_frame)
            points = self.pc.calculate(depth_frame)
            v = points.get_vertices()
            if v:
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3).copy()
                cols = (pc_color_image.reshape(-1, 3) / 255.0).astype(np.float64).copy()
                self.pcd.points = o3d.utility.Vector3dVector(verts.astype(np.float64))
                self.pcd.colors = o3d.utility.Vector3dVector(cols)
                if self.first_frame: self.vis.add_geometry(self.pcd); self.first_frame = False
                self.vis.update_geometry(self.pcd)
                if not self.vis.poll_events(): rclpy.shutdown()
                self.vis.update_renderer()

            # --- DISPLAY & LOGGING ---
            if self.est_T_W_C is not None:
                tx, ty, tz = self.est_T_W_C[:3, 3]
                rpy = R.from_matrix(self.est_T_W_C[:3, :3]).as_euler('xyz', degrees=True)
                
                # 1. Console Log (Throttled to 1Hz)
                self.get_logger().info(
                    f"CAM POSE -> X:{tx:.3f} Y:{ty:.3f} Z:{tz:.3f} | R:{rpy[0]:.1f} P:{rpy[1]:.1f} Y:{rpy[2]:.1f}",
                    throttle_duration_sec=1.0
                )

                # 2. On-screen Text UI
                y_off = 140
                cv2.putText(color_image, "STATIC CAM (FUSED):", (20, y_off), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(color_image, f"X:{tx:.3f} Y:{ty:.3f} Z:{tz:.3f} [m]", (25, y_off+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(color_image, f"R:{rpy[0]:.1f} P:{rpy[1]:.1f} Y:{rpy[2]:.1f} [deg]", (25, y_off+45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.rectangle(color_image, (10, 10), (410, 210), (0, 0, 0), -1)
            cv2.putText(color_image, f"STATUS: {hud_status}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hud_color, 2)
            cv2.putText(color_image, f"Samples: {self.iteration_count}", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            cv2.imshow("RealSense RGB Preview", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        if hasattr(self, 'vis'): self.vis.destroy_window()
        if hasattr(self, 'pipeline'): self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCamPoseEstimator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.run_once() 
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()