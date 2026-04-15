import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------
# Helper Function: SVD 3D-3D Alignment (Arun's Method)
# ---------------------------------------------------------
def get_rigid_transform(P_tag, P_cam):
    """
    Calculates the optimal Rotation and Translation to align P_tag to P_cam.
    Returns the 4x4 transformation matrix (Camera to Tag).
    """
    # 1. Find centroids
    centroid_tag = np.mean(P_tag, axis=0)
    centroid_cam = np.mean(P_cam, axis=0)

    # 2. Center the points
    p_tag_centered = P_tag - centroid_tag
    p_cam_centered = P_cam - centroid_cam

    # 3. Covariance matrix
    H = p_tag_centered.T @ p_cam_centered

    # 4. SVD
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 5. Handle Reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 6. Calculate translation
    t = centroid_cam.T - R @ centroid_tag.T

    # 7. Build 4x4 Homogeneous Matrix
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    return T

# ---------------------------------------------------------
# 1. Initialize the ArUco detector
# ---------------------------------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Marker size in meters (100mm = 0.100m)
marker_size = 0.100 

# Define the ideal 3D corners of the marker (P_tag)
P_tag = np.array([
    [-marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32)

# ---------------------------------------------------------
# 2. Configure RealSense Pipeline for RGB-D Fusion
# ---------------------------------------------------------
pipeline = rs.pipeline()
config = rs.config()

# Enable BOTH Color and Depth streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
profile = pipeline.start(config)

# Get the depth sensor's depth scale (usually 0.001 meters)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create an align object to warp depth to color frame (CRITICAL FOR MOVING OBJECTS)
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Wait for coherent frames
        frames = pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # Get RealSense Factory Intrinsics dynamically
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # ---------------------------------------------------------
        # 3. Detect ArUco markers & Perform RGB-D Math
        # ---------------------------------------------------------
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            
            for i, marker_id in enumerate(ids):
                if marker_id in [0, 50, 100, 250, 1000]:
                    cv2.aruco.drawDetectedMarkers(color_image, corners)
                    
                    marker_corners_2d = corners[i][0]
                    P_cam = []
                    valid_depth = True

                    # Extract 3D coordinates for all 4 corners using the Depth Map
                    for corner in marker_corners_2d:
                        u, v = int(corner[0]), int(corner[1])
                        
                        # Ensure coordinates are within image bounds
                        u = np.clip(u, 0, depth_image.shape[1] - 1)
                        v = np.clip(v, 0, depth_image.shape[0] - 1)

                        # Get depth in meters
                        depth = aligned_depth_frame.get_distance(u, v)
                        
                        if depth <= 0:
                            valid_depth = False
                            break # Invalid depth reading at the corner
                            
                        # Deproject 2D pixel to 3D point using RealSense intrinsics
                        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
                        P_cam.append(point_3d)

                    if valid_depth:
                        P_cam = np.array(P_cam, dtype=np.float32)

                        # Calculate Transformation from Camera to Tag (T_Cam_to_Tag) using SVD
                        T_C_to_T = get_rigid_transform(P_tag, P_cam)
                        
                        # For testing on the belt, assume the Tag is the "World Origin".
                        # To find where the Camera is in the World, invert the matrix:
                        T_World_to_Cam = np.linalg.inv(T_C_to_T)
                        
                        cam_x = T_World_to_Cam[0, 3]
                        cam_y = T_World_to_Cam[1, 3]
                        cam_z = T_World_to_Cam[2, 3]

                        pose_text = f"Cam Pose (m): [{cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}]"
                        
                        # Draw text
                        center_x = int(np.mean(marker_corners_2d[:, 0]))
                        center_y = int(np.mean(marker_corners_2d[:, 1]))
                        cv2.putText(color_image, pose_text, (center_x - 100, center_y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("RGB-D ArUco Pose Estimation", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()