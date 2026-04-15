import cv2
import numpy as np
import pyrealsense2 as rs

# ---------------------------------------------------------
# 1. Initialize the ArUco detector (Replaces AprilTag)
# ---------------------------------------------------------
# Your generator used a 5x5 dictionary. DICT_5X5_1000 covers this perfectly.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ---------------------------------------------------------
# 2. Camera & Marker Parameters
# ---------------------------------------------------------
# Intrinsic matrix and distortion coefficients (keep your existing ones)
camera_matrix = np.array([
    [600, 0, 320],
    [0, 600, 240],
    [0, 0, 1]
], dtype=np.float32)
dist_coeffs = np.zeros(5, dtype=np.float32)

# Marker size in meters (100mm = 0.100m based on your screenshot)
marker_size = 0.100 

# Define the 3D corners of the marker for pose estimation
marker_points = np.array([
    [-marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32)

# ---------------------------------------------------------
# 3. Configure RealSense Pipeline
# ---------------------------------------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the frame to a numpy array and then to grayscale
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ---------------------------------------------------------
        # 4. Detect ArUco markers
        # ---------------------------------------------------------
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Flatten the ArUco IDs array to easily check them
            ids = ids.flatten()
            
            for i, marker_id in enumerate(ids):
                # Filter for your specific IDs (0, 50, 100, 250, 1000)
                if marker_id in [0, 50, 100, 250, 1000]:
                    
                    # Draw the green bounding box around the detected marker
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    # Estimate the pose using OpenCV's solvePnP
                    success, rvec, tvec = cv2.solvePnP(
                        marker_points, corners[i], camera_matrix, dist_coeffs
                    )

                    if success:
                        # Format the translation vector (pose)
                        tvec_str = f"[{tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}]"
                        pose_text = f"ID: {marker_id}, Pose: {tvec_str}"
                        
                        # Calculate the center of the marker to place the text nicely
                        c = corners[i][0]
                        center_x = int((c[0][0] + c[2][0]) / 2)
                        center_y = int((c[0][1] + c[2][1]) / 2)
                        
                        # Draw the text on the screen
                        cv2.putText(frame, pose_text, (center_x - 50, center_y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        # Optional: Draw the 3D coordinate axes on the marker
                        # Note: requires cv2.drawFrameAxes in newer OpenCV versions
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size / 2)

        # Show the frame
        cv2.imshow("ArUco Marker Detection", frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()