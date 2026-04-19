import pyrealsense2 as rs
import numpy as np

class SourceRealCam:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable depth and color streams from the real camera
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)

        # Better to get the device and sensor to set some options if needed, 
        # but for now, we'll follow the SourceBag structure.
        
        self.align = rs.align(rs.stream.color)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        self.fx, self.fy = intr.fx, intr.fy
        self.cx, self.cy = intr.ppx, intr.ppy
        self.width, self.height = intr.width, intr.height

        # --- Camera Warmup ---
        print("[INFO] Warming up camera...")
        for _ in range(30):
            try:
                self.pipeline.wait_for_frames()
            except:
                pass
        print("[INFO] Camera warmup complete.")

    def process(self, data):
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            print(f"Error fetching frames: {e}")
            return None

        # Align depth frame to color frame
        frames = self.align.process(frames)

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            return None

        data["depth_frame"] = depth
        data["color"] = np.asanyarray(color.get_data())

        return data

    def close(self):
        self.pipeline.stop()
