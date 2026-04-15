import pyrealsense2 as rs
import numpy as np
import cv2

# ===== INIT =====
pipeline = rs.pipeline()
config = rs.config()

config.enable_device_from_file(
    "/home/vinbui/vinh_ws/src/aruco_tag/data/20260414_203433.bag",
    repeat_playback=False
)

config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

# Lấy thông tin video
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
fps = int(color_stream.fps())
width = color_stream.width()
height = color_stream.height()

print(f"Resolution: {width}x{height}, FPS: {fps}")

# ===== VIDEO WRITER =====
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('rgb_output.avi', fourcc, fps, (width, height))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('/home/vinbui/vinh_ws/src/realsense_cam/data/video_2.mp4', fourcc, fps, (width, height))

# ===== LOOP =====
try:
    while True:
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert to numpy
        color_image = np.asanyarray(color_frame.get_data())

        # Show
        cv2.imshow("RGB", color_image)

        # Save video
        out.write(color_image)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break

except RuntimeError:
    print("End of bag file reached")

finally:
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()