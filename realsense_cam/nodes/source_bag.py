import pyrealsense2 as rs
import numpy as np

class SourceBag:
    def __init__(self, path=None, mode="bag"):
        self.mode = mode
        self.pipeline = rs.pipeline()
        config = rs.config()

        if mode == "bag":
            if not path:
                raise ValueError("Bag mode requires a valid .bag path")
            config.enable_device_from_file(path, repeat_playback=False)

        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)

        profile = self.pipeline.start(config)

        if mode == "bag":
            playback = profile.get_device().as_playback()
            playback.set_real_time(False)

        self.align = rs.align(rs.stream.color)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        self.fx, self.fy = intr.fx, intr.fy
        self.cx, self.cy = intr.ppx, intr.ppy
        self.width, self.height = intr.width, intr.height

    def process(self, data):
        try:
            frames = self.pipeline.wait_for_frames()
        except RuntimeError:
            # Bag playback ends here.
            return None

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