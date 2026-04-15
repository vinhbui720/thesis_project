import cv2

class Preprocess:
    def __init__(self, cfg=None):
        gpu_cfg = (cfg or {}).get("gpu", {})
        self.use_cuda = (
            gpu_cfg.get("enable", True)
            and gpu_cfg.get("opencv_cuda", True)
            and hasattr(cv2, "cuda")
            and cv2.cuda.getCudaEnabledDeviceCount() > 0
        )
        self.frame_gpu = cv2.cuda_GpuMat() if self.use_cuda else None

    def process(self, data):
        color = data["color"]

        if self.use_cuda:
            self.frame_gpu.upload(color)
            gray_gpu = cv2.cuda.cvtColor(self.frame_gpu, cv2.COLOR_BGR2GRAY)
            data["gray_gpu"] = gray_gpu
            # CPU gray is still needed by downstream CPU-only nodes.
            data["gray"] = gray_gpu.download()
        else:
            data["gray"] = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        return data