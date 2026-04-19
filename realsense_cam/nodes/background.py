import cv2
import numpy as np

class Background:
    def __init__(self, config, width, height, initial_bg=None):
        self.cfg = config
        u1, v1, u2, v2 = self.cfg["roi"]
        self.roi = (
            max(0, min(int(u1), width - 1)),
            max(0, min(int(v1), height - 1)),
            max(1, min(int(u2), width)),
            max(1, min(int(v2), height)),
        )
        self.kernel = np.ones((config["morph"]["kernel"],)*2, np.uint8)
        self.blur = int(config["bg"]["blur"]) | 1

        gpu_cfg = config.get("gpu", {})
        self.use_cuda = (
            gpu_cfg.get("enable", True)
            and gpu_cfg.get("opencv_cuda", True)
            and hasattr(cv2, "cuda")
            and cv2.cuda.getCudaEnabledDeviceCount() > 0
        )

        if initial_bg is not None:
            bg = initial_bg
            if len(bg.shape) == 3:
                bg = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)
        else:
            bg = cv2.imread(config["bg"]["path"])
            if bg is None:
                raise FileNotFoundError(f"Could not load background image from {config['bg']['path']}")
            bg = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)

        if bg.shape != (height, width):
            bg = cv2.resize(bg, (width, height))

        self.bg = bg

        if self.use_cuda:
            u1, v1, u2, v2 = self.roi
            self.bg_roi_gpu = cv2.cuda_GpuMat()
            self.bg_roi_gpu.upload(self.bg[v1:v2, u1:u2])
            self.gauss = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (self.blur, self.blur), 0)
            self.bg_blur_gpu = self.gauss.apply(self.bg_roi_gpu)

            morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.cfg["morph"]["kernel"], self.cfg["morph"]["kernel"]))
            self.morph_filter = cv2.cuda.createMorphologyFilter(
                cv2.MORPH_OPEN,
                cv2.CV_8UC1,
                morph_kernel,
            )
            self.dilate_filter = cv2.cuda.createMorphologyFilter(
                cv2.MORPH_DILATE,
                cv2.CV_8UC1,
                morph_kernel,
            )
            self.roi_gpu = cv2.cuda_GpuMat()

    def process(self, data):
        u1, v1, u2, v2 = self.roi
        data["roi_used"] = self.roi

        if self.use_cuda and "gray_gpu" in data:
            gray_roi = data["gray"][v1:v2, u1:u2]
            self.roi_gpu.upload(gray_roi)

            roi_blur = self.gauss.apply(self.roi_gpu)
            diff = cv2.cuda.absdiff(roi_blur, self.bg_blur_gpu)
            _, mask_gpu = cv2.cuda.threshold(diff, self.cfg["bg"]["threshold"], 255, cv2.THRESH_BINARY)
            mask_gpu = self.morph_filter.apply(mask_gpu)

            if self.cfg["morph"]["dilate_iter"] > 0:
                for _ in range(int(self.cfg["morph"]["dilate_iter"])):
                    mask_gpu = self.dilate_filter.apply(mask_gpu)

            data["mask_bg_gpu"] = mask_gpu
            mask = mask_gpu.download()
        else:
            roi = data["gray"][v1:v2, u1:u2]

            roi = cv2.GaussianBlur(roi, (self.blur, self.blur), 0)
            bg = cv2.GaussianBlur(self.bg[v1:v2, u1:u2], (self.blur, self.blur), 0)

            diff = cv2.absdiff(roi, bg)

            _, mask = cv2.threshold(diff, self.cfg["bg"]["threshold"],255,cv2.THRESH_BINARY)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.dilate(mask, self.kernel, iterations=self.cfg["morph"]["dilate_iter"])

        data["mask_bg"] = mask
        return data