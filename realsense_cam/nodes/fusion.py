import cv2

class Fusion:
    def __init__(self):
        self.use_cuda = hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0

    def process(self,data):
        if self.use_cuda and "mask_bg_gpu" in data:
            depth_gpu = cv2.cuda_GpuMat()
            depth_gpu.upload(data["mask_depth"])
            final_gpu = cv2.cuda.bitwise_and(data["mask_bg_gpu"], depth_gpu)
            data["mask_final_gpu"] = final_gpu
            data["mask_final"] = final_gpu.download()
        else:
            data["mask_final"] = cv2.bitwise_and(data["mask_bg"], data["mask_depth"])
        return data