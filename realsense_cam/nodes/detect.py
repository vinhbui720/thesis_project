import cv2

class Detect:
    def __init__(self, cfg):
        self.cfg = cfg

    def process(self,data):
        contours,_ = cv2.findContours(data["mask_final"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return data

        cnt = max(contours, key=cv2.contourArea)

        if cv2.contourArea(cnt) < self.cfg["object"]["min_area"]:
            return data

        x,y,w,h = cv2.boundingRect(cnt)

        u1, v1, _, _ = data.get("roi_used", self.cfg["roi"])

        data["bbox"] = (x+u1,y+v1,w,h)
        return data