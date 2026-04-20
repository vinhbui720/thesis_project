import cv2
import numpy as np
import re

class TuningGUI:
    def __init__(self, src, config, bg_image=None):
        self.src = src
        self.config = config
        self.bg_image = bg_image
        self.win_name = "Tuning GUI - Press ENTER to continue"

        # Initialize tracking vars
        u1, v1, u2, v2 = self.config["roi"]
        self.u1, self.v1, self.u2, self.v2 = int(u1), int(v1), int(u2), int(v2)

        self.bg_thresh = int(self.config["bg"]["threshold"])
        self.bg_blur = int(self.config["bg"]["blur"])
        self.morph_k = int(self.config["morph"]["kernel"])
        self.morph_i = int(self.config["morph"]["dilate_iter"])

        self.z_low = float(self.config["depth"]["z_low"])
        self.z_height = float(self.config["depth"]["height"])
        
        self.min_detect = int(self.config["detect"]["min_area"])

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.text_color = (0, 255, 255) # Yellow for high visibility

    def pass_func(self, x):
        pass

    def draw_params(self, img, params):
        """Helper to draw tuning parameters on the image."""
        y = 70
        for label, value in params:
            # Draw with a black background for maximum readability
            cv2.putText(img, f"{label}: {value}", (10, y), self.font, 0.6, (0, 0, 0), 3) # Outline
            cv2.putText(img, f"{label}: {value}", (10, y), self.font, 0.6, self.text_color, 1) # Foreground
            y += 25

    def get_frame(self):
        data = {}
        data = self.src.process(data)
        if data is None or "color" not in data:
            return None, None
        return data["color"], data.get("depth_frame")

    def run_stage_0_roi(self):
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("u1", self.win_name, self.u1, self.src.width, self.pass_func)
        cv2.createTrackbar("v1", self.win_name, self.v1, self.src.height, self.pass_func)
        cv2.createTrackbar("u2", self.win_name, self.u2, self.src.width, self.pass_func)
        cv2.createTrackbar("v2", self.win_name, self.v2, self.src.height, self.pass_func)

        print("[Tuning] Stage 0: Calibrate ROI. Adjust Trackbars and press ENTER")

        while True:
            color, _ = self.get_frame()
            if color is None: continue

            self.u1 = cv2.getTrackbarPos("u1", self.win_name)
            self.v1 = cv2.getTrackbarPos("v1", self.win_name)
            self.u2 = cv2.getTrackbarPos("u2", self.win_name)
            self.v2 = cv2.getTrackbarPos("v2", self.win_name)

            self.u2 = max(self.u1 + 1, self.u2)
            self.v2 = max(self.v1 + 1, self.v2)

            vis = color.copy()
            cv2.rectangle(vis, (self.u1, self.v1), (self.u2, self.v2), (0, 255, 0), 2)
            
            cv2.putText(vis, "Stage 0: ROI Config (Press ENTER)", (10, 30), self.font, 0.7, (0, 0, 255), 2)
            
            # Pad with black so window size is consistent and we have space for text
            black_pad = np.zeros_like(color)
            vis = np.hstack((vis, black_pad))

            # Draw real-time params on image since trackbar labels might be hidden
            params = [
                ("u1", self.u1), ("v1", self.v1),
                ("u2", self.u2), ("v2", self.v2)
            ]
            self.draw_params(vis, params)

            cv2.imshow(self.win_name, vis)
            if cv2.waitKey(30) == 13: # ENTER key
                break
        
        cv2.destroyAllWindows()

    def run_stage_1_bg(self):
        if self.bg_image is None:
            # try to load from disk
            self.bg_image = cv2.imread(self.config["bg"]["path"])
            if self.bg_image is None:
                self.bg_image, _ = self.get_frame() # fallback

        if len(self.bg_image.shape) == 3:
            self.bg_image = cv2.cvtColor(self.bg_image, cv2.COLOR_BGR2GRAY)
            
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("Threshold", self.win_name, self.bg_thresh, 255, self.pass_func)
        cv2.createTrackbar("Blur", self.win_name, self.bg_blur, 21, self.pass_func)
        cv2.createTrackbar("Morph K", self.win_name, self.morph_k, 15, self.pass_func)
        cv2.createTrackbar("Dilate Iter", self.win_name, self.morph_i, 10, self.pass_func)
        
        print("[Tuning] Stage 1: Calibrate Background. Press ENTER")

        while True:
            color, _ = self.get_frame()
            if color is None: continue

            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            self.bg_thresh = cv2.getTrackbarPos("Threshold", self.win_name)
            self.bg_blur = cv2.getTrackbarPos("Blur", self.win_name)
            self.bg_blur = self.bg_blur | 1 # must be odd
            
            self.morph_k = max(1, cv2.getTrackbarPos("Morph K", self.win_name))
            self.morph_i = cv2.getTrackbarPos("Dilate Iter", self.win_name)

            roi_gray = gray[self.v1:self.v2, self.u1:self.u2]
            roi_bg = self.bg_image[self.v1:self.v2, self.u1:self.u2]

            try:
                roi_gray_blur = cv2.GaussianBlur(roi_gray, (self.bg_blur, self.bg_blur), 0)
                roi_bg_blur = cv2.GaussianBlur(roi_bg, (self.bg_blur, self.bg_blur), 0)

                diff = cv2.absdiff(roi_gray_blur, roi_bg_blur)
                _, mask = cv2.threshold(diff, self.bg_thresh, 255, cv2.THRESH_BINARY)
                
                kernel = np.ones((self.morph_k, self.morph_k), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.dilate(mask, kernel, iterations=self.morph_i)
                vis_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            except Exception as e:
                vis_mask = np.zeros_like(color[self.v1:self.v2, self.u1:self.u2])
                print(e)
            
            full_mask = np.zeros_like(color)
            if vis_mask.shape[0] > 0 and vis_mask.shape[1] > 0:
                full_mask[self.v1:self.v2, self.u1:self.u2] = vis_mask
                
            color_with_roi = color.copy()
            cv2.rectangle(color_with_roi, (self.u1, self.v1), (self.u2, self.v2), (0, 255, 0), 2)
            
            vis = np.hstack((color_with_roi, full_mask))
            
            cv2.putText(vis, "Stage 1: BG Sub (Press ENTER)", (10, 30), self.font, 0.7, (0, 0, 255), 2)
            
            # Draw real-time params on image
            params = [
                ("Threshold", self.bg_thresh), 
                ("Blur", self.bg_blur),
                ("Morph K", self.morph_k), 
                ("Dilate Iter", self.morph_i)
            ]
            self.draw_params(vis, params)
            
            cv2.imshow(self.win_name, vis)
            if cv2.waitKey(30) == 13: # ENTER key
                break
                
        cv2.destroyAllWindows()

    def run_stage_2_depth(self):
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("Z_low (cm)", self.win_name, int(self.z_low * 100), 200, self.pass_func)
        cv2.createTrackbar("Z_height (cm)", self.win_name, int(self.z_height * 100), 100, self.pass_func)
        cv2.createTrackbar("Detect Area", self.win_name, self.min_detect, 5000, self.pass_func)
        
        print("[Tuning] Stage 2: Calibrate Depth & Detect. Press ENTER")
        
        intr = (self.src.fx, self.src.fy, self.src.cx, self.src.cy, self.src.width, self.src.height)
        uu, vv = np.meshgrid(np.arange(self.src.width), np.arange(self.src.height))

        while True:
            color, depth_frame = self.get_frame()
            if color is None or depth_frame is None: continue

            z_l = cv2.getTrackbarPos("Z_low (cm)", self.win_name) / 100.0
            z_h = cv2.getTrackbarPos("Z_height (cm)", self.win_name) / 100.0
            self.z_low = z_l
            self.z_height = z_h
            self.min_detect = max(1, cv2.getTrackbarPos("Detect Area", self.win_name))

            depth_u16 = np.asanyarray(depth_frame.get_data())
            depth_scale = depth_frame.get_units()
            depth_image = depth_u16.astype(np.float32) * depth_scale

            roi_depth = depth_image[self.v1:self.v2, self.u1:self.u2]
            
            valid = np.isfinite(roi_depth) & (roi_depth > 0)
            z_valid = roi_depth[valid]

            # Display/log the current total depth points in ROI
            total_points = z_valid.size
            print(f"[Tuning][Depth] Total valid depth points in ROI: {total_points}")
            
            mask_roi = np.zeros_like(roi_depth, dtype=np.uint8)
            near_candidates = z_valid[z_valid > z_l]
            
            if near_candidates.size > 0:
                z_ref = float(np.percentile(near_candidates, 5.0))
                obj_mask = valid & (roi_depth > z_l) & (roi_depth < z_ref + z_h)
                
                v_local, u_local = np.where(obj_mask)
                mask_roi[v_local, u_local] = 255

            contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            vis_mask = cv2.cvtColor(mask_roi, cv2.COLOR_GRAY2BGR)
            if contours:
                cnt = max(contours, key=cv2.contourArea)
                if cv2.contourArea(cnt) > self.min_detect:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(vis_mask, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    
            full_mask = np.zeros_like(color)
            if vis_mask.shape[0] > 0 and vis_mask.shape[1] > 0:
                full_mask[self.v1:self.v2, self.u1:self.u2] = vis_mask
                
            color_with_roi = color.copy()
            cv2.rectangle(color_with_roi, (self.u1, self.v1), (self.u2, self.v2), (0, 255, 0), 2)
            
            vis = np.hstack((color_with_roi, full_mask))
            
            cv2.putText(vis, "Stage 2: Depth Sub (Press ENTER)", (10, 30), self.font, 0.7, (0, 0, 255), 2)
            
            # Draw real-time params on image
            params = [
                ("Z_low", f"{self.z_low:.2f}m"),
                ("Z_height", f"{self.z_height:.2f}m"),
                ("Min Area", self.min_detect),
                ("Total Depth Points", total_points)
            ]
            self.draw_params(vis, params)

            cv2.imshow(self.win_name, vis)
            if cv2.waitKey(30) == 13: # ENTER
                break
                
        cv2.destroyAllWindows()


    def rewrite_config(self, text, section, key, val, is_tuple=False):
        # Finds dictionary key assignment block and modifies it.
        # Simple regex strategy: limit to the section context
        
        # This is a naive regex matching, but valid enough for the structure:
        # "section": {
        #    ...
        #    "key": value,
        if is_tuple:
            pattern = r'("' + key + r'"\s*:\s*\()([\d\s,]+)(\))'
            replacement = r'\g<1>' + str(val).strip('()') + r'\g<3>'
        else:
            if isinstance(val, float):
                val_str = f"{val:.2f}"
            else:
                val_str = str(val)
            pattern = r'("' + key + r'"\s*:\s*)([\d\.]+)(,)'
            replacement = r'\g<1>' + val_str + r'\g<3>'

        text = re.sub(pattern, replacement, text)
        return text

    def save_config(self):
        print("[Tuning] Saving configuration to config.py...")
        with open("config.py", "r") as f:
            content = f.read()

        content = self.rewrite_config(content, "roi", "roi", (self.u1, self.v1, self.u2, self.v2), is_tuple=True)
        content = self.rewrite_config(content, "bg", "threshold", self.bg_thresh)
        content = self.rewrite_config(content, "bg", "blur", self.bg_blur)
        content = self.rewrite_config(content, "morph", "kernel", self.morph_k)
        content = self.rewrite_config(content, "morph", "dilate_iter", self.morph_i)
        content = self.rewrite_config(content, "depth", "z_low", self.z_low)
        content = self.rewrite_config(content, "depth", "height", self.z_height)
        content = self.rewrite_config(content, "detect", "min_area", self.min_detect)

        with open("config.py", "w") as f:
            f.write(content)
        print("[Tuning] Config saved successfully!")

    def start(self):
        self.run_stage_0_roi()
        self.run_stage_1_bg()
        self.run_stage_2_depth()
        self.save_config()
        self.src.close()
        print("[Tuning] Finished.")
