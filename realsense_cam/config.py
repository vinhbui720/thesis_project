# =========================
# region GLOBAL CONFIG
# =========================

CONFIG = {

    # =========================
    # region SOURCE
    # =========================
    "source": {
        "mode": "bag",  # "bag" or "live"
        "bag_path": "data/20260408_203426.bag",
    },
    # endregion


    # =========================
    # region GPU
    # =========================
    "gpu": {
        "enable": True,
        "opencv_cuda": True,
        "open3d_cuda": True,
    },
    # endregion

    # =========================
    # region ROI (pixel space)
    # =========================
    "roi": (0, 0, 1000, 300),
    # endregion


    # =========================
    # region DEPTH FILTER
    # =========================
    "depth": {
        "z_low": 0.2,        # bỏ điểm quá gần
        "height": 0.04,      # object = z_min + height
        "median_ksize": 5,
    },
    # endregion


    # =========================
    # region BACKGROUND SUBTRACTION
    # =========================
    "bg": {
        "threshold": 25,     # 20 nhạy, 40 ổn định
        "blur": 5,           # kernel Gaussian (3 hoặc 5)
        "path": "data/background.png",
    },
    # endregion


    # =========================
    # region MORPHOLOGY
    # =========================
    "morph": {
        "kernel": 5,
        "dilate_iter": 2,
    },
    # endregion


    # =========================
    # region DETECTION (2D)
    # =========================
    "detect": {
        "min_area": 300,     # dùng cho contour depth
    },
    # endregion


    # =========================
    # region OBJECT (FINAL FILTER)
    # =========================
    "object": {
        "min_area": 500,     # contour sau fusion
        "pad_meter": 0.01,   # padding bbox (m → pixel)
    },
    # endregion


    # =========================
    # region INITIAL POSE (ICP)
    # =========================
    "init_pose": {

        # trigger condition
        "min_points": 500,        # số điểm mask_final tối thiểu

        # downsample
        "voxel_size": 0.005,

        # ICP
        "icp_threshold": 0.02,

        # outlier removal
        "outlier_nb": 20,
        "outlier_std": 2.0,

        # performance
        "max_points": 20000,      # limit cloud size (🔥 tăng tốc)
        "target_points": 5000,    # hard cap for ICP input cloud
        "max_iter": 30,
        "async": True,
        "invert_rotation": True,
        "min_points_icp": 100,
    },
    # endregion


    # =========================
    # region DEBUG
    # =========================
    "debug": {
        "show_init_cloud": True,   # show cloud trước ICP
        "print_icp": True,         # log fitness + rmse
        "print_fps": False,
    },
    # endregion


    # =========================
    # region VISUALIZATION
    # =========================
    "vis": {

        # window toggle
        "rgb": True,
        "mask_bg": True,
        "mask_depth": True,
        "mask_final": True,
        "crop": True,

        # overlays
        "show_roi": True,
        "show_depth_box": True,
        "show_final_box": True,

        # future use
        "show_velocity": True,
        "show_debug_text": True,
        "draw_every_n": 1,
    }
    # endregion

}

# =========================
# endregion
# =========================