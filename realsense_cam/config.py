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
    "roi": (0, 0, 640, 300),
    # endregion


    # =========================
    # region DEPTH FILTER
    # =========================
    "depth": {
        "z_low": 0.02,        # bỏ điểm quá gần
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
    # region TRACKING GATE (pixel space)
    # =========================
    "tracking_gate": {
        # ── Zone gate ──────────────────────────────────────────────────────
        "enabled":    True,
        "start_line": [(150, 0), (150, 300)],  # green line  (object enters here)
        "stop_line":  [(500, 0), (500, 300)],  # red   line  (object exits  here)

        # ── Kalman process noise (Q matrix, diagonal) ─────────────────────
        # process_noise_pos : trust on position model  (smaller → position smoother)
        # process_noise_vel : trust on velocity model  (larger → velocity adapts faster)
        # Rule: process_noise_vel >> process_noise_pos to make velocity responsive
        "process_noise_pos": 1e-4,  # Q[x, y]   — keep small: position is stable
        "process_noise_vel": 5e-2,  # Q[vx, vy] — keep large: velocity must learn fast

        # ── Kalman measurement noise (R matrix) ───────────────────────────
        # measure_noise : how much to trust the raw bbox centroid
        # larger → smoother position, slower to react; smaller → jumpy but responsive
        "measure_noise": 1e-2,

        # ── 3-D centre smoothing ───────────────────────────────────────────
        # ema_alpha: weight on OLD value (0.0 = no smoothing, 1.0 = frozen)
        "ema_alpha": 0.2,

        # ── Depth patch for robust Z lookup ───────────────────────────────
        # depth_patch_k: half-size of median window in pixels (window = 2k+1 × 2k+1)
        "depth_patch_k": 5,

        # ── Direction enforcement (start → stop) ──────────────────────────
        # init_velocity_hint : initial vx seed (px/s) in the start→stop direction
        #   when object first enters the gate — prevents zero/reverse velocity cold start
        "init_velocity_hint": 0.05,

        # velocity_dampen_reverse : if True, clamp any velocity component that points
        #   backwards (against start→stop direction) to zero inside the gate
        "velocity_dampen_reverse": True,
    },
    # endregion
    # =========================
    # region OBJECT (FINAL FILTER)
    # =========================
    "object": {
        "min_area": 300,     # contour sau fusion
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

        # quality gate for retrigger service response
        # ICP result is reported as FAIL if fitness < this threshold
        "icp_min_fitness": 0.5,
    },
    # endregion


    # =========================
    # region ROS2 PUBLISHER
    # =========================
    "ros2_publisher": {
        "frame_id": "world_depth_camera_link",    # Parent TF frame for published messages
        "tracking_frame_id": "tracking_task",     # Child TF frame representing tracked object
        "debug_tracking_frame_id": "tracking_target",
        "world_frame": "world",
        "magnetic_link_frame": "magnetic_link",

        # Minimum ICP fitness to consider a retrigger successful.
        "icp_min_fitness": 0.5,

        # ── Estimator (complementary filter) ──────────────────────────────
        # How strongly to blend the velocity measurement in (0=never update, 1=always raw)
        "vel_smooth_alpha":    0.85,
        # How strongly to pull the position estimate toward the camera measurement
        # (0=trust prediction only, 1=trust camera only)
        "pos_correct_alpha":   0.30,
        # Age (seconds) after which the camera is considered occluded
        "cam_timeout_s":       0.25,
        # Added to the tracked object Z target during follow/predict
        "target_z_offset_m":   0.01,

        # ── Bootstrap from magnetic_link ──────────────────────────────────
        # First published poses start from magnetic_link then converge rapidly
        # to the tracked object estimate.
        "bootstrap_gain":              12.0,
        "bootstrap_pos_tolerance_m":   0.01,
        "bootstrap_max_duration_s":    0.35,

        # ── Lead-time (interception prediction) ───────────────────────────
        # Fixed fallback lead time used when robot EE position is unavailable
        "lead_time_s":         0.30,
        # Hard cap on lead time
        "lead_time_max_s":     1.00,
        # Assumed robot closing speed (m/s) used for dynamic lead-time:
        #   lead_time = dist(EE → object) / approach_speed_m_s
        "approach_speed_m_s":  0.15,

        # ── Prediction rollout after temporary camera loss ────────────────
        "prediction_max_time_s":      0.75,
        "prediction_max_distance_m":  0.25,

        # ── Publisher timer rate ───────────────────────────────────────────
        "pub_rate_hz":        50.0,
    },
    # endregion


    # =========================
    # region DEBUG
    # =========================
    "debug": {
        "show_init_cloud": True,   # show cloud trước ICP
        "print_icp": True,         # log fitness + rmse
        "print_fps": True,
    },
    # endregion


    # =========================
    # region VISUALIZATION
    # =========================
    "vis": {

        # window toggle
        "rgb": True,
        "mask_bg": False,
        "mask_depth": False,
        "mask_final": False,
        "crop": False,

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
