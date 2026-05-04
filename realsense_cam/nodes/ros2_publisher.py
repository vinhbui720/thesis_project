import time
import numpy as np
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, TransformStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# Identity quaternion [x, y, z, w] — used for all published poses (no orientation)
_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


class ROS2Publisher(Node):
    """
    Pipeline node — inserts after PoseFusion, before Visualize.

    Architecture
    ─────────────
    camera pipeline  ──► process()       ──► updates estimator AND publishes
    robot feedback   ──► _feedback_cb()  ──► caches EE position for lead-time

    State machine (driven by tracker_kalman gate):
      "idle"     — before start line: nothing published
      "tracking" — between start and stop lines: estimator live, publishes each frame
      "done"     — after stop line: estimator frozen, no publishing

    When in "tracking" and camera has no measurement this frame (dead-reckon frame),
    the last known velocity is used to propagate the estimate forward.

    Lead-time: dist(EE → object_now) / approach_speed  (capped at lead_time_max_s).
    Published target = pos_now + vel_est * lead_time.
    """

    def __init__(self, cfg, icp_node):
        super().__init__('object_tracker')
        self.cfg = cfg
        self.icp_node = icp_node
        ros_cfg = cfg.get("ros2_publisher", {})

        self.frame_id          = ros_cfg.get("frame_id",          "world_depth_camera_link")
        self.tracking_frame_id = ros_cfg.get("tracking_frame_id", "tracking_task")

        self._vel_alpha       = ros_cfg.get("vel_smooth_alpha",   0.85)
        self._pos_alpha       = ros_cfg.get("pos_correct_alpha",  0.30)
        self._cam_timeout     = ros_cfg.get("cam_timeout_s",      0.25)
        self._lead_time_fixed = ros_cfg.get("lead_time_s",        0.30)
        self._lead_time_max   = ros_cfg.get("lead_time_max_s",    1.00)
        self._approach_speed  = ros_cfg.get("approach_speed_m_s", 0.15)

        # ── Estimator state machine ────────────────────────────────────────
        self._est_state       = "idle"   # "idle" | "tracking" | "done"
        self._pos_est         = None
        self._vel_est         = np.zeros(3, dtype=np.float64)
        self._t_est           = None
        self._cam_t_last      = None
        self._cached_rotation = np.eye(3, dtype=np.float64)

        # ── Robot EE state (protected by _ee_lock) ────────────────────────
        self._ee_pos  = None
        self._ee_lock = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_pose   = self.create_publisher(PoseStamped,  '/tracking_target_pose',   10)
        self._pub_vel    = self.create_publisher(TwistStamped, '/object/velocity',        10)
        self._pub_status = self.create_publisher(String,       '/object/tracking_status', 10)
        self._pub_active = self.create_publisher(Bool,         '/object/tracking_active', 10)

        # ── TF2 Broadcaster ───────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Robot feedback subscription ───────────────────────────────────
        self._sub_feedback = self.create_subscription(
            Twist, '/motomini/feedback', self._feedback_cb, 10)

        # ── Service ───────────────────────────────────────────────────────
        self._srv_icp = self.create_service(
            Trigger, '/object/retrigger_icp', self._handle_retrigger_icp)

        self._icp_min_fitness    = ros_cfg.get(
            "icp_min_fitness", cfg.get("init_pose", {}).get("icp_min_fitness", 0.5))
        self._retrigger_pending  = False
        self._icp_result_ready   = False
        self._icp_result_fitness = 0.0
        self._icp_result_rmse    = 0.0
        self._icp_result_success = False
        self._icp_result_lock    = threading.Lock()
        self._service_event      = threading.Event()

    # ======================================================================
    # ROBOT FEEDBACK
    # ======================================================================
    def _feedback_cb(self, msg):
        """Cache EE Cartesian position from /motomini/feedback (linear = position)."""
        with self._ee_lock:
            self._ee_pos = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float64)

    # ======================================================================
    # SERVICE HANDLER
    # ======================================================================
    def _handle_retrigger_icp(self, request, response):
        self.get_logger().info('ICP retrigger requested')
        with self._icp_result_lock:
            self._icp_result_ready  = False
            self._retrigger_pending = True
        with self.icp_node.lock:
            self.icp_node.ready   = False
            self.icp_node.running = False
        self._service_event.clear()
        finished = self._service_event.wait(timeout=30.0)
        if not finished:
            response.success = False
            response.message = "ICP timed out after 30 s"
            return response
        with self._icp_result_lock:
            fitness = self._icp_result_fitness
            rmse    = self._icp_result_rmse
            ok      = self._icp_result_success
        if ok:
            response.success = True
            response.message = f"fitness={fitness:.4f} rmse={rmse:.5f}"
        else:
            response.success = False
            response.message = (f"fitness too low: {fitness:.4f} < "
                                f"threshold {self._icp_min_fitness:.4f}  rmse={rmse:.5f}")
        return response

    # ======================================================================
    # PIPELINE process()  — called once per camera frame
    # Updates estimator AND publishes directly (no separate timer).
    # ======================================================================
    def process(self, data):
        stamp = self.get_clock().now().to_msg()

        # ── ICP retrigger bookkeeping ──────────────────────────────────────
        with self._icp_result_lock:
            pending = self._retrigger_pending
        if pending and self.icp_node.ready:
            with self.icp_node.lock:
                fitness = self.icp_node.fitness
                rmse    = self.icp_node.inlier_rmse
            ok = fitness >= self._icp_min_fitness
            with self._icp_result_lock:
                self._icp_result_fitness = fitness
                self._icp_result_rmse    = rmse
                self._icp_result_success = ok
                self._icp_result_ready   = True
                self._retrigger_pending  = False
            self._service_event.set()

        # ── Gate state machine ─────────────────────────────────────────────
        # None  → bbox not detected this frame (camera miss) — keep current state
        # True  → object confirmed inside start→stop zone
        # False → object confirmed outside zone (passed stop line)
        gate_active = data.get("tracking_active", None)

        if gate_active is True:
            if self._est_state in ("idle", "done"):
                self._pos_est    = None
                self._vel_est    = np.zeros(3, dtype=np.float64)
                self._t_est      = None
                self._cam_t_last = None
                self._est_state  = "tracking"
                self.get_logger().info('Gate entered — estimator reset, tracking started.')
        elif gate_active is False:
            if self._est_state == "tracking":
                self._est_state = "done"
                self.get_logger().info('Gate exited — estimator frozen, publishing stopped.')

        # ── Only run estimator and publish when in tracking state ──────────
        if self._est_state != "tracking":
            # Still publish TF so the frame stays visible in RViz at all times
            pose_mat  = data.get("pose",      None)
            center_3d = data.get("center_3d", None)
            if pose_mat is not None:
                t_raw = pose_mat[:3, 3].copy()
                t_raw[2] += 0.05
                self._publish_transform(stamp, t_raw, _IDENTITY_QUAT)
            elif center_3d is not None:
                t_raw    = np.asarray(center_3d, dtype=np.float64).copy()
                t_raw[2] += 0.05
                self._publish_transform(stamp, t_raw, _IDENTITY_QUAT)
            self._publish_status(stamp, data)
            return data

        # ── Extract camera measurement ─────────────────────────────────────
        pose_mat  = data.get("pose",      None)
        center_3d = data.get("center_3d", None)
        velocity  = data.get("velocity",  None)  # (vx_m, vy_m) in world XY

        now = time.monotonic()

        if pose_mat is not None:
            meas_pos = pose_mat[:3, 3].copy()
        elif center_3d is not None:
            meas_pos = np.asarray(center_3d, dtype=np.float64).copy()
        else:
            # ── OCCLUSION: camera miss this frame ──────────────────────────
            # Propagate the estimator forward so state stays current.
            # When camera returns, dt_pred is only ~1 frame → smooth correction.
            if self._pos_est is not None:
                dt = max(0.0, now - self._t_est)
                self._pos_est = self._pos_est + self._vel_est * dt
                self._t_est   = now
                # _cam_t_last intentionally NOT updated → cam_ok stays False
                self._publish_target_pose(stamp, self._pos_est, self._vel_est)
            self._publish_status(stamp, data)
            return data

        # Apply Z offset
        meas_pos[2] += 0.05

        # 2D camera velocity → 3D (no Z from camera; preserve current Z estimate)
        meas_vel    = np.zeros(3, dtype=np.float64)
        if velocity is not None:
            meas_vel[0] = float(velocity[0])
            meas_vel[1] = float(velocity[1])
        meas_vel[2] = self._vel_est[2]

        # ── Fuse measurement into estimator (complementary filter) ─────────
        if self._pos_est is None:
            self._pos_est    = meas_pos.copy()
            self._vel_est    = meas_vel.copy()
            self._t_est      = now
            self._cam_t_last = now
            self.get_logger().info(f'Estimator initialized: pos={meas_pos}')
        else:
            cam_gap  = (now - self._cam_t_last) if self._cam_t_last is not None else 0.0
            dt_pred  = max(0.0, now - self._t_est)
            pos_pred = self._pos_est + self._vel_est * dt_pred

            # Adaptive alpha: snap harder after occlusion to zero drift fast
            snap_rate = 0.5
            a = min(1.0, self._pos_alpha + cam_gap * snap_rate)
            self._pos_est = (1.0 - a) * pos_pred + a * meas_pos

            b = self._vel_alpha
            self._vel_est = b * self._vel_est + (1.0 - b) * meas_vel

            self._t_est      = now
            self._cam_t_last = now

            if cam_gap > self._cam_timeout:
                self.get_logger().debug(
                    f'Occlusion ended ({cam_gap*1000:.0f} ms) — '
                    f'alpha={a:.2f}, '
                    f'err={np.linalg.norm(pos_pred - meas_pos)*1000:.1f} mm')

        self._publish_target_pose(stamp, self._pos_est, self._vel_est)
        self._publish_status(stamp, data)
        return data

    # ======================================================================
    # PUBLISH HELPERS
    # ======================================================================
    def _publish_target_pose(self, stamp, pos_now, vel_now):
        """Publish current object position (x, y, z+0.05) with identity orientation."""
        quat = _IDENTITY_QUAT

        # PoseStamped
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(pos_now[0])
        ps.pose.position.y = float(pos_now[1])
        ps.pose.position.z = float(pos_now[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        self._pub_pose.publish(ps)

        # TF
        self._publish_transform(stamp, pos_now, quat)

        # Velocity
        ts = TwistStamped()
        ts.header.stamp    = stamp
        ts.header.frame_id = self.frame_id
        ts.twist.linear.x  = float(vel_now[0])
        ts.twist.linear.y  = float(vel_now[1])
        ts.twist.linear.z  = float(vel_now[2])
        self._pub_vel.publish(ts)

        # Active flag
        active_msg      = Bool()
        active_msg.data = True
        self._pub_active.publish(active_msg)

    # ======================================================================
    # HELPERS
    # ======================================================================
    def _publish_transform(self, stamp, translation, quaternion):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id  = self.tracking_frame_id
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(t)

    def _publish_status(self, stamp, data):
        icp_state = ("READY"   if data.get("icp_ready",   False) else
                     "RUNNING" if data.get("icp_running", False) else "IDLE")
        tracking  = "ACTIVE" if data.get("tracking_active", False) else "WAIT"
        has_pose  = "pose" in data
        fitness   = self.icp_node.fitness
        rmse      = self.icp_node.inlier_rmse
        now       = time.monotonic()
        dt_cam    = (now - self._cam_t_last) if self._cam_t_last is not None else 999.0
        cam_ok    = dt_cam < self._cam_timeout
        status = (f'{{"tracking":"{tracking}",'
                  f'"est_state":"{self._est_state}",'
                  f'"icp":"{icp_state}",'
                  f'"has_pose":{str(has_pose).lower()},'
                  f'"icp_fitness":{fitness:.4f},'
                  f'"icp_rmse":{rmse:.5f},'
                  f'"cam_fresh":{str(cam_ok).lower()},'
                  f'"cam_age_ms":{dt_cam * 1000:.0f}}}')
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)
