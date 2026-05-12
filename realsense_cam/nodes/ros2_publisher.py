import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

_IDENTITY_QUAT = np.array([1.0, 1.0, 0.0, 0.0], dtype=np.float64)


def _quat_normalize(quat):
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        return _IDENTITY_QUAT.copy()
    return quat / norm


def _quat_to_rot(quat):
    x, y, z, w = _quat_normalize(quat)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def _rot_to_quat(rot):
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return _quat_normalize(np.array([
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
            0.25 * s,
        ], dtype=np.float64))
    if rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        return _quat_normalize(np.array([
            0.25 * s,
            (rot[0, 1] + rot[1, 0]) / s,
            (rot[0, 2] + rot[2, 0]) / s,
            (rot[2, 1] - rot[1, 2]) / s,
        ], dtype=np.float64))
    if rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        return _quat_normalize(np.array([
            (rot[0, 1] + rot[1, 0]) / s,
            0.25 * s,
            (rot[1, 2] + rot[2, 1]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
        ], dtype=np.float64))
    s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
    return _quat_normalize(np.array([
        (rot[0, 2] + rot[2, 0]) / s,
        (rot[1, 2] + rot[2, 1]) / s,
        0.25 * s,
        (rot[1, 0] - rot[0, 1]) / s,
    ], dtype=np.float64))


def _transform_matrix(translation, quaternion):
    tfm = np.eye(4, dtype=np.float64)
    tfm[:3, :3] = _quat_to_rot(quaternion)
    tfm[:3, 3] = np.asarray(translation, dtype=np.float64)
    return tfm


class ROS2Publisher(Node):
    def __init__(self, cfg, icp_node):
        super().__init__('object_tracker')
        self.cfg = cfg
        self.icp_node = icp_node
        ros_cfg = cfg.get("ros2_publisher", {})

        self.frame_id = ros_cfg.get("frame_id", "world_depth_camera_link")
        self.tracking_frame_id = ros_cfg.get("tracking_frame_id", "tracking_task")
        self.debug_tracking_frame_id = ros_cfg.get("debug_tracking_frame_id", "tracking_target")
        self.world_frame = ros_cfg.get("world_frame", "world")
        self.magnetic_link_frame = ros_cfg.get("magnetic_link_frame", "magnetic_link")
        self.controller_pose_topic = ros_cfg.get("controller_pose_topic", "/motomini/target_pose")
        self.controller_vel_topic = ros_cfg.get("controller_vel_topic", "/motomini/target_vel")

        self._vel_alpha = ros_cfg.get("vel_smooth_alpha", 0.85)
        self._pos_alpha = ros_cfg.get("pos_correct_alpha", 0.30)
        self._cam_timeout = ros_cfg.get("cam_timeout_s", 0.25)
        self._target_z_offset = ros_cfg.get("target_z_offset_m", 0.01)
        self._bootstrap_gain = ros_cfg.get("bootstrap_gain", 5.0)
        self._bootstrap_tol = ros_cfg.get("bootstrap_pos_tolerance_m", 0.01)
        self._bootstrap_max_duration = ros_cfg.get("bootstrap_max_duration_s", 0.35)
        self._prediction_max_time = ros_cfg.get("prediction_max_time_s", 0.75)
        self._prediction_max_distance = ros_cfg.get("prediction_max_distance_m", 0.25)

        # Smooth published target.
        self._target_smooth_tau = ros_cfg.get("target_smooth_tau_s", 0.12)
        self._target_max_step = ros_cfg.get("target_max_step_m", 0.015)

        # Wait for stable velocity after start-line crossing.
        self._vel_stability_window = int(ros_cfg.get("velocity_stability_window", 8))
        self._vel_stability_std_thresh = float(
            ros_cfg.get("velocity_stability_std_thresh_mps", 0.015)
        )
        self._vel_stability_min_speed = float(
            ros_cfg.get("velocity_stability_min_speed_mps", 0.02)
        )
        self._vel_stability_timeout = float(
            ros_cfg.get("velocity_stability_timeout_s", 1.0)
        )
        self._use_locked_velocity_after_stable = bool(
            ros_cfg.get("use_locked_velocity_after_stable", True)
        )

        # Future tracking using Kalman velocity.
        self._lookahead_steps = ros_cfg.get("lookahead_steps", 3)
        self._lookahead_dt = ros_cfg.get("lookahead_dt_s", 0.033)
        self._lookahead_max_time = ros_cfg.get("lookahead_max_time_s", 0.20)

        # Object disappearance / robot occlusion handling.
        self._occlusion_expected_s = ros_cfg.get("occlusion_expected_s", 1.20)
        self._occlusion_hold_after_s = ros_cfg.get("occlusion_hold_after_s", 1.80)
        self._occlusion_max_predict_distance = ros_cfg.get(
            "occlusion_max_predict_distance_m", 0.35
        )
        self._hold_target_on_occlusion_timeout = ros_cfg.get(
            "hold_target_on_occlusion_timeout", True
        )

        self._occlusion_started_t = None
        self._occlusion_start_pos = None

        self._est_state = "idle"
        self._publish_phase = "wait"
        self._publish_source = "none"
        self._status_error = ""
        self._pending_start = False

        self._pos_est = None
        self._vel_est = np.zeros(3, dtype=np.float64)
        self._t_est = None
        self._cam_t_last = None

        self._bootstrap_pose = None
        self._bootstrap_quat = _IDENTITY_QUAT.copy()
        self._publish_pos = None
        self._publish_quat = _IDENTITY_QUAT.copy()
        self._track_started_t = None
        self._predict_started_t = None
        self._predict_start_pos = None
        self._last_target_pos = None
        self._publish_t_last = None

        self._ee_pos = None
        self._ee_lock = threading.Lock()
        self._vel_samples = []
        self._wait_stable_started_t = None
        self._stable_velocity = None
        self._stable_direction = None
        self._stable_start_pos = None
        self._stable_start_t = None
        self._tracking_publish_active = False

        self._pub_pose = self.create_publisher(PoseStamped, '/tracking_target_pose', 10)
        self._pub_vel = self.create_publisher(TwistStamped, '/object/velocity', 10)
        self._pub_ctrl_pose = self.create_publisher(PoseStamped, self.controller_pose_topic, 10)
        self._pub_ctrl_vel = self.create_publisher(TwistStamped, self.controller_vel_topic, 10)
        self._pub_status = self.create_publisher(String, '/object/tracking_status', 10)
        self._pub_active = self.create_publisher(Bool, '/object/tracking_active', 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sub_feedback = self.create_subscription(
            Twist, '/motomini/feedback', self._feedback_cb, 10)

        # Gate: only send commands to the controller when explicitly triggered.
        self._controller_ready = False
        self._controller_ready_lock = threading.Lock()
        self._sub_start_publish = self.create_subscription(
            Bool, '/object/start_publish_tracking', self._start_publish_cb, 10)

        self._srv_icp = self.create_service(
            Trigger, '/object/retrigger_icp', self._handle_retrigger_icp)

        self._icp_min_fitness = ros_cfg.get(
            "icp_min_fitness", cfg.get("init_pose", {}).get("icp_min_fitness", 0.5))
        self._retrigger_pending = False
        self._icp_result_ready = False
        self._icp_result_fitness = 0.0
        self._icp_result_rmse = 0.0
        self._icp_result_success = False
        self._icp_result_lock = threading.Lock()
        self._service_event = threading.Event()
        self._log_times = {}
        self._debug_tf_seen = False

    def _start_publish_cb(self, msg):
        """Receive trigger signal to enable/disable controller publishing."""
        if msg.data:
            with self._controller_ready_lock:
                if not self._controller_ready:
                    self.get_logger().info(
                        "Received start_publish_tracking signal — controller publish enabled.")
                    self._controller_ready = True
        else:
            with self._controller_ready_lock:
                if self._controller_ready:
                    self.get_logger().info(
                        "Received stop_publish_tracking signal — controller publish disabled.")
                    self._controller_ready = False

    def _log_throttled(self, key, level, interval_s, message):
        now = time.monotonic()
        last = self._log_times.get(key)
        if last is not None and (now - last) < interval_s:
            return
        self._log_times[key] = now
        # Explicit branches so each severity has a unique call-site line number.
        # rclpy raises ValueError if the same call-site is used with two different
        # severity levels, so we must NOT use getattr(...level)(message) here.
        if level == "debug":
            self.get_logger().debug(message)
        elif level == "info":
            self.get_logger().info(message)
        elif level in ("warning", "warn"):
            self.get_logger().warning(message)
        elif level == "error":
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def _feedback_cb(self, msg):
        with self._ee_lock:
            self._ee_pos = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float64)

    def _handle_retrigger_icp(self, request, response):
        self.get_logger().info('ICP retrigger requested')
        with self._icp_result_lock:
            self._icp_result_ready = False
            self._retrigger_pending = True
        with self.icp_node.lock:
            self.icp_node.ready = False
            self.icp_node.running = False
        self._service_event.clear()
        finished = self._service_event.wait(timeout=30.0)
        if not finished:
            response.success = False
            response.message = "ICP timed out after 30 s"
            return response
        with self._icp_result_lock:
            fitness = self._icp_result_fitness
            rmse = self._icp_result_rmse
            ok = self._icp_result_success
        if ok:
            response.success = True
            response.message = f"fitness={fitness:.4f} rmse={rmse:.5f}"
        else:
            response.success = False
            response.message = (
                f"fitness too low: {fitness:.4f} < "
                f"threshold {self._icp_min_fitness:.4f}  rmse={rmse:.5f}"
            )
        return response

    def _reset_estimator(self):
        self._pos_est = None
        self._vel_est = np.zeros(3, dtype=np.float64)
        self._t_est = None
        self._cam_t_last = None
        self._predict_started_t = None
        self._predict_start_pos = None
        self._occlusion_started_t = None
        self._occlusion_start_pos = None
        self._vel_samples = []
        self._wait_stable_started_t = None
        self._stable_velocity = None
        self._stable_direction = None
        self._stable_start_pos = None
        self._stable_start_t = None
        self._tracking_publish_active = False

    def _reset_track_session(self):
        self._reset_estimator()
        self._est_state = "idle"
        self._publish_phase = "wait"
        self._publish_source = "none"
        self._status_error = ""
        self._pending_start = False
        self._bootstrap_pose = None
        self._bootstrap_quat = _IDENTITY_QUAT.copy()
        self._publish_pos = None
        self._publish_quat = _IDENTITY_QUAT.copy()
        self._track_started_t = None
        self._last_target_pos = None
        self._publish_t_last = None
        self._occlusion_started_t = None
        self._occlusion_start_pos = None
        self._tracking_publish_active = False

    def _finish_tracking(self, reason):
        self._est_state = "done"
        self._publish_phase = "done"
        self._publish_source = reason
        self._status_error = ""
        self._pending_start = False
        self.get_logger().info(f'Tracking finished: reason={reason}')

    def _try_lookup_bootstrap_pose(self):
        try:
            tf_world_mag = self._tf_buffer.lookup_transform(
                self.world_frame, self.magnetic_link_frame, rclpy.time.Time())
        except TransformException as exc:
            self._status_error = f"bootstrap_tf:{exc}"
            self._log_throttled(
                "bootstrap_tf_fail", "warning", 1.0,
                f'Cannot start bootstrap: missing TF {self.world_frame} -> {self.magnetic_link_frame}: {exc}')
            return None, None

        world_pos = np.array([
            tf_world_mag.transform.translation.x,
            tf_world_mag.transform.translation.y,
            tf_world_mag.transform.translation.z,
        ], dtype=np.float64)
        world_quat = np.array([
            tf_world_mag.transform.rotation.x,
            tf_world_mag.transform.rotation.y,
            tf_world_mag.transform.rotation.z,
            tf_world_mag.transform.rotation.w,
        ], dtype=np.float64)

        if self.frame_id == self.world_frame:
            return world_pos, _quat_normalize(world_quat)

        try:
            tf_frame_world = self._tf_buffer.lookup_transform(
                self.frame_id, self.world_frame, rclpy.time.Time())
        except TransformException as exc:
            self._status_error = f"frame_tf:{exc}"
            self._log_throttled(
                "frame_tf_fail", "warning", 1.0,
                f'Cannot convert bootstrap pose into publish frame {self.frame_id}: {exc}')
            return None, None

        frame_world = _transform_matrix(
            [tf_frame_world.transform.translation.x,
             tf_frame_world.transform.translation.y,
             tf_frame_world.transform.translation.z],
            [tf_frame_world.transform.rotation.x,
             tf_frame_world.transform.rotation.y,
             tf_frame_world.transform.rotation.z,
             tf_frame_world.transform.rotation.w],
        )
        world_mag = _transform_matrix(world_pos, world_quat)
        frame_mag = frame_world @ world_mag
        return frame_mag[:3, 3].copy(), _rot_to_quat(frame_mag[:3, :3])

    def _start_tracking_session(self):
        bootstrap_pos, bootstrap_quat = self._try_lookup_bootstrap_pose()
        if bootstrap_pos is None:
            return False

        self._reset_estimator()
        self._est_state = "wait_vel_stable"
        self._publish_phase = "wait_vel_stable"
        self._publish_source = "waiting_for_stable_velocity"
        self._status_error = ""
        self._bootstrap_pose = bootstrap_pos
        self._bootstrap_quat = bootstrap_quat
        self._publish_pos = bootstrap_pos.copy()
        self._publish_quat = bootstrap_quat.copy()
        self._track_started_t = time.monotonic()
        self._wait_stable_started_t = self._track_started_t
        self._last_target_pos = bootstrap_pos.copy()
        self._publish_t_last = self._track_started_t
        self._tracking_publish_active = False
        self.get_logger().info(
            'Start-line crossed — waiting for stable velocity before publishing tracking.'
        )
        return True

    def _extract_measurement(self, data):
        pose_mat = data.get("pose", None)
        center_3d = data.get("center_3d", None)
        velocity = data.get("velocity", None)

        meas_pos = None
        if pose_mat is not None:
            meas_pos = pose_mat[:3, 3].astype(np.float64).copy()
        elif center_3d is not None:
            meas_pos = np.asarray(center_3d, dtype=np.float64).copy()

        if meas_pos is not None:
            meas_pos[2] += self._target_z_offset

        meas_vel = np.zeros(3, dtype=np.float64)
        if velocity is not None:
            meas_vel[0] = float(velocity[0])
            meas_vel[1] = float(velocity[1])
        meas_vel[2] = self._vel_est[2]
        return meas_pos, meas_vel

    def _fuse_measurement(self, meas_pos, meas_vel, now):
        if self._pos_est is None:
            self._pos_est = meas_pos.copy()
            self._vel_est = meas_vel.copy()
            self._t_est = now
            self._cam_t_last = now
            return

        cam_gap = (now - self._cam_t_last) if self._cam_t_last is not None else 0.0
        dt_pred = max(0.0, now - self._t_est)
        pos_pred = self._pos_est + self._vel_est * dt_pred
        snap_rate = 0.5
        alpha = min(1.0, self._pos_alpha + cam_gap * snap_rate)
        self._pos_est = (1.0 - alpha) * pos_pred + alpha * meas_pos
        self._vel_est = self._vel_alpha * self._vel_est + (1.0 - self._vel_alpha) * meas_vel
        self._t_est = now
        self._cam_t_last = now

    def _propagate_prediction(self, now):
        if self._pos_est is None or self._t_est is None:
            return False
        dt = max(0.0, now - self._t_est)
        self._pos_est = self._pos_est + self._vel_est * dt
        self._t_est = now
        if self._predict_started_t is None:
            self._predict_started_t = now
            self._predict_start_pos = self._pos_est.copy()
        return True

    def _record_velocity_sample(self, vel):
        vel = np.asarray(vel, dtype=np.float64).copy()
        if not np.all(np.isfinite(vel)):
            return
        speed = np.linalg.norm(vel[:2])
        if speed < self._vel_stability_min_speed:
            return
        self._vel_samples.append(vel)
        if len(self._vel_samples) > self._vel_stability_window:
            self._vel_samples.pop(0)

    def _velocity_is_stable(self):
        if len(self._vel_samples) < self._vel_stability_window:
            return False, None, None

        samples = np.asarray(self._vel_samples, dtype=np.float64)
        mean_vel = np.mean(samples, axis=0)
        std_vel = np.std(samples[:, :2], axis=0)
        std_norm = np.linalg.norm(std_vel)

        speed = np.linalg.norm(mean_vel[:2])
        if speed < self._vel_stability_min_speed or std_norm > self._vel_stability_std_thresh:
            return False, None, None

        direction = mean_vel.copy()
        direction_norm = np.linalg.norm(direction[:2])
        if direction_norm < 1e-9:
            return False, None, None

        direction[:2] = direction[:2] / direction_norm
        direction[2] = 0.0
        return True, mean_vel, direction

    def _lock_stable_velocity(self, now):
        stable, mean_vel, direction = self._velocity_is_stable()

        timed_out = False
        if self._wait_stable_started_t is not None:
            timed_out = (now - self._wait_stable_started_t) >= self._vel_stability_timeout

        if not stable and not timed_out:
            return False

        if stable:
            self._stable_velocity = mean_vel.copy()
            self._stable_direction = direction.copy()
        else:
            self._stable_velocity = self._vel_est.copy()
            speed = np.linalg.norm(self._stable_velocity[:2])
            if speed < self._vel_stability_min_speed:
                self._status_error = "stable_velocity_timeout_but_speed_too_low"
                return False

            self._stable_direction = self._stable_velocity.copy()
            self._stable_direction[:2] /= max(np.linalg.norm(self._stable_direction[:2]), 1e-9)
            self._stable_direction[2] = 0.0

        if self._pos_est is not None:
            self._stable_start_pos = self._pos_est.copy()
        elif self._publish_pos is not None:
            self._stable_start_pos = self._publish_pos.copy()
        elif self._bootstrap_pose is not None:
            self._stable_start_pos = self._bootstrap_pose.copy()
        else:
            self._status_error = "stable_velocity_locked_but_no_start_position"
            return False

        self._stable_start_t = now
        self._vel_est = 0.05
        self._t_est = now
        self._est_state = "tracking"
        self._publish_phase = "follow"
        self._publish_source = "stable_velocity_locked"
        self._tracking_publish_active = True

        self.get_logger().info(
            "Stable velocity locked: "
            f"vel=[{self._stable_velocity[0]:.4f}, "
            f"{self._stable_velocity[1]:.4f}, "
            f"{self._stable_velocity[2]:.4f}], "
            f"direction=[{self._stable_direction[0]:.4f}, "
            f"{self._stable_direction[1]:.4f}, "
            f"{self._stable_direction[2]:.4f}]"
        )
        return True

    def _stable_trajectory_position(self, now):
        if (
            self._stable_start_pos is None
            or self._stable_velocity is None
            or self._stable_start_t is None
        ):
            return None

        dt = max(0.0, now - self._stable_start_t)
        return self._stable_start_pos + self._stable_velocity * dt

    def _desired_future_target(self, now):
        """Predict future target."""
        if (
            self._use_locked_velocity_after_stable
            and self._tracking_publish_active
            and self._stable_velocity is not None
            and self._stable_start_pos is not None
        ):
            base_pos = self._stable_trajectory_position(now)
            if base_pos is None:
                return None

            lookahead = self._lookahead_steps * self._lookahead_dt
            lookahead = min(lookahead, self._lookahead_max_time)
            return base_pos + self._stable_velocity * lookahead

        if self._pos_est is None:
            return None

        dt_from_est = 0.0 if self._t_est is None else max(0.0, now - self._t_est)
        lookahead = self._lookahead_steps * self._lookahead_dt
        lookahead = min(lookahead, self._lookahead_max_time)
        horizon = dt_from_est + lookahead
        return self._pos_est + self._vel_est * horizon

    def _smooth_publish_target(self, desired_pos, now):
        """Smoothly move the published robot target toward desired_pos."""
        desired_pos = np.asarray(desired_pos, dtype=np.float64)

        if self._publish_pos is None:
            self._publish_pos = desired_pos.copy()
            self._publish_t_last = now
            return self._publish_pos

        dt = 0.0 if self._publish_t_last is None else max(1e-3, now - self._publish_t_last)
        alpha = 1.0 - math.exp(-dt / max(self._target_smooth_tau, 1e-3))
        next_pos = self._publish_pos + alpha * (desired_pos - self._publish_pos)

        step = next_pos - self._publish_pos
        step_norm = np.linalg.norm(step)
        if step_norm > self._target_max_step:
            next_pos = self._publish_pos + step * (self._target_max_step / step_norm)

        self._publish_pos = next_pos
        self._publish_t_last = now
        return self._publish_pos

    def _update_occlusion_state(self, now, has_measurement):
        """Track whether the object is currently hidden from the camera."""
        if has_measurement:
            self._occlusion_started_t = None
            self._occlusion_start_pos = None
            return 0.0

        if self._occlusion_started_t is None:
            self._occlusion_started_t = now
            if self._pos_est is not None:
                self._occlusion_start_pos = self._pos_est.copy()

        return now - self._occlusion_started_t

    def _prediction_limits_reached(self, now):
        """Prediction limit during object disappearance.

        Short disappearance and robot-hover occlusion are expected.
        We only stop or hold after the occlusion becomes too long or too far.
        """
        if self._predict_started_t is None or self._predict_start_pos is None:
            return False

        if self._pos_est is None:
            return False

        age = now - self._predict_started_t
        dist = np.linalg.norm(self._pos_est - self._predict_start_pos)

        # Allow expected robot occlusion longer than normal camera timeout.
        if age <= self._occlusion_expected_s and dist <= self._occlusion_max_predict_distance:
            return False

        # After hold time, prediction is no longer trusted.
        if age > self._occlusion_hold_after_s:
            return True

        # Distance safety limit.
        if dist > self._occlusion_max_predict_distance:
            return True

        return False

    def _update_publish_target(self, now):
        """Compute the desired future target and publish a smooth target.

        Never directly copy _pos_est to _publish_pos.
        Always smooth the published target. Orientation is not changed here.
        """
        desired_pos = self._desired_future_target(now)

        if desired_pos is None:
            if self._publish_pos is not None:
                self._publish_source = "magnetic_bootstrap"
            return self._publish_pos

        if self._publish_phase == "bootstrap":
            self._smooth_publish_target(desired_pos, now)

            elapsed = 0.0 if self._track_started_t is None else (now - self._track_started_t)
            err = np.linalg.norm(self._publish_pos - desired_pos)
            if err <= self._bootstrap_tol or elapsed >= self._bootstrap_max_duration:
                self._publish_phase = "follow"
            self._publish_source = "magnetic_bootstrap"

        elif self._publish_phase == "predict":
            self._smooth_publish_target(desired_pos, now)
            self._publish_source = "predicted_occlusion"

        else:
            self._publish_phase = "follow"
            self._smooth_publish_target(desired_pos, now)
            self._publish_source = "measured"

        self._last_target_pos = self._publish_pos.copy()
        return self._publish_pos

    def process(self, data):
        stamp = self.get_clock().now().to_msg()

        with self._icp_result_lock:
            pending = self._retrigger_pending
        if pending and self.icp_node.ready:
            with self.icp_node.lock:
                fitness = self.icp_node.fitness
                rmse = self.icp_node.inlier_rmse
            ok = fitness >= self._icp_min_fitness
            with self._icp_result_lock:
                self._icp_result_fitness = fitness
                self._icp_result_rmse = rmse
                self._icp_result_success = ok
                self._icp_result_ready = True
                self._retrigger_pending = False
            self._service_event.set()

        tracking_started = bool(data.get("tracking_started", False))
        tracking_stopped = bool(data.get("tracking_stopped", False))
        gate_active_raw = data.get("tracking_active", None)
        gate_active = None if gate_active_raw is None else bool(gate_active_raw)

        if tracking_started:
            self.get_logger().info('Tracker reported start-line crossing.')
            self._pending_start = True
            self._reset_track_session()
            self._pending_start = True

        if tracking_stopped:
            self.get_logger().info('Tracker reported stop-line exit.')
            self._finish_tracking("stop_line")

        if self._pending_start and gate_active and self._est_state != "tracking":
            self._log_throttled(
                "bootstrap_attempt", "info", 1.0,
                'Bootstrap attempt: gate is active and publisher is waiting to start tracking.')
            if self._start_tracking_session():
                self._pending_start = False
            else:
                self._log_throttled(
                    "bootstrap_retry", "info", 1.0,
                    'Tracking start pending, waiting for TF needed to bootstrap from magnetic_link.')

        meas_pos, meas_vel = self._extract_measurement(data)
        now = time.monotonic()

        if self._est_state == "wait_vel_stable":
            has_measurement = meas_pos is not None
            if has_measurement:
                self._fuse_measurement(meas_pos, meas_vel, now)
                self._record_velocity_sample(self._vel_est)

            locked = self._lock_stable_velocity(now)
            if not locked:
                if self._publish_pos is not None:
                    self._publish_transform(
                        stamp, self._publish_pos, self._publish_quat, publish_debug=False
                    )
                self._publish_active_flag(False)
                self._publish_status(stamp, data)
                self._log_throttled(
                    "wait_vel_stable", "info", 0.5,
                    f"Waiting for stable velocity: "
                    f"samples={len(self._vel_samples)}/{self._vel_stability_window} "
                    f"vel=[{self._vel_est[0]:.4f}, {self._vel_est[1]:.4f}, {self._vel_est[2]:.4f}]"
                )
                return data

        if self._est_state not in ("tracking",):
            passive_pos = meas_pos
            if passive_pos is not None:
                self._publish_transform(stamp, passive_pos, _IDENTITY_QUAT, publish_debug=False)
            self._publish_active_flag(False)
            self._log_throttled(
                "publisher_wait", "info", 1.0,
                f'Publisher idle: gate_active={gate_active} pending_start={self._pending_start} '
                f'has_measurement={meas_pos is not None} error="{self._status_error}"')
            self._publish_status(stamp, data)
            return data

        has_measurement = meas_pos is not None
        occlusion_age = self._update_occlusion_state(now, has_measurement)  # noqa: F841

        if has_measurement:
            if self._use_locked_velocity_after_stable and self._stable_velocity is not None:
                traj_pos = self._stable_trajectory_position(now)
                if traj_pos is not None:
                    self._pos_est = traj_pos.copy()
                    self._vel_est = self._stable_velocity.copy()
                    self._t_est = now
                    self._cam_t_last = now
            else:
                self._fuse_measurement(meas_pos, meas_vel, now)
            self._predict_started_t = None
            self._predict_start_pos = None
            if self._publish_phase != "bootstrap":
                self._publish_phase = "follow"
        else:
            # Object not visible — expected when robot hovers over the object.
            if not self._propagate_prediction(now):
                target = self._publish_pos if self._publish_pos is not None else self._bootstrap_pose
                if target is not None:
                    self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)
                self._publish_status(stamp, data)
                return data
            self._publish_phase = "predict"
            if self._prediction_limits_reached(now):
                if self._hold_target_on_occlusion_timeout:
                    # Hold last smooth target rather than stopping.
                    self._publish_source = "occlusion_hold"
                else:
                    self._finish_tracking("prediction_limit")
                    self._publish_active_flag(False)
                    self._publish_status(stamp, data)
                    return data

        publish_pos = self._update_publish_target(now)
        if publish_pos is not None:
            self._publish_target_pose(stamp, publish_pos, self._vel_est, self._publish_quat)
        else:
            self._publish_active_flag(False)

        with self._controller_ready_lock:
            _ctrl_rdy = self._controller_ready
        self._log_throttled(
            "publisher_active", "info", 1.0,
            f'Publisher active: phase={self._publish_phase} source={self._publish_source} '
            f'ctrl_ready={_ctrl_rdy} cam_fresh={meas_pos is not None} pos='
            f'[{self._publish_pos[0]:.3f}, {self._publish_pos[1]:.3f}, {self._publish_pos[2]:.3f}]')
        self._publish_status(stamp, data)
        return data

    def _publish_active_flag(self, active):
        msg = Bool()
        msg.data = bool(active)
        self._pub_active.publish(msg)

    def _publish_target_pose(self, stamp, pos_now, vel_now, quat):
        quat = _quat_normalize(quat)

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(pos_now[0])
        ps.pose.position.y = float(pos_now[1])
        ps.pose.position.z = float(pos_now[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        self._pub_pose.publish(ps)

        with self._controller_ready_lock:
            controller_ready = self._controller_ready

        # The debug TF (tracking_target) is only broadcast when the controller is
        # actively receiving data, so RViz stays in sync with what the robot sees.
        self._publish_transform(stamp, pos_now, quat, publish_debug=controller_ready)

        ts = TwistStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.frame_id
        ts.twist.linear.x = float(vel_now[0])
        ts.twist.linear.y = float(vel_now[1])
        ts.twist.linear.z = float(vel_now[2])
        self._pub_vel.publish(ts)

        world_pos, world_quat = self._target_to_world(pos_now, quat, update_error=True)
        if world_pos is not None and world_quat is not None:
            if controller_ready:
                ctrl_pose = PoseStamped()
                ctrl_pose.header.stamp = stamp
                ctrl_pose.header.frame_id = self.world_frame
                ctrl_pose.pose.position.x = float(world_pos[0])
                ctrl_pose.pose.position.y = float(world_pos[1])
                ctrl_pose.pose.position.z = float(world_pos[2])
                ctrl_pose.pose.orientation.x = float(world_quat[0])
                ctrl_pose.pose.orientation.y = float(world_quat[1])
                ctrl_pose.pose.orientation.z = float(world_quat[2])
                ctrl_pose.pose.orientation.w = float(world_quat[3])
                self._pub_ctrl_pose.publish(ctrl_pose)

                world_vel = self._vector_to_world(vel_now)
                if world_vel is not None:
                    ctrl_vel = TwistStamped()
                    ctrl_vel.header.stamp = stamp
                    ctrl_vel.header.frame_id = self.world_frame
                    ctrl_vel.twist.linear.x = float(world_vel[0])
                    ctrl_vel.twist.linear.y = float(world_vel[1])
                    ctrl_vel.twist.linear.z = float(world_vel[2])
                    self._pub_ctrl_vel.publish(ctrl_vel)
            else:
                self._log_throttled(
                    "ctrl_publish_gated", "info", 2.0,
                    "Controller not in STATE_POSE_FOLLOW — skipping controller pose/vel publish.")

        self._publish_active_flag(self._tracking_publish_active)

    def _publish_transform(self, stamp, translation, quaternion, publish_debug=True):
        # tracking_task = raw Kalman + lookahead estimate (what the sensor/filter computes).
        # This is independent of the smoothing applied to the robot command.
        # Falls back to `translation` only when there is no active state estimate (e.g. idle).
        raw_pos = self._desired_future_target(time.monotonic())
        if raw_pos is None:
            raw_pos = translation

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.tracking_frame_id
        t.transform.translation.x = float(raw_pos[0])
        t.transform.translation.y = float(raw_pos[1])
        t.transform.translation.z = float(raw_pos[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(t)

        if publish_debug:
            # tracking_target = smoothed commanded pose that the robot controller receives.
            # `translation` here is always _publish_pos (the smoothed target).
            self._publish_world_debug_transform(stamp, translation, quaternion)

    def _frame_to_world_transform(self, update_error=False, context_key="frame_to_world"):
        if self.world_frame == self.frame_id:
            return np.eye(4, dtype=np.float64)
        try:
            tf_world_frame = self._tf_buffer.lookup_transform(
                self.world_frame, self.frame_id, rclpy.time.Time())
        except TransformException as exc:
            if update_error:
                self._status_error = f"{context_key}:{exc}"
            self._log_throttled(
                f"{context_key}_fail", "warning", 1.0,
                f'Cannot transform {self.frame_id} -> {self.world_frame} for {context_key}: {exc}')
            return None
        return _transform_matrix(
            [tf_world_frame.transform.translation.x,
             tf_world_frame.transform.translation.y,
             tf_world_frame.transform.translation.z],
            [tf_world_frame.transform.rotation.x,
             tf_world_frame.transform.rotation.y,
             tf_world_frame.transform.rotation.z,
             tf_world_frame.transform.rotation.w],
        )

    def _target_to_world(self, translation, quaternion, update_error=False):
        if self.world_frame == self.frame_id:
            return np.asarray(translation, dtype=np.float64), _quat_normalize(quaternion)
        world_frame_tfm = self._frame_to_world_transform(update_error=update_error, context_key="target_world")
        if world_frame_tfm is None:
            return None, None
        frame_target_tfm = _transform_matrix(translation, quaternion)
        world_target_tfm = world_frame_tfm @ frame_target_tfm
        return world_target_tfm[:3, 3].copy(), _rot_to_quat(world_target_tfm[:3, :3])

    def _vector_to_world(self, vec):
        vec = np.asarray(vec, dtype=np.float64)
        if self.world_frame == self.frame_id:
            return vec.copy()
        world_frame_tfm = self._frame_to_world_transform(update_error=False, context_key="vel_world")
        if world_frame_tfm is None:
            return None
        return world_frame_tfm[:3, :3] @ vec

    def _publish_world_debug_transform(self, stamp, translation, quaternion):
        world_pos, world_quat = self._target_to_world(translation, quaternion, update_error=True)
        if world_pos is None or world_quat is None:
            return

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.debug_tracking_frame_id
        t.transform.translation.x = float(world_pos[0])
        t.transform.translation.y = float(world_pos[1])
        t.transform.translation.z = float(world_pos[2])
        t.transform.rotation.x = float(world_quat[0])
        t.transform.rotation.y = float(world_quat[1])
        t.transform.rotation.z = float(world_quat[2])
        t.transform.rotation.w = float(world_quat[3])
        self._tf_broadcaster.sendTransform(t)
        if not self._debug_tf_seen:
            self._debug_tf_seen = True
            self.get_logger().info(
                f'Publishing debug TF {self.world_frame} -> {self.debug_tracking_frame_id}.')

    def _publish_status(self, stamp, data):
        del stamp
        icp_state = (
            "READY" if data.get("icp_ready", False) else
            "RUNNING" if data.get("icp_running", False) else "IDLE"
        )
        tracking = "ACTIVE" if self._est_state == "tracking" else "WAIT"
        has_pose = self._pos_est is not None
        fitness = self.icp_node.fitness
        rmse = self.icp_node.inlier_rmse
        now = time.monotonic()
        dt_cam = (now - self._cam_t_last) if self._cam_t_last is not None else 999.0
        cam_ok = dt_cam < self._cam_timeout
        pred_age_ms = 0.0
        if self._predict_started_t is not None:
            pred_age_ms = (now - self._predict_started_t) * 1000.0
        occlusion_age_ms = 0.0
        if self._occlusion_started_t is not None:
            occlusion_age_ms = (now - self._occlusion_started_t) * 1000.0
        stable_speed = 0.0
        if self._stable_velocity is not None:
            stable_speed = float(np.linalg.norm(self._stable_velocity[:2]))
        status = json.dumps({
            "tracking": tracking,
            "est_state": self._est_state,
            "publish_phase": self._publish_phase,
            "publish_source": self._publish_source,
            "tracking_publish_active": self._tracking_publish_active,
            "stable_velocity_locked": self._stable_velocity is not None,
            "stable_speed_mps": round(stable_speed, 4),
            "stable_direction": (
                None if self._stable_direction is None else [
                    round(float(self._stable_direction[0]), 4),
                    round(float(self._stable_direction[1]), 4),
                    round(float(self._stable_direction[2]), 4),
                ]
            ),
            "velocity_samples": len(self._vel_samples),
            "icp": icp_state,
            "has_pose": has_pose,
            "icp_fitness": round(float(fitness), 4),
            "icp_rmse": round(float(rmse), 5),
            "cam_fresh": cam_ok,
            "cam_age_ms": round(dt_cam * 1000.0),
            "prediction_age_ms": round(pred_age_ms),
            "occlusion_age_ms": round(occlusion_age_ms),
            "error": self._status_error,
        })
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)
