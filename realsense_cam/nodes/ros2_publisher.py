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

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


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

        self._vel_alpha = ros_cfg.get("vel_smooth_alpha", 0.85)
        self._pos_alpha = ros_cfg.get("pos_correct_alpha", 0.30)
        self._cam_timeout = ros_cfg.get("cam_timeout_s", 0.25)
        self._target_z_offset = ros_cfg.get("target_z_offset_m", 0.01)
        self._bootstrap_gain = ros_cfg.get("bootstrap_gain", 12.0)
        self._bootstrap_tol = ros_cfg.get("bootstrap_pos_tolerance_m", 0.01)
        self._bootstrap_max_duration = ros_cfg.get("bootstrap_max_duration_s", 0.35)
        self._prediction_max_time = ros_cfg.get("prediction_max_time_s", 0.75)
        self._prediction_max_distance = ros_cfg.get("prediction_max_distance_m", 0.25)

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

        self._pub_pose = self.create_publisher(PoseStamped, '/tracking_target_pose', 10)
        self._pub_vel = self.create_publisher(TwistStamped, '/object/velocity', 10)
        self._pub_status = self.create_publisher(String, '/object/tracking_status', 10)
        self._pub_active = self.create_publisher(Bool, '/object/tracking_active', 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sub_feedback = self.create_subscription(
            Twist, '/motomini/feedback', self._feedback_cb, 10)

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

    def _reset_track_session(self):
        self._reset_estimator()
        self._est_state = "idle"
        self._publish_phase = "wait"
        self._publish_source = "none"
        self._status_error = ""
        self._bootstrap_pose = None
        self._bootstrap_quat = _IDENTITY_QUAT.copy()
        self._publish_pos = None
        self._publish_quat = _IDENTITY_QUAT.copy()
        self._track_started_t = None
        self._last_target_pos = None
        self._publish_t_last = None

    def _finish_tracking(self, reason):
        self._est_state = "done"
        self._publish_phase = "done"
        self._publish_source = reason
        self._status_error = ""
        self._pending_start = False

    def _try_lookup_bootstrap_pose(self):
        try:
            tf_world_mag = self._tf_buffer.lookup_transform(
                self.world_frame, self.magnetic_link_frame, rclpy.time.Time())
        except TransformException as exc:
            self._status_error = f"bootstrap_tf:{exc}"
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
        self._est_state = "tracking"
        self._publish_phase = "bootstrap"
        self._publish_source = "magnetic_bootstrap"
        self._status_error = ""
        self._bootstrap_pose = bootstrap_pos
        self._bootstrap_quat = bootstrap_quat
        self._publish_pos = bootstrap_pos.copy()
        self._publish_quat = bootstrap_quat.copy()
        self._track_started_t = time.monotonic()
        self._last_target_pos = bootstrap_pos.copy()
        self._publish_t_last = self._track_started_t
        self.get_logger().info('Start-line crossed — bootstrap from magnetic_link.')
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

    def _prediction_limits_reached(self, now):
        if self._predict_started_t is None or self._predict_start_pos is None:
            return False
        age = now - self._predict_started_t
        dist = np.linalg.norm(self._pos_est - self._predict_start_pos)
        return age > self._prediction_max_time or dist > self._prediction_max_distance

    def _update_publish_target(self, now):
        if self._pos_est is None:
            if self._publish_pos is not None:
                self._publish_source = "magnetic_bootstrap"
            return self._publish_pos

        target_pos = self._pos_est.copy()
        if self._publish_phase == "bootstrap":
            if self._publish_pos is None:
                self._publish_pos = target_pos.copy()
            else:
                dt = 0.0 if self._publish_t_last is None else max(0.0, now - self._publish_t_last)
                blend = 1.0 - math.exp(-self._bootstrap_gain * max(dt, 1e-3))
                self._publish_pos = self._publish_pos + blend * (target_pos - self._publish_pos)

            elapsed = 0.0 if self._track_started_t is None else (now - self._track_started_t)
            err = np.linalg.norm(self._publish_pos - target_pos)
            if err <= self._bootstrap_tol or elapsed >= self._bootstrap_max_duration:
                self._publish_phase = "follow"
            self._publish_source = "magnetic_bootstrap"
        elif self._publish_phase == "predict":
            self._publish_pos = target_pos.copy()
            self._publish_source = "predicted"
        else:
            self._publish_phase = "follow"
            self._publish_pos = target_pos.copy()
            self._publish_source = "measured"

        self._last_target_pos = self._publish_pos.copy()
        self._publish_t_last = now
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

        tracking_started = data.get("tracking_started", False)
        tracking_stopped = data.get("tracking_stopped", False)
        gate_active = data.get("tracking_active", None)

        if tracking_started:
            self._pending_start = True
            self._reset_track_session()
            self._pending_start = True

        if tracking_stopped:
            self._finish_tracking("stop_line")

        if self._pending_start and gate_active is True and self._est_state != "tracking":
            if self._start_tracking_session():
                self._pending_start = False

        meas_pos, meas_vel = self._extract_measurement(data)
        now = time.monotonic()

        if self._est_state != "tracking":
            passive_pos = meas_pos
            if passive_pos is not None:
                self._publish_transform(stamp, passive_pos, _IDENTITY_QUAT)
            self._publish_active_flag(False)
            self._publish_status(stamp, data)
            return data

        if meas_pos is not None:
            self._fuse_measurement(meas_pos, meas_vel, now)
            self._predict_started_t = None
            self._predict_start_pos = None
            if self._publish_phase != "bootstrap":
                self._publish_phase = "follow"
        else:
            if not self._propagate_prediction(now):
                target = self._publish_pos if self._publish_pos is not None else self._bootstrap_pose
                if target is not None:
                    self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)
                self._publish_status(stamp, data)
                return data
            self._publish_phase = "predict"
            if self._prediction_limits_reached(now):
                self._finish_tracking("prediction_limit")
                self._publish_active_flag(False)
                self._publish_status(stamp, data)
                return data

        publish_pos = self._update_publish_target(now)
        if publish_pos is not None:
            self._publish_target_pose(stamp, publish_pos, self._vel_est, self._publish_quat)
        else:
            self._publish_active_flag(False)

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

        self._publish_transform(stamp, pos_now, quat)

        ts = TwistStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.frame_id
        ts.twist.linear.x = float(vel_now[0])
        ts.twist.linear.y = float(vel_now[1])
        ts.twist.linear.z = float(vel_now[2])
        self._pub_vel.publish(ts)

        self._publish_active_flag(True)

    def _publish_transform(self, stamp, translation, quaternion):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.tracking_frame_id
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(t)
        self._publish_world_debug_transform(stamp, translation, quaternion)

    def _publish_world_debug_transform(self, stamp, translation, quaternion):
        if self.world_frame == self.frame_id:
            world_pos = np.asarray(translation, dtype=np.float64)
            world_quat = _quat_normalize(quaternion)
        else:
            try:
                tf_world_frame = self._tf_buffer.lookup_transform(
                    self.world_frame, self.frame_id, rclpy.time.Time())
            except TransformException as exc:
                self._status_error = f"debug_tf:{exc}"
                return

            world_frame_tfm = _transform_matrix(
                [tf_world_frame.transform.translation.x,
                 tf_world_frame.transform.translation.y,
                 tf_world_frame.transform.translation.z],
                [tf_world_frame.transform.rotation.x,
                 tf_world_frame.transform.rotation.y,
                 tf_world_frame.transform.rotation.z,
                 tf_world_frame.transform.rotation.w],
            )
            frame_target_tfm = _transform_matrix(translation, quaternion)
            world_target_tfm = world_frame_tfm @ frame_target_tfm
            world_pos = world_target_tfm[:3, 3].copy()
            world_quat = _rot_to_quat(world_target_tfm[:3, :3])

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
        status = json.dumps({
            "tracking": tracking,
            "est_state": self._est_state,
            "publish_phase": self._publish_phase,
            "publish_source": self._publish_source,
            "icp": icp_state,
            "has_pose": has_pose,
            "icp_fitness": round(float(fitness), 4),
            "icp_rmse": round(float(rmse), 5),
            "cam_fresh": cam_ok,
            "cam_age_ms": round(dt_cam * 1000.0),
            "prediction_age_ms": round(pred_age_ms),
            "error": self._status_error,
        })
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)
