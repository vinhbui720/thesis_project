import numpy as np
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class ROS2Publisher(Node):
    """
    Pipeline node — inserts after PoseFusion, before Visualize.

    Publishes every frame:
      /object/pose          geometry_msgs/PoseStamped   — 6-DOF pose (position + orientation)
      /object/velocity      geometry_msgs/TwistStamped  — linear velocity in m/s
      /object/tracking_status  std_msgs/String          — JSON-like status string
      /object/tracking_active  std_msgs/Bool            — True while inside the gate

    Service:
      /object/retrigger_icp  std_srvs/Trigger
        → forces ICP to re-run on the next frame that has enough points.
        → Response:  success=True,  message="fitness=X rmse=Y"
                     success=False, message="fitness too low: X < threshold Y"
                               or  "ICP not yet complete"
    """

    def __init__(self, cfg, icp_node):
        super().__init__('object_tracker')
        self.cfg = cfg
        self.icp_node = icp_node          # reference to PoseInitICPAsync
        self.frame_id = cfg.get("ros2_publisher", {}).get("frame_id", "camera_link")

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_pose     = self.create_publisher(PoseStamped,    '/object/pose',            10)
        self._pub_vel      = self.create_publisher(TwistStamped,   '/object/velocity',        10)
        self._pub_status   = self.create_publisher(String,          '/object/tracking_status', 10)
        self._pub_active   = self.create_publisher(Bool,            '/object/tracking_active', 10)

        # ── Service ───────────────────────────────────────────────────────
        self._srv_icp = self.create_service(
            Trigger, '/object/retrigger_icp', self._handle_retrigger_icp)

        # ── ICP re-trigger state ──────────────────────────────────────────
        self._icp_min_fitness = cfg.get("ros2_publisher", {}).get(
            "icp_min_fitness", cfg.get("init_pose", {}).get("icp_min_fitness", 0.5))
        self._retrigger_pending  = False   # set by service → consumed by process()
        self._icp_result_ready   = False   # set by process() → consumed by service response
        self._icp_result_fitness = 0.0
        self._icp_result_rmse    = 0.0
        self._icp_result_success = False
        self._icp_result_lock    = threading.Lock()
        self._service_event      = threading.Event()  # service handler waits on this

        # ── Cached rotation (used when ICP not yet ready) ─────────────────
        self._cached_rotation = np.eye(3, dtype=np.float64)

    # ======================================================================
    # SERVICE HANDLER
    # ======================================================================
    def _handle_retrigger_icp(self, request, response):
        """
        Called in a separate ROS 2 executor thread.
        Sets a flag consumed by process(), then blocks until ICP finishes.
        """
        self.get_logger().info('ICP retrigger requested')

        with self._icp_result_lock:
            self._icp_result_ready  = False
            self._retrigger_pending = True

        # Reset the ICP node state so it will run again
        with self.icp_node.lock:
            self.icp_node.ready   = False
            self.icp_node.running = False

        # Wait up to 30 s for ICP to complete
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
    # PIPELINE process()
    # ======================================================================
    def process(self, data):
        stamp = self.get_clock().now().to_msg()

        # ── Check if a pending ICP just finished ──────────────────────────
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

            self._service_event.set()   # unblock the waiting service handler

        # ── Resolve pose ──────────────────────────────────────────────────
        pose_mat = data.get("pose", None)

        # Position from Kalman tracker 3D centre (most up-to-date)
        center_3d = data.get("center_3d", None)

        if pose_mat is not None:
            R = pose_mat[:3, :3]
            t = pose_mat[:3, 3]
            self._cached_rotation = R.copy()
        elif center_3d is not None:
            # ICP not yet ready — use cached rotation + live position
            R = self._cached_rotation
            t = center_3d
        else:
            # Nothing to publish yet
            self._publish_status(stamp, data)
            return data

        quat = self._rot_to_quat(R)

        # ── Publish PoseStamped ───────────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(t[0])
        ps.pose.position.y = float(t[1])
        ps.pose.position.z = float(t[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        self._pub_pose.publish(ps)

        # ── Publish TwistStamped (linear velocity only) ───────────────────
        velocity = data.get("velocity", None)
        if velocity is not None:
            vx_m, vy_m = velocity
            ts = TwistStamped()
            ts.header.stamp    = stamp
            ts.header.frame_id = self.frame_id
            ts.twist.linear.x  = float(vx_m)
            ts.twist.linear.y  = float(vy_m)
            ts.twist.linear.z  = 0.0
            self._pub_vel.publish(ts)

        # ── Publish tracking active flag ──────────────────────────────────
        active_msg = Bool()
        active_msg.data = bool(data.get("tracking_active", False))
        self._pub_active.publish(active_msg)

        # ── Publish status string ─────────────────────────────────────────
        self._publish_status(stamp, data)

        return data

    # ======================================================================
    # HELPERS
    # ======================================================================
    def _publish_status(self, stamp, data):
        icp_state = ("READY"   if data.get("icp_ready",   False) else
                     "RUNNING" if data.get("icp_running", False) else "IDLE")
        tracking  = "ACTIVE" if data.get("tracking_active", False) else "WAIT"
        has_pose  = "pose" in data
        fitness   = self.icp_node.fitness
        rmse      = self.icp_node.inlier_rmse

        status = (f'{{"tracking":"{tracking}",'
                  f'"icp":"{icp_state}",'
                  f'"has_pose":{str(has_pose).lower()},'
                  f'"icp_fitness":{fitness:.4f},'
                  f'"icp_rmse":{rmse:.5f}}}')

        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    @staticmethod
    def _rot_to_quat(R):
        """Rotation matrix → quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w], dtype=np.float64)
