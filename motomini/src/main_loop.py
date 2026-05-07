#!/usr/bin/env python3

import threading
import time
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Bool, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GantryControl:
    """Gantry controller helper: publish trajectory commands to gantry."""

    def __init__(self, node: Node):
        self._node = node
        self._pub_gantry_traj = node.create_publisher(
            JointTrajectory, "/gantry_controller/joint_trajectory", 10
        )

    def move_to(
        self,
        x_pos: float,
        z_pos: float,
        motion_time_sec: float = 1.0,
    ) -> None:
        """
        Publish a trajectory command to move the gantry to (x, z).

        Args:
            x_pos: X-axis position in meters (-0.28 to 0.0).
            z_pos: Z-axis position in meters (-0.06 to 0.0).
            motion_time_sec: Time for the motion to complete.
        """
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names = ["joint_x", "joint_z"]

        point = JointTrajectoryPoint()
        point.positions = [float(x_pos), float(z_pos)]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=int(motion_time_sec), nanosec=0)

        msg.points.append(point)
        self._pub_gantry_traj.publish(msg)
        self._node.get_logger().info(f"Gantry move published: x={x_pos:.3f}, z={z_pos:.3f}")

    def move_to_origin(self) -> None:
        """Move gantry back to origin (0.0, 0.0)."""
        self.move_to(0.0, 0.0, motion_time_sec=2.0)

    def loop_motion(
        self,
        x_start: float,
        x_end: float,
        z_start: float,
        z_end: float,
        steps: int = 28,
        publish_period_sec: float = 0.05,
        motion_time_sec: float = 0.12,
        loop_period_sec: float = 16.0,
    ) -> None:
        """
        Send loop motion parameters to the gantry loop controller node.
        This publishes a single command that the loop controller uses.

        Args:
            x_start, x_end: X-axis range in meters.
            z_start, z_end: Z-axis range in meters.
            steps: Number of steps in the loop.
            publish_period_sec: Interval between trajectory updates.
            motion_time_sec: Time for each motion segment.
            loop_period_sec: Total loop cycle time.
        """
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names = ["joint_x", "joint_z"]

        point = JointTrajectoryPoint()
        # Single point defines the loop boundaries
        point.positions = [float(x_end), float(z_end)]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=int(loop_period_sec), nanosec=0)

        msg.points.append(point)
        self._pub_gantry_traj.publish(msg)
        self._node.get_logger().info(
            f"Gantry loop motion published: x=[{x_start:.3f}, {x_end:.3f}], "
            f"z=[{z_start:.3f}, {z_end:.3f}], loop_period={loop_period_sec}s"
        )

class ObjectProcess:
    """Geometry helpers for generating structured PoseArrays."""

    @staticmethod
    def generate_circle_poses(
        x: float,
        y: float,
        z: float,
        number_of_points: int,
        radius: float,
        alpha_degrees: float,
    ) -> PoseArray:
        """
        Generate poses on the edge of a circle with orientation.

        Each pose faces radially outward from the circle centre, rotated by
        ``alpha_degrees`` around the perpendicular XY axis so the tool can
        approach the surface at an angle.

        Args:
            x, y, z:          Centre of the circle (world frame).
            number_of_points: How many evenly-spaced poses to generate.
            radius:           Circle radius (metres).
            alpha_degrees:    Tilt angle for the approach orientation.

        Returns:
            PoseArray ready to pass to MotoMiniMainLoop.execute_target_pose().
        """
        center = np.array([x, y, z], dtype=float)
        alpha_rad = np.radians(alpha_degrees)

        pose_array = PoseArray()

        for i in range(number_of_points):
            theta = 2 * np.pi * i / number_of_points

            pos = np.array([
                x + radius * np.cos(theta),
                y + radius * np.sin(theta),
                z,
            ], dtype=float)

            relative_vec = pos - center
            dx, dy = relative_vec[0], relative_vec[1]

            if np.isclose(dx, 0.0) and np.isclose(dy, 0.0):
                new_rot = R.identity()
            else:
                axis_vector = np.array([-dy, dx, 0.0])
                axis_normalized = axis_vector / np.linalg.norm(axis_vector)
                new_rot = R.from_rotvec(alpha_rad * axis_normalized)

            # scipy returns [qx, qy, qz, qw]
            qx, qy, qz, qw = new_rot.as_quat()

            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            pose_array.poses.append(pose)

        return pose_array

    @staticmethod
    def wait_for_tracking_status_change(node: Node, timeout_sec: float = 10.0) -> bool:
        """
        Listen for /object/tracking_status topic and wait for "tracking":"WAIT" to
        transition to "tracking":"ACTIVE".

        Args:
            node: ROS2 node to use for subscription.
            timeout_sec: Maximum time to wait for the transition.

        Returns:
            True if the transition occurs within the timeout, False otherwise.
        """
        status_lock = threading.Lock()
        last_status = ""

        def status_callback(msg):
            nonlocal last_status
            with status_lock:
                last_status = msg.data

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)

        start_time = time.time()
        saw_wait = False

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status

            if "\"tracking\":\"WAIT\"" in status:
                saw_wait = True
            elif saw_wait and "\"tracking\":\"ACTIVE\"" in status:
                node.get_logger().info("Transition detected: WAIT -> ACTIVE")
                return True

            time.sleep(0.05)

        node.get_logger().error("Timeout waiting for tracking status transition")
        return False

class ObjectControl:
	"""Object TF tester helper: publish target pose and attach/detach signal."""

	def __init__(self, node: Node):
		self._node = node
		self._pub_object_pose = node.create_publisher(PoseStamped, "/target_object_pose", 10)
		self._pub_attach = node.create_publisher(Bool, "/object_attach_signal", 10)

	def set_current_target(
		self,
		x: float,
		y: float,
		z: float,
		qx: float = 0.0,
		qy: float = 0.0,
		qz: float = 0.0,
		qw: float = 1.0,
	) -> None:
		msg = PoseStamped()
		msg.header.frame_id = "world"
		msg.header.stamp = self._node.get_clock().now().to_msg()
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = float(z)
		msg.pose.orientation.x = float(qx)
		msg.pose.orientation.y = float(qy)
		msg.pose.orientation.z = float(qz)
		msg.pose.orientation.w = float(qw)

		self._pub_object_pose.publish(msg)
		self._node.get_logger().info("Published /target_object_pose")

	def attach(self) -> None:
		msg = Bool()
		msg.data = True
		self._pub_attach.publish(msg)
		self._node.get_logger().info("Published /object_attach_signal=true")

	def detach(self) -> None:
		msg = Bool()
		msg.data = False
		self._pub_attach.publish(msg)
		self._node.get_logger().info("Published /object_attach_signal=false")

class MotoMiniMainLoop(Node):
	def __init__(self):
		super().__init__("motomini_main_loop")

		self.pub_targets = self.create_publisher(PoseArray, "/target_poses", 10)
		self.pub_start = self.create_publisher(Bool, "/start", 10)
		self.pub_clear = self.create_publisher(Bool, "/clear_targets", 10)
		self.param_client = self.create_client(SetParameters, "/motomini_planning_node/set_parameters")

		self.create_subscription(String, "/optimization_status", self._status_cb, 10)
		self.create_subscription(Bool, "/trajectory_executing", self._executing_cb, 10)

		self._status_lock = threading.Lock()
		self._last_status = ""

		self._exec_lock = threading.Lock()
		self._exec_seen_true = False
		self._exec_is_true = False

	def _load_mode_params(self, mode_case: int):
		"""Load planner params from the same YAML presets used by manual_controller.py."""
		mode_to_file = {
			1: "planning_params.yaml",
			2: "planning_params_longdistance.yaml",
			3: "planning_params_grinding.yaml",
		}

		filename = mode_to_file.get(mode_case)
		if filename is None:
			self.get_logger().error("Invalid mode case. Use 1=Default, 2=Long Distance, 3=Grinding")
			return None

		search_dirs = [
			Path.home() / "vinh_ws" / "install" / "robot_planning" / "share" / "robot_planning" / "config",
			Path.home() / "vinh_ws" / "src" / "thesis_project" / "robot_planning" / "config",
			Path("/home/vinbui/vinh_ws/src/thesis_project/robot_planning/config"),
		]

		for directory in search_dirs:
			config_path = directory / filename
			if not config_path.exists():
				continue

			try:
				with open(config_path, "r", encoding="utf-8") as f:
					data = yaml.safe_load(f)
			except Exception as exc:
				self.get_logger().error(f"Failed to read {config_path}: {exc}")
				return None

			if data and "motomini_planning_node" in data:
				params = data["motomini_planning_node"].get("ros__parameters", {})
				self.get_logger().info(f"Loaded planner preset from {config_path}")
				return params

		self.get_logger().error(f"Could not find valid preset file: {filename}")
		return None

	def set_planner_mode(self, mode_case: int, timeout_sec: float = 5.0) -> bool:
		"""Apply planner preset by case id. 1=Default, 2=Long Distance, 3=Grinding."""
		params_dict = self._load_mode_params(mode_case)
		if params_dict is None:
			return False

		if not self.param_client.wait_for_service(timeout_sec=timeout_sec):
			self.get_logger().error("Service /motomini_planning_node/set_parameters is not available")
			return False

		req = SetParameters.Request()
		for name, value in params_dict.items():
			pv = ParameterValue()
			if isinstance(value, bool):
				pv.type = ParameterType.PARAMETER_BOOL
				pv.bool_value = value
			elif isinstance(value, int):
				pv.type = ParameterType.PARAMETER_INTEGER
				pv.integer_value = value
			elif isinstance(value, float):
				pv.type = ParameterType.PARAMETER_DOUBLE
				pv.double_value = value
			elif isinstance(value, str):
				pv.type = ParameterType.PARAMETER_STRING
				pv.string_value = value
			else:
				continue

			p = Parameter()
			p.name = name
			p.value = pv
			req.parameters.append(p)

		future = self.param_client.call_async(req)
		deadline = time.time() + timeout_sec
		while rclpy.ok() and (not future.done()) and time.time() < deadline:
			time.sleep(0.05)

		if not future.done():
			self.get_logger().error("Timed out waiting for set_parameters response")
			return False

		try:
			result = future.result()
		except Exception as exc:
			self.get_logger().error(f"set_parameters call failed: {exc}")
			return False

		failed = [r for r in result.results if not r.successful]
		if failed:
			reasons = "; ".join((r.reason or "unknown") for r in failed)
			self.get_logger().error(f"Planner mode apply failed: {reasons}")
			return False

		self.get_logger().info(f"Planner mode {mode_case} applied successfully")
		return True

	def _status_cb(self, msg: String) -> None:
		with self._status_lock:
			self._last_status = msg.data
		self.get_logger().info(f"/optimization_status: {msg.data}")

	def _executing_cb(self, msg: Bool) -> None:
		with self._exec_lock:
			self._exec_is_true = bool(msg.data)
			if msg.data:
				self._exec_seen_true = True
		self.get_logger().info(f"/trajectory_executing: {msg.data}")

	def _reset_execution_wait_state(self) -> None:
		with self._exec_lock:
			self._exec_seen_true = False

	def _stamp_pose_array(self, targets: PoseArray) -> PoseArray:
		targets.header.frame_id = "world"
		targets.header.stamp = self.get_clock().now().to_msg()
		return targets

	@staticmethod
	def make_pose(
		x: float,
		y: float,
		z: float,
		qx: float = 0.0,
		qy: float = 0.0,
		qz: float = 0.0,
		qw: float = 1.0,
	) -> Pose:
		pose = Pose()
		pose.position.x = float(x)
		pose.position.y = float(y)
		pose.position.z = float(z)
		pose.orientation.x = float(qx)
		pose.orientation.y = float(qy)
		pose.orientation.z = float(qz)
		pose.orientation.w = float(qw)
		return pose

	def make_targets(self, waypoints: Iterable[Tuple[float, ...]]) -> PoseArray:
		"""
		Create PoseArray from compact tuples:
		- (x, y, z)
		- (x, y, z, qx, qy, qz, qw)
		"""
		targets = PoseArray()
		for i, wp in enumerate(waypoints):
			if len(wp) == 3:
				targets.poses.append(self.make_pose(wp[0], wp[1], wp[2]))
			elif len(wp) == 7:
				targets.poses.append(
					self.make_pose(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5], wp[6])
				)
			else:
				raise ValueError(
					f"Waypoint #{i} must have 3 or 7 values, got {len(wp)}"
				)
		return targets

	def _publish_start(self) -> None:
		msg = Bool()
		msg.data = True
		self.pub_start.publish(msg)
		self.get_logger().info("Published /start true")

	def _publish_clear(self) -> None:
		msg = Bool()
		msg.data = True
		self.pub_clear.publish(msg)
		self.get_logger().info("Published /clear_targets true")

	def execute_target_pose(self, targets: PoseArray) -> None:
		"""Execute target poses: publish PoseArray -> wait -> start -> wait done -> clear."""
		if len(targets.poses) == 0:
			raise ValueError("targets PoseArray is empty")

		self._reset_execution_wait_state()

		target = self._stamp_pose_array(targets)
		self.pub_targets.publish(target)
		self.get_logger().info(f"Published {len(target.poses)} target pose(s) on /target_poses")

		expected_status = f"Accumulating Poses: {len(target.poses)}"
		if not self.wait_for_status_contains(expected_status, timeout_sec=10.0):
			raise RuntimeError(f"Did not receive status '{expected_status}' in time")

		self._publish_start()

		if not self.wait_for_exec_true_then_false(timeout_sec=120.0):
			raise RuntimeError("Did not observe /trajectory_executing transition true -> false in time")

		self._publish_clear()
		self.get_logger().info("Main loop complete")

	def wait_for_status_contains(self, text: str, timeout_sec: float) -> bool:
		deadline = time.time() + timeout_sec
		while rclpy.ok() and time.time() < deadline:
			with self._status_lock:
				s = self._last_status
			if text in s:
				self.get_logger().info(f"Matched status: {s}")
				return True
			time.sleep(0.05)
		return False

	def wait_for_exec_true_then_false(self, timeout_sec: float) -> bool:
		deadline = time.time() + timeout_sec
		while rclpy.ok() and time.time() < deadline:
			with self._exec_lock:
				seen_true = self._exec_seen_true
				is_true = self._exec_is_true

			if seen_true and (not is_true):
				self.get_logger().info("Observed /trajectory_executing transition true -> false")
				return True

			time.sleep(0.05)
		return False


def main() -> None:
	rclpy.init()
	node = MotoMiniMainLoop()
	object_control = ObjectControl(node)
	gantry_control = GantryControl(node)
	executor = rclpy.executors.SingleThreadedExecutor()
	executor.add_node(node)

	spin_thread = threading.Thread(target=executor.spin, daemon=True)
	spin_thread.start()

	try:
		# Give discovery/subscriptions a short moment to connect.
		time.sleep(0.5)
		gantry_control.move_to(x_pos=-0.14, z_pos=-0.03, motion_time_sec=1.5)
		# node._publish_clear()

		# node.set_planner_mode(2)
		# object_control.set_current_target(-0.05, -0.25, 0.07)
		# targets = node.make_targets([
        #     (-0.03, -0.24, 0.2),
        #     (-0.03, -0.24, 0.071)
        # ])
		# node.execute_target_pose(targets)
		# object_control.attach()
		# time.sleep(0.5)
		# #Moving to hover ready to Grinding
		# targets = node.make_targets([
        #     (0.1, -0.25, 0.2)
        # ])
		# node.execute_target_pose(targets)
		# node.set_planner_mode(3)
		# circle_targets = ObjectProcess.generate_circle_poses(
		# 	x=0.1, y=-0.25, z=0.2,
		# 	number_of_points=10,
		# 	radius=0.02,
		# 	alpha_degrees=-45.0,
		# )
		# node.execute_target_pose(circle_targets)
		# # moving hove ready for tracking
		# targets = node.make_targets([
        #     (0.18, 0.00, 0.245)
        # ])
		# node.execute_target_pose(targets)
		# targets = node.make_targets([
        #     (0.25, -0.2, 0.15)
        # ])
		# node.execute_target_pose(targets)
		# sucess = ObjectProcess.wait_for_tracking_status_change(node, timeout_sec=15.0)
		# if sucess:
		# 	print("Tracking status transition detected successfully.")
		# else:
		# 	print("Failed to detect tracking status transition in time.")
		# object_control.detach()
		# targets = node.make_targets([
        #     (0.18, 0.00, 0.245)
        # ])
		# node.execute_target_pose(targets)
	except Exception as exc:
		node.get_logger().error(f"Main loop failed: {exc}")
	finally:
		executor.shutdown()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
