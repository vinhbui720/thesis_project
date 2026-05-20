#!/usr/bin/env python3

import json
import threading
import time
from pathlib import Path
from typing import Iterable, Tuple, Optional, Dict, Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as ROSDuration
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger

class GantryControl:
    """Gantry controller helper: publish trajectory commands to gantry."""

    def __init__(self, node: Node):
        self._node = node
        self._pub_gantry_traj = node.create_publisher(
            JointTrajectory, "/gantry_controller/joint_trajectory", 10
        )
        
        # Joint state feedback
        self._joint_state_lock = threading.Lock()
        self._joint_state = None
        self._sub_joint_state = node.create_subscription(
            JointState, "/gantry_joint_states", self._joint_state_cb, 10
        )
    
    def _joint_state_cb(self, msg: JointState) -> None:
        """Callback for joint state feedback."""
        with self._joint_state_lock:
            self._joint_state = msg

    def move_to(
        self,
        x_pos: float,
        z_pos: float,
        motion_time_sec: float = 1.0,
        wait_timeout_sec: float = 10.0,
        position_tolerance_m: float = 0.001,
    ) -> bool:
        """
        Publish a trajectory command to move the gantry to (x, z) and wait for feedback.

        Args:
            x_pos: X-axis position in meters (-0.28 to 0.0).
            z_pos: Z-axis position in meters (-0.06 to 0.0).
            motion_time_sec: Time for the motion to complete.
            wait_timeout_sec: Maximum time to wait for target position feedback (seconds).
            position_tolerance_m: Position tolerance for considering target reached (meters).
            
        Returns:
            True if target position was reached within timeout, False otherwise.
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
        
        # Wait for feedback to reach target position
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < wait_timeout_sec):
            with self._joint_state_lock:
                joint_state = self._joint_state
            
            if joint_state is not None and len(joint_state.position) >= 2:
                # Assuming first position is joint_x, second is joint_z
                current_x = joint_state.position[0]
                current_z = joint_state.position[1]
                
                error_x = abs(current_x - x_pos)
                error_z = abs(current_z - z_pos)
                
                if error_x <= position_tolerance_m and error_z <= position_tolerance_m:
                    self._node.get_logger().info(
                        f"Gantry reached target: x={current_x:.6f}, z={current_z:.6f} "
                        f"(errors: dx={error_x:.6f}, dz={error_z:.6f})"
                    )
                    return True
            
            time.sleep(0.05)
        
        self._node.get_logger().error(
            f"Gantry failed to reach target x={x_pos:.3f}, z={z_pos:.3f} "
            f"within timeout {wait_timeout_sec}s"
        )
        return False

    def move_to_origin(self, wait_timeout_sec: float = 10.0) -> bool:
        """Move gantry back to origin (0.0, 0.0).
        
        Args:
            wait_timeout_sec: Maximum time to wait for target position feedback (seconds).
            
        Returns:
            True if target position was reached, False otherwise.
        """
        return self.move_to(0.0, 0.0, motion_time_sec=2.0, wait_timeout_sec=wait_timeout_sec)

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
    def _parse_tracking_status(status_json_str: str) -> Optional[Dict[str, Any]]:
        """
        Parse the JSON tracking status string from /object/tracking_status.
        
        Args:
            status_json_str: JSON string from the status message.
            
        Returns:
            Dict with parsed status fields, or None if parse fails.
        """
        try:
            return json.loads(status_json_str)
        except (json.JSONDecodeError, ValueError) as e:
            return None

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
        last_status = {}

        def status_callback(msg):
            nonlocal last_status
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)

        start_time = time.time()
        saw_wait = False

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status.copy() if last_status else {}

            tracking_state = status.get("tracking", "")
            
            if tracking_state == "WAIT":
                saw_wait = True
            elif saw_wait and tracking_state == "ACTIVE":
                node.get_logger().info("Transition detected: WAIT -> ACTIVE")
                return True

            time.sleep(0.05)

        node.get_logger().error("Timeout waiting for tracking status transition")
        return False

    @staticmethod
    def wait_for_tracking_active(node: Node, timeout_sec: float = 10.0) -> bool:
        """
        Wait for tracking state to become ACTIVE (without requiring WAIT first).
        Useful for cases where the system is already in WAIT state.
        
        Args:
            node: ROS2 node to use for subscription.
            timeout_sec: Maximum time to wait for ACTIVE state.
            
        Returns:
            True if tracking becomes ACTIVE within timeout, False otherwise.
        """
        status_lock = threading.Lock()
        last_status = {}

        def status_callback(msg):
            nonlocal last_status
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status.copy() if last_status else {}

            if status.get("tracking") == "ACTIVE":
                node.get_logger().info("Tracking is now ACTIVE")
                return True

            time.sleep(0.05)

        node.get_logger().error(f"Timeout waiting for tracking ACTIVE state")
        return False

    @staticmethod
    def check_icp_fitness_ok(
        node: Node,
        min_fitness: float = 0.5,
        timeout_sec: float = 5.0
    ) -> bool:
        """
        Monitor /object/tracking_status and check if ICP fitness is above threshold.
        
        Args:
            node: ROS2 node to use for subscription.
            min_fitness: Minimum acceptable fitness threshold (0.0 - 1.0).
            timeout_sec: Maximum time to wait for a good fitness reading.
            
        Returns:
            True if ICP fitness exceeds threshold, False otherwise.
        """
        status_lock = threading.Lock()
        last_status = {}

        def status_callback(msg):
            nonlocal last_status
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status.copy() if last_status else {}

            fitness = status.get("icp_fitness", 0.0)
            if fitness >= min_fitness:
                node.get_logger().info(f"ICP fitness OK: {fitness:.4f} >= {min_fitness:.4f}")
                return True

            time.sleep(0.05)

        node.get_logger().error(
            f"ICP fitness failed to reach {min_fitness:.4f} within {timeout_sec}s"
        )
        return False

    @staticmethod
    def check_tracking_error_ok(
        node: Node,
        timeout_sec: float = 5.0
    ) -> bool:
        """
        Check if tracking error is empty (no error state reported).
        
        Args:
            node: ROS2 node to use for subscription.
            timeout_sec: Maximum time to wait for a clean status.
            
        Returns:
            True if error field is empty, False if an error persists.
        """
        status_lock = threading.Lock()
        last_status = {}

        def status_callback(msg):
            nonlocal last_status
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status.copy() if last_status else {}

            error = status.get("error", "")
            if not error:  # Empty error string means no error
                node.get_logger().info("Tracking error cleared")
                return True

            node.get_logger().warning(f"Tracking error: {error}")
            time.sleep(0.05)

        node.get_logger().error(
            f"Tracking error persisted for {timeout_sec}s. "
            f"Last error: {status.get('error', 'unknown')}"
        )
        return False

    @staticmethod
    def retrigger_icp(node: Node, timeout_sec: float = 35.0) -> bool:
        """
        Call the /object/retrigger_icp service to manually trigger ICP realignment.
        
        Args:
            node: ROS2 node to use for service call.
            timeout_sec: Maximum wait time for the service to respond.
            
        Returns:
            True if ICP retrigger succeeded, False otherwise.
        """
        client = node.create_client(Trigger, "/object/retrigger_icp")
        
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(
                "Service /object/retrigger_icp is not available"
            )
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        node.get_logger().info(
                            f"ICP retrigger succeeded: {response.message}"
                        )
                        return True
                    else:
                        node.get_logger().error(
                            f"ICP retrigger failed: {response.message}"
                        )
                        return False
                except Exception as e:
                    node.get_logger().error(f"ICP retrigger call failed: {e}")
                    return False
            time.sleep(0.1)

        node.get_logger().error(
            f"ICP retrigger timed out after {timeout_sec}s"
        )
        return False

    @staticmethod
    def get_tracking_status(node: Node, timeout_sec: float = 2.0) -> Optional[Dict[str, Any]]:
        """
        Get the current tracking status snapshot.
        
        Args:
            node: ROS2 node to use for subscription.
            timeout_sec: Maximum time to wait for first status message.
            
        Returns:
            Dict with parsed status fields, or None if timed out.
        """
        status_lock = threading.Lock()
        last_status = [None]  # Use list to allow modification in nested function

        def status_callback(msg):
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status[0] = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                if last_status[0] is not None:
                    return last_status[0]
            time.sleep(0.02)

        node.get_logger().warning("Timeout getting tracking status")
        return None

    @staticmethod
    def wait_for_camera_fresh(
        node: Node,
        timeout_sec: float = 10.0,
        max_age_ms: float = 250.0
    ) -> bool:
        """
        Wait for camera measurements to be fresh (recently updated).
        
        Args:
            node: ROS2 node to use for subscription.
            timeout_sec: Maximum time to wait.
            max_age_ms: Maximum acceptable camera age in milliseconds.
            
        Returns:
            True if camera becomes fresh, False if timeout.
        """
        status_lock = threading.Lock()
        last_status = {}

        def status_callback(msg):
            nonlocal last_status
            parsed = ObjectProcess._parse_tracking_status(msg.data)
            if parsed:
                with status_lock:
                    last_status = parsed

        sub = node.create_subscription(String, "/object/tracking_status", status_callback, 10)
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < timeout_sec):
            with status_lock:
                status = last_status.copy() if last_status else {}

            if status.get("cam_fresh") is True:
                cam_age = status.get("cam_age_ms", 0)
                node.get_logger().info(
                    f"Camera is fresh (age: {cam_age:.1f}ms <= {max_age_ms:.1f}ms)"
                )
                return True

            time.sleep(0.05)

        node.get_logger().error(f"Camera did not become fresh within {timeout_sec}s")
        return False

    @staticmethod
    def start_tracking_publish(node: Node) -> None:
        """Signal the tracking publisher to start publishing to the robot controller.

        This should be called after the controller has confirmed STATE_POSE_FOLLOW.
        
        Args:
            node: ROS2 node to use for publisher.
        """
        msg = Bool()
        msg.data = True
        pub = node.create_publisher(Bool, "/object/start_publish_tracking", 10)
        pub.publish(msg)
        node.get_logger().info("Published /object/start_publish_tracking=True")

    @staticmethod
    def stop_tracking_publish(node: Node) -> None:
        """Signal the tracking publisher to stop publishing to the robot controller.

        Args:
            node: ROS2 node to use for publisher.
        """
        msg = Bool()
        msg.data = False
        pub = node.create_publisher(Bool, "/object/start_publish_tracking", 10)
        pub.publish(msg)
        node.get_logger().info("Published /object/start_publish_tracking=False")

class ObjectControl:
	"""Object TF tester helper: publish target pose and attach/detach signal."""

	def __init__(self, node: Node):
		self._node = node
		self._pub_object_pose = node.create_publisher(PoseStamped, "/target_object_pose", 10)
		self._pub_attach = node.create_publisher(Bool, "/object_attach_signal", 10)
		
		# TF buffer for frame transformations
		self._tf_buffer = Buffer(cache_time=ROSDuration(seconds=5))
		self._tf_listener = TransformListener(self._tf_buffer, node)

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

	def get_tracking_error(self, timeout_sec: float = 1.0) -> Optional[float]:
		"""
		Calculate the Euclidean distance between magnetic_link origin and tracking_task origin.
		
		Args:
			timeout_sec: Timeout for TF lookup in seconds.
			
		Returns:
			Float number representing Euclidean distance (m) between the two frame origins.
			Returns None if TF lookup fails.
		"""
		try:
			# Lookup transform from magnetic_link to tracking_task
			transform = self._tf_buffer.lookup_transform(
				"magnetic_link", "tracking_task", rclpy.time.Time(), timeout=ROSDuration(seconds=timeout_sec)
			)
			
			# Extract translation components
			x = transform.transform.translation.x
			y = transform.transform.translation.y
			z = transform.transform.translation.z
			
			# Calculate Euclidean distance
			distance = float(np.sqrt(x**2 + y**2 + z**2))
			self._node.get_logger().info(
				f"Tracking error (distance): {distance:.6f} m "
				f"(dx={x:.6f}, dy={y:.6f}, dz={z:.6f})"
			)
			return distance
			
		except TransformException as exc:
			self._node.get_logger().error(
				f"Cannot lookup transform magnetic_link -> tracking_task: {exc}"
			)
			return None

class MotoMiniMainLoop(Node):
	def __init__(self):
		super().__init__("motomini_main_loop")

		self.pub_targets = self.create_publisher(PoseArray, "/target_poses", 10)
		self.pub_start = self.create_publisher(Bool, "/start", 10)
		self.pub_clear = self.create_publisher(Bool, "/clear_targets", 10)
		self.pub_tracking_control = self.create_publisher(Bool, "/tracking_control", 10)
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

	def _publish_start(self, timeout_sec: float = 10.0) -> bool:
		"""Spam /start until the planning node confirms 'Planning Started' on /optimization_status.

		Returns:
			True if planning started successfully, False if it failed or timed out.
		"""
		# Clear stale status so we don't match a value from a previous operation.
		with self._status_lock:
			self._last_status = ""

		deadline = time.time() + timeout_sec
		while rclpy.ok() and time.time() < deadline:
			msg = Bool()
			msg.data = True
			self.pub_start.publish(msg)
			self.get_logger().info("Published /start — waiting for 'Planning Started'...")

			# Wait up to 500 ms for a response before re-publishing.
			check_until = min(time.time() + 0.5, deadline)
			while rclpy.ok() and time.time() < check_until:
				with self._status_lock:
					s = self._last_status
				if "Planning Started" in s:
					self.get_logger().info(f"Start confirmed: {s}")
					return True
				if s.startswith("Failed:"):
					self.get_logger().error(f"Planning node rejected start: {s}")
					return False
				time.sleep(0.05)

		self.get_logger().error(f"Timed out ({timeout_sec}s) waiting for 'Planning Started'")
		return False

	def _publish_clear(self, timeout_sec: float = 5.0) -> bool:
		"""Spam /clear_targets until the planning node confirms 'Buffer Cleared' on /optimization_status.

		Returns:
			True if the buffer was cleared, False if timed out.
		"""
		# Clear stale status so we don't match a value from a previous operation.
		with self._status_lock:
			self._last_status = ""

		deadline = time.time() + timeout_sec
		while rclpy.ok() and time.time() < deadline:
			msg = Bool()
			msg.data = True
			self.pub_clear.publish(msg)
			self.get_logger().info("Published /clear_targets — waiting for 'Buffer Cleared'...")

			# Wait up to 500 ms for a response before re-publishing.
			check_until = min(time.time() + 0.5, deadline)
			while rclpy.ok() and time.time() < check_until:
				with self._status_lock:
					s = self._last_status
				if "Buffer Cleared" in s:
					self.get_logger().info(f"Clear confirmed: {s}")
					return True
				time.sleep(0.05)

		self.get_logger().error(f"Timed out ({timeout_sec}s) waiting for 'Buffer Cleared'")
		return False

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

		if not self._publish_start(timeout_sec=10.0):
			raise RuntimeError("Planning node did not confirm 'Planning Started' in time")

		if not self.wait_for_exec_true_then_false(timeout_sec=120.0):
			raise RuntimeError("Did not observe /trajectory_executing transition true -> false in time")

		if not self._publish_clear(timeout_sec=5.0):
			self.get_logger().warning("Clear not confirmed — proceeding anyway")
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

	def disable_tracking(self) -> None:
		"""Disable object tracking mode."""
		msg = Bool()
		msg.data = False
		self.pub_tracking_control.publish(msg)
		self.get_logger().info("Tracking mode DISABLED")

	def enable_tracking(self, timeout_sec: float = 5.0) -> bool:
		"""Enable object tracking mode and wait until the controller enters POSE_FOLLOW.

		Returns:
			True if the controller reached POSE_FOLLOW within timeout_sec, False otherwise.
		"""
		msg = Bool()
		msg.data = True
		self.pub_tracking_control.publish(msg)
		self.get_logger().info("Tracking mode ENABLED — waiting for STATE_POSE_FOLLOW...")

		# Wait for transitionTrackingState to publish "STATE_POSE_FOLLOW" on /optimization_status.
		if self.wait_for_status_contains("STATE_POSE_FOLLOW", timeout_sec=timeout_sec):
			self.get_logger().info("Tracking controller is now in STATE_POSE_FOLLOW — ready.")
			return True

		self.get_logger().error(
			f"Tracking controller did NOT reach STATE_POSE_FOLLOW within {timeout_sec} s."
		)
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
        gantry_control.move_to(x_pos=-0.0, z_pos=-0.00, motion_time_sec=1.5)
        gantry_control.move_to(x_pos=-0.04, z_pos=-0.035, motion_time_sec=1.5)
      
      
      
      
        node._publish_clear()

        node.set_planner_mode(2)
        object_control.set_current_target(-0.04, -0.25, 0.07)
        targets = node.make_targets([
            (-0.03, -0.24, 0.2),
            (-0.03, -0.24, 0.071)
        ])
        node.execute_target_pose(targets)
        object_control.attach()

        # time.sleep(1.0) 
        node._publish_clear()
        # # Moving to hover ready to Grinding
        node.set_planner_mode(2)
        # time.sleep(1.0) 
        # node._publish_clear()
        targets = node.make_targets([
            (-0.03, -0.24, 0.2),
            (0.0530, -0.2709, 0.175)
        ])
        node.execute_target_pose(targets)
        node.set_planner_mode(3)
        time.sleep(0.5)
        node.set_planner_mode(3)
        circle_targets = ObjectProcess.generate_circle_poses(
            x=0.0530, y=-0.275, z=0.175,
            number_of_points=10,
            radius=0.04,
            alpha_degrees=-30.0,
        )
        node.execute_target_pose(circle_targets)
        
        # moving hove ready for tracking
        targets = node.make_targets([
            (0.18, 0.00, 0.245)
        ])
        node.execute_target_pose(targets)




        targets = node.make_targets([
            (0.13, -0.19, 0.1)
        ])
        node.execute_target_pose(targets)
        controller_ready = node.enable_tracking(timeout_sec=5.0)
        if not controller_ready:
            print("Warning: tracking controller did not reach POSE_FOLLOW — proceeding anyway.")
        
        # Signal the tracking publisher to start sending commands to the robot.
        time.sleep(1.0)  # Short delay to ensure controller is ready to receive tracking commands
        sucess = ObjectProcess.wait_for_tracking_status_change(node, timeout_sec=15.0)
        if sucess:
            ObjectProcess.start_tracking_publish(node)
            # Poll get_tracking_error() until EE is close enough to the belt object.
            # Error must stay below threshold for a sustained duration before settling.
            error_threshold_m = 0.009   # metres — tune as needed
            settle_duration_s = 1.0     # must stay below threshold for this long
            tracking_timeout_sec = 10.0
            start_t = time.time()
            tracking_settled = False
            settle_start_t = None
            
            while rclpy.ok() and (time.time() - start_t) < tracking_timeout_sec:
                # dist = object_control.get_tracking_error(timeout_sec=0.5)
                
                # if dist is not None:
                #     # Always log the current error
                #     status = "OK" if dist < error_threshold_m else "HIGH"
                #     elapsed = time.time() - start_t
                #     print(f"[{elapsed:.1f}s] Tracking error: {dist:.6f} m "
                #           f"(threshold: {error_threshold_m} m) [{status}]")
                    
                #     if dist < error_threshold_m:
                #         # Error is below threshold; start/continue settlement timer
                #         if settle_start_t is None:
                #             settle_start_t = time.time()
                #             print(f"  → Error below threshold, settlement timer started")
                        
                #         # Check if error stayed good long enough
                #         settle_elapsed = time.time() - settle_start_t
                #         if settle_elapsed >= settle_duration_s:
                #             print(f"Tracking settled: error stayed < {error_threshold_m} m for {settle_duration_s}s")
                #             tracking_settled = True
                #             break
                #     else:
                #         # Error exceeded threshold; reset settlement timer
                #         if settle_start_t is not None:
                #             print(f"  → Error exceeded threshold, settlement timer reset")
                #             settle_start_t = None
                time.sleep(3.0)
                object_control.detach()
                object_control.detach()
                object_control.detach()
                tracking_settled = True
                
                time.sleep(0.05)
            
            if tracking_settled:
                object_control.detach()
                ObjectProcess.stop_tracking_publish(node)
                node.disable_tracking()
            else:
                print(f"Tracking did not settle within {tracking_timeout_sec} s.")
                node.set_planner_mode(2)
                node.disable_tracking()
                object_control.set_current_target(-0.05, -0.25, 0.07)
                targets = node.make_targets([
                    (-0.03, -0.24, 0.2),
                    (-0.03, -0.24, 0.071)
                ])
                node.execute_target_pose(targets)
                object_control.detach()
        else:
            print("Failed to detect tracking status transition in time.")
        
        node._publish_clear()
        node.set_planner_mode(3)
        # time.sleep(0.5) 
        targets = node.make_targets([
            (0.18, 0.00, 0.245)
        ])
        node.execute_target_pose(targets)

    except Exception as exc:
        node.get_logger().error(f"Main loop failed: {exc}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()