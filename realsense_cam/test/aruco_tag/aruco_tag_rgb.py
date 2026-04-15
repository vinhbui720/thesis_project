#!/usr/bin/env python3

import argparse
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(rot_mat: np.ndarray) -> np.ndarray:
	trace = np.trace(rot_mat)
	if trace > 0.0:
		s = np.sqrt(trace + 1.0) * 2.0
		qw = 0.25 * s
		qx = (rot_mat[2, 1] - rot_mat[1, 2]) / s
		qy = (rot_mat[0, 2] - rot_mat[2, 0]) / s
		qz = (rot_mat[1, 0] - rot_mat[0, 1]) / s
	elif rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
		s = np.sqrt(1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2]) * 2.0
		qw = (rot_mat[2, 1] - rot_mat[1, 2]) / s
		qx = 0.25 * s
		qy = (rot_mat[0, 1] + rot_mat[1, 0]) / s
		qz = (rot_mat[0, 2] + rot_mat[2, 0]) / s
	elif rot_mat[1, 1] > rot_mat[2, 2]:
		s = np.sqrt(1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2]) * 2.0
		qw = (rot_mat[0, 2] - rot_mat[2, 0]) / s
		qx = (rot_mat[0, 1] + rot_mat[1, 0]) / s
		qy = 0.25 * s
		qz = (rot_mat[1, 2] + rot_mat[2, 1]) / s
	else:
		s = np.sqrt(1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1]) * 2.0
		qw = (rot_mat[1, 0] - rot_mat[0, 1]) / s
		qx = (rot_mat[0, 2] + rot_mat[2, 0]) / s
		qy = (rot_mat[1, 2] + rot_mat[2, 1]) / s
		qz = 0.25 * s

	quat = np.array([qx, qy, qz, qw], dtype=np.float64)
	norm = np.linalg.norm(quat)
	if norm > 0.0:
		quat /= norm
	return quat


class ArucoTagRgbNode(Node):
	def __init__(
		self,
		image_topic: str,
		output_topic: str,
		debug: bool,
		marker_size_m: float,
		target_id: int,
		camera_matrix: np.ndarray,
		dist_coeffs: np.ndarray,
	):
		super().__init__("aruco_tag_rgb")

		qos = QoSProfile(depth=10)
		qos.reliability = ReliabilityPolicy.RELIABLE
		qos.durability = DurabilityPolicy.VOLATILE

		self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos)
		self.image_pub = self.create_publisher(Image, output_topic, qos) if debug else None
		self.tf_broadcaster = TransformBroadcaster(self)
		self.debug = debug

		self.marker_size_m = marker_size_m
		self.target_id = target_id
		self.camera_matrix = camera_matrix
		self.dist_coeffs = dist_coeffs

		self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
		self.detector_params = cv2.aruco.DetectorParameters()
		self.detector = None
		if hasattr(cv2.aruco, "ArucoDetector"):
			self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

		self.get_logger().info(f"Subscribed to: {image_topic}")
		if self.debug:
			self.get_logger().info(f"Annotated image topic: {output_topic}")
		self.get_logger().info(f"Detecting ArUco DICT_5X5_250 ID={target_id}")

	@staticmethod
	def _ros_image_to_bgr(msg: Image) -> Optional[np.ndarray]:
		if msg.encoding not in ("bgr8", "rgb8"):
			return None

		raw = np.frombuffer(msg.data, dtype=np.uint8)
		expected = msg.height * msg.step
		if raw.size < expected:
			return None

		image = raw[:expected].reshape((msg.height, msg.step))
		channels = 3
		width_bytes = msg.width * channels
		image = image[:, :width_bytes].reshape((msg.height, msg.width, channels))

		if msg.encoding == "rgb8":
			image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		return image.copy()

	@staticmethod
	def _bgr_to_ros_image(image: np.ndarray, src_header) -> Image:
		img = np.ascontiguousarray(image)
		out = Image()
		out.header = src_header
		out.height = img.shape[0]
		out.width = img.shape[1]
		out.encoding = "bgr8"
		out.is_bigendian = 0
		out.step = int(img.strides[0])
		out.data = img.tobytes()
		return out

	def _publish_marker_tf(self, header, rvec: np.ndarray, tvec: np.ndarray) -> None:
		rot_mat, _ = cv2.Rodrigues(rvec)
		quat = rotation_matrix_to_quaternion(rot_mat)

		tf_msg = TransformStamped()
		tf_msg.header = header
		tf_msg.child_frame_id = "aruco_link"
		tf_msg.transform.translation.x = float(tvec[0])
		tf_msg.transform.translation.y = float(tvec[1])
		tf_msg.transform.translation.z = float(tvec[2])
		tf_msg.transform.rotation.x = float(quat[0])
		tf_msg.transform.rotation.y = float(quat[1])
		tf_msg.transform.rotation.z = float(quat[2])
		tf_msg.transform.rotation.w = float(quat[3])
		self.tf_broadcaster.sendTransform(tf_msg)

	def _estimate_pose(self, marker_corners: np.ndarray):
		if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
			rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
				[marker_corners],
				self.marker_size_m,
				self.camera_matrix,
				self.dist_coeffs,
			)
			return rvecs[0][0], tvecs[0][0]

		half = self.marker_size_m * 0.5
		obj_pts = np.array(
			[
				[-half, half, 0.0],
				[half, half, 0.0],
				[half, -half, 0.0],
				[-half, -half, 0.0],
			],
			dtype=np.float32,
		)
		img_pts = marker_corners.reshape(4, 2).astype(np.float32)
		ok, rvec, tvec = cv2.solvePnP(
			obj_pts,
			img_pts,
			self.camera_matrix,
			self.dist_coeffs,
			flags=cv2.SOLVEPNP_IPPE_SQUARE,
		)
		if not ok:
			return None, None
		return rvec.reshape(3), tvec.reshape(3)

	def image_callback(self, msg: Image) -> None:
		image = self._ros_image_to_bgr(msg)
		if image is None:
			return

		debug_img = image.copy() if self.debug else None

		if self.detector is not None:
			corners, ids, _ = self.detector.detectMarkers(image)
		else:
			corners, ids, _ = cv2.aruco.detectMarkers(
				image,
				self.aruco_dict,
				parameters=self.detector_params,
			)

		if ids is not None and len(ids) > 0:
			if self.debug and debug_img is not None:
				cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

			target_idxs = np.where(ids.flatten() == self.target_id)[0]
			if target_idxs.size > 0:
				idx = int(target_idxs[0])
				rvec, tvec = self._estimate_pose(corners[idx])
				if rvec is None or tvec is None:
					if self.debug and self.image_pub is not None and debug_img is not None:
						self.image_pub.publish(self._bgr_to_ros_image(debug_img, msg.header))
					return

				if self.debug and debug_img is not None:
					cv2.drawFrameAxes(
						debug_img,
						self.camera_matrix,
						self.dist_coeffs,
						rvec,
						tvec,
						self.marker_size_m * 0.5,
					)
				self._publish_marker_tf(msg.header, rvec, tvec)

		if self.debug and self.image_pub is not None and debug_img is not None:
			self.image_pub.publish(self._bgr_to_ros_image(debug_img, msg.header))


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Detect ArUco 5x5 ID 0 from RGB topic")
	parser.add_argument("--image-topic", default="/cam/rgb/image_raw", help="Input RGB topic")
	parser.add_argument("--output-topic", default="/cam/rgb/aruco_annotated", help="Output annotated image topic")
	parser.add_argument("--debug", action="store_true", help="Publish annotated debug image")
	parser.add_argument("--target-id", type=int, default=0, help="Target ArUco marker ID")
	parser.add_argument("--marker-size", type=float, default=0.05, help="Marker size in meters")

	parser.add_argument("--fx", type=float, default=615.0, help="Camera fx")
	parser.add_argument("--fy", type=float, default=615.0, help="Camera fy")
	parser.add_argument("--cx", type=float, default=320.0, help="Camera cx")
	parser.add_argument("--cy", type=float, default=240.0, help="Camera cy")
	parser.add_argument("--k1", type=float, default=0.0, help="Distortion k1")
	parser.add_argument("--k2", type=float, default=0.0, help="Distortion k2")
	parser.add_argument("--p1", type=float, default=0.0, help="Distortion p1")
	parser.add_argument("--p2", type=float, default=0.0, help="Distortion p2")
	parser.add_argument("--k3", type=float, default=0.0, help="Distortion k3")

	return parser.parse_args()


def main() -> None:
	args = parse_args()

	camera_matrix = np.array(
		[
			[args.fx, 0.0, args.cx],
			[0.0, args.fy, args.cy],
			[0.0, 0.0, 1.0],
		],
		dtype=np.float64,
	)
	dist_coeffs = np.array([args.k1, args.k2, args.p1, args.p2, args.k3], dtype=np.float64)

	rclpy.init()
	node = ArucoTagRgbNode(
		image_topic=args.image_topic,
		output_topic=args.output_topic,
		debug=args.debug,
		marker_size_m=args.marker_size,
		target_id=args.target_id,
		camera_matrix=camera_matrix,
		dist_coeffs=dist_coeffs,
	)
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
