#!/usr/bin/env python3

import argparse
import importlib
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

try:
	cp = importlib.import_module("cupy")
except ImportError:
	cp = None


class ArucoDepthNode(Node):
	def __init__(
		self,
		image_topic: str,
		cloud_topic: str,
		target_id: int,
		marker_size_m: float,
		boundary_inset_m: float,
		fx: float,
		fy: float,
		cx: float,
		cy: float,
		sync_tolerance_s: float,
		point_stride: int,
		min_points: int,
		cluster_radius_m: float,
		center_radius_ratio: float,
		use_gpu: bool,
	):
		super().__init__("aruco_depth_node")

		qos = QoSProfile(depth=10)
		qos.reliability = ReliabilityPolicy.RELIABLE
		qos.durability = DurabilityPolicy.VOLATILE

		self.image_sub = self.create_subscription(Image, image_topic, self.image_cb, qos)
		self.cloud_sub = self.create_subscription(PointCloud2, cloud_topic, self.cloud_cb, qos)
		self.used_cloud_pub = self.create_publisher(PointCloud2, "/aruco_dep/points", qos)
		self.tf_broadcaster = TransformBroadcaster(self)

		self.target_id = target_id
		self.marker_size_m = marker_size_m
		self.boundary_inset_m = max(0.0, boundary_inset_m)
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.sync_tolerance_ns = int(sync_tolerance_s * 1e9)
		self.point_stride = max(1, point_stride)
		self.min_points = max(1, min_points)
		self.cluster_radius_m = max(1e-4, cluster_radius_m)
		self.center_radius_ratio = float(np.clip(center_radius_ratio, 0.05, 0.8))
		self.use_gpu = bool(use_gpu and cp is not None)

		self.latest_cloud: Optional[PointCloud2] = None
		self._last_warn_ns = 0

		self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
		self.detector_params = cv2.aruco.DetectorParameters()
		self.detector = None
		if hasattr(cv2.aruco, "ArucoDetector"):
			self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

		self.get_logger().info(f"Image topic: {image_topic}")
		self.get_logger().info(f"Cloud topic: {cloud_topic}")
		self.get_logger().info(f"Target ArUco ID: {target_id}")
		self.get_logger().info("Publishing TF child frame: aruco_dep")
		self.get_logger().info("Publishing selected point cloud: /aruco_dep/points")
		self.get_logger().info(f"GPU acceleration: {'enabled' if self.use_gpu else 'disabled'}")

	@staticmethod
	def _stamp_to_ns(stamp) -> int:
		return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

	def _warn_throttle(self, msg: str, period_s: float = 2.0) -> None:
		now_ns = self._stamp_to_ns(self.get_clock().now().to_msg())
		if now_ns - self._last_warn_ns > int(period_s * 1e9):
			self.get_logger().warn(msg)
			self._last_warn_ns = now_ns

	@staticmethod
	def _ros_image_to_bgr(msg: Image) -> Optional[np.ndarray]:
		if msg.encoding not in ("bgr8", "rgb8"):
			return None

		raw = np.frombuffer(msg.data, dtype=np.uint8)
		expected = msg.height * msg.step
		if raw.size < expected:
			return None

		image = raw[:expected].reshape((msg.height, msg.step))
		image = image[:, : msg.width * 3].reshape((msg.height, msg.width, 3))
		if msg.encoding == "rgb8":
			image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		return image

	def cloud_cb(self, msg: PointCloud2) -> None:
		self.latest_cloud = msg

	def _detect_target_center(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, int]]:
		if self.detector is not None:
			corners, ids, _ = self.detector.detectMarkers(image)
		else:
			corners, ids, _ = cv2.aruco.detectMarkers(
				image,
				self.aruco_dict,
				parameters=self.detector_params,
			)

		if ids is None or len(ids) == 0:
			return None

		target_idxs = np.where(ids.flatten() == self.target_id)[0]
		if target_idxs.size == 0:
			return None

		pts = corners[int(target_idxs[0])].reshape(-1, 2)
		center = np.mean(pts, axis=0)
		edges = np.roll(pts, -1, axis=0) - pts
		avg_edge_px = float(np.mean(np.linalg.norm(edges, axis=1)))
		if self.marker_size_m > 0.0:
			inner_ratio = 1.0 - (2.0 * self.boundary_inset_m / self.marker_size_m)
			inner_ratio = float(np.clip(inner_ratio, 0.1, 1.0))
		else:
			inner_ratio = 1.0
		radius_px = max(2, int(0.5 * avg_edge_px * inner_ratio * self.center_radius_ratio))
		return center.astype(np.float32), radius_px

	@staticmethod
	def _cloud_xyz_array(cloud: PointCloud2) -> np.ndarray:
		if cloud.point_step < 12:
			return np.empty((0, 3), dtype=np.float32)

		point_count = cloud.width * cloud.height
		cloud_array = np.frombuffer(cloud.data, dtype=np.float32)
		cloud_array = cloud_array.reshape(point_count, cloud.point_step // 4)
		return cloud_array[:, :3].copy()

	def _select_points_in_circle(
		self,
		cloud: PointCloud2,
		center_uv: np.ndarray,
		radius_px: int,
		image_shape: Tuple[int, int],
	) -> np.ndarray:
		xyz = self._cloud_xyz_array(cloud)
		if xyz.size == 0:
			return np.empty((0, 3), dtype=np.float32)

		xyz = xyz[:: self.point_stride]
		height, width = image_shape
		radius2 = float(radius_px * radius_px)
		center_u = float(center_uv[0])
		center_v = float(center_uv[1])

		if self.use_gpu:
			xyz_gpu = cp.asarray(xyz)
			z_gpu = xyz_gpu[:, 2]
			valid_gpu = cp.isfinite(xyz_gpu).all(axis=1) & (z_gpu > 0.0)
			xyz_gpu = xyz_gpu[valid_gpu]
			if xyz_gpu.shape[0] == 0:
				return np.empty((0, 3), dtype=np.float32)

			u_gpu = cp.rint(self.fx * xyz_gpu[:, 0] / xyz_gpu[:, 2] + self.cx).astype(cp.int32)
			v_gpu = cp.rint(self.fy * xyz_gpu[:, 1] / xyz_gpu[:, 2] + self.cy).astype(cp.int32)
			inside_gpu = (u_gpu >= 0) & (u_gpu < width) & (v_gpu >= 0) & (v_gpu < height)
			u_gpu = u_gpu[inside_gpu]
			v_gpu = v_gpu[inside_gpu]
			xyz_gpu = xyz_gpu[inside_gpu]
			if xyz_gpu.shape[0] == 0:
				return np.empty((0, 3), dtype=np.float32)

			du_gpu = u_gpu.astype(cp.float32) - center_u
			dv_gpu = v_gpu.astype(cp.float32) - center_v
			inside_circle_gpu = (du_gpu * du_gpu + dv_gpu * dv_gpu) <= radius2
			selected_gpu = xyz_gpu[inside_circle_gpu]
			return cp.asnumpy(selected_gpu)

		valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.0)
		xyz = xyz[valid]
		if xyz.shape[0] == 0:
			return np.empty((0, 3), dtype=np.float32)

		u = np.rint(self.fx * xyz[:, 0] / xyz[:, 2] + self.cx).astype(np.int32)
		v = np.rint(self.fy * xyz[:, 1] / xyz[:, 2] + self.cy).astype(np.int32)
		inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
		u = u[inside]
		v = v[inside]
		xyz = xyz[inside]
		if xyz.shape[0] == 0:
			return np.empty((0, 3), dtype=np.float32)

		du = u.astype(np.float32) - center_u
		dv = v.astype(np.float32) - center_v
		inside_circle = (du * du + dv * dv) <= radius2
		return xyz[inside_circle]

	def _compute_cluster_center(
		self,
		cloud: PointCloud2,
		center_uv: np.ndarray,
		radius_px: int,
		image_shape: Tuple[int, int],
	):
		selected_points = self._select_points_in_circle(cloud, center_uv-0.06431385974052871-0.06431385974052871, radius_px, image_shape)
		if selected_points.shape[0] < self.min_points:
			return None

		median = np.median(selected_points, axis=0)
		dists = np.linalg.norm(selected_points - median, axis=1)
		cluster = selected_points[dists <= self.cluster_radius_m]
		if cluster.shape[0] < self.min_points:
			sorted_idx = np.argsort(dists)
			cluster = selected_points[sorted_idx[: self.min_points]]
		if cluster.shape[0] < self.min_points:
			return None

		center = np.mean(cluster, axis=0)
		return (float(center[0]), float(center[1]), float(center[2])), cluster.tolist()

	def _publish_used_cloud(self, parent_frame: str, stamp, selected_points) -> None:
		header = Header()
		header.stamp = stamp
		header.frame_id = parent_frame

		red_rgb = np.array([0x00FF0000], dtype=np.uint32).view(np.float32)[0]
		fields = [
			PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
			PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
			PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
			PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
		]
		points = ((x, y, z, float(red_rgb)) for x, y, z in selected_points)
		self.used_cloud_pub.publish(point_cloud2.create_cloud(header, fields, points))

	def _publish_tf(self, parent_frame: str, stamp, xyz) -> None:
		tf_msg = TransformStamped()
		tf_msg.header.stamp = stamp
		tf_msg.header.frame_id = parent_frame
		tf_msg.child_frame_id = "aruco_dep"
		tf_msg.transform.translation.x = xyz[0]
		tf_msg.transform.translation.y = xyz[1]
		tf_msg.transform.translation.z = xyz[2]
		tf_msg.transform.rotation.x = 0.0
		tf_msg.transform.rotation.y = 0.0
		tf_msg.transform.rotation.z = 0.0
		tf_msg.transform.rotation.w = 1.0
		self.tf_broadcaster.sendTransform(tf_msg)

	def image_cb(self, msg: Image) -> None:
		if self.latest_cloud is None:
			self._warn_throttle("No point cloud received yet on /cam/points")
			return

		image = self._ros_image_to_bgr(msg)
		if image is None:
			return

		t_img = self._stamp_to_ns(msg.header.stamp)
		t_cloud = self._stamp_to_ns(self.latest_cloud.header.stamp)
		if abs(t_img - t_cloud) > self.sync_tolerance_ns:
			dt = abs(t_img - t_cloud) / 1e9
			self._warn_throttle(f"Image-cloud timestamp mismatch: {dt:.3f}s")
			return

		center_result = self._detect_target_center(image)
		if center_result is None:
			self._warn_throttle("ArUco target id not detected in RGB frame")
			return

		center_uv, radius_px = center_result
		result = self._compute_cluster_center(self.latest_cloud, center_uv, radius_px, image.shape[:2])
		if result is None:
			self._warn_throttle("Not enough stable 3D points near ArUco center to publish aruco_dep")
			return

		xyz, selected_points = result
		self._publish_used_cloud(self.latest_cloud.header.frame_id, msg.header.stamp, selected_points)
		self._publish_tf(self.latest_cloud.header.frame_id, msg.header.stamp, xyz)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Aruco bbox + point cloud center/high TF publisher")
	parser.add_argument("--image-topic", default="/cam/rgb/image_raw", help="Input RGB topic")
	parser.add_argument("--cloud-topic", default="/cam/points", help="Input point cloud topic")
	parser.add_argument("--target-id", type=int, default=0, help="Target ArUco marker ID")
	parser.add_argument("--marker-size", type=float, default=0.05, help="Marker size in meters")
	parser.add_argument("--boundary-inset", type=float, default=0.005, help="Inset inside marker boundary in meters")

	parser.add_argument("--fx", type=float, default=615.0, help="Camera fx")
	parser.add_argument("--fy", type=float, default=615.0, help="Camera fy")
	parser.add_argument("--cx", type=float, default=320.0, help="Camera cx")
	parser.add_argument("--cy", type=float, default=240.0, help="Camera cy")

	parser.add_argument("--sync-tolerance", type=float, default=0.12, help="Image-cloud max time diff (s)")
	parser.add_argument("--point-stride", type=int, default=4, help="Use every Nth point for speed")
	parser.add_argument("--min-points", type=int, default=5, help="Minimum points in dense cluster to publish TF")
	parser.add_argument("--cluster-radius", type=float, default=0.015, help="Dense-point clustering radius in meters")
	parser.add_argument("--center-radius-ratio", type=float, default=0.25, help="Circle radius as fraction of marker size in image")
	parser.add_argument("--use-gpu", action="store_true", help="Use CuPy GPU acceleration when available")
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	rclpy.init()

	node = ArucoDepthNode(
		image_topic=args.image_topic,
		cloud_topic=args.cloud_topic,
		target_id=args.target_id,
		marker_size_m=args.marker_size,
		boundary_inset_m=args.boundary_inset,
		fx=args.fx,
		fy=args.fy,
		cx=args.cx,
		cy=args.cy,
		sync_tolerance_s=max(args.sync_tolerance, 0.5),
		point_stride=args.point_stride,
		min_points=args.min_points,
		cluster_radius_m=args.cluster_radius,
		center_radius_ratio=args.center_radius_ratio,
		use_gpu=args.use_gpu,
	)
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
