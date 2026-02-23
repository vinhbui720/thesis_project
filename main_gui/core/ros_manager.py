import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from threading import Thread


class ROSManager:
    def __init__(self):
        rclpy.init()
        self.node = Node("thesis_gui_node")

        self.executor_thread = Thread(target=self.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()

        self.cloud_callback = None

    def spin(self):
        rclpy.spin(self.node)

    def subscribe_cloud(self, topic="/model_cloud"):
        self.node.create_subscription(
            PointCloud2,
            topic,
            self._internal_cloud_callback,
            10
        )

    def _internal_cloud_callback(self, msg):
        if self.cloud_callback:
            self.cloud_callback(msg)