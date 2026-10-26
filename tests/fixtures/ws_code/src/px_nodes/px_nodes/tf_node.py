import argparse
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster


class TfNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self._odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self._base_frame = str(self.declare_parameter("base_frame", "base_footprint").value)
        self._sensor_frame = str(self.declare_parameter("sensor_frame", "px_lidar").value)
        self._publish_tf = bool(self.declare_parameter("publish_tf", True).value)

        self._broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._static_broadcaster = StaticTransformBroadcaster(self)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        self.cloud_sub = self.create_subscription(PointCloud2, "/px/points", self._on_cloud, 10)
        self._send_mount()

    def _send_odom(self):
        tf = TransformStamped()
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        self._broadcaster.sendTransform(tf)

    def _send_mount(self):
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "px_lidar"
        self._static_broadcaster.sendTransform(tf)

    def _on_cloud(self, msg):
        self._buffer.lookup_transform("map", "base_footprint", Time())
        self._buffer.lookup_transform(self._base_frame, self._sensor_frame, Time())
        self._buffer.lookup_transform(self._base_frame, msg.header.frame_id, Time())
        self._buffer.can_transform("map", self._odom_frame, Time())


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-name", default="px_tf")
    known, _ = parser.parse_known_args(sys.argv[1:])
    rclpy.init(args=args)
    node = TfNode(known.node_name)
    rclpy.spin(node)
    rclpy.shutdown()
