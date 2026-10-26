"""Stops the rover when the filtered scan sees something closer than the stop distance."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")
        self.stop_distance = self.declare_parameter("stop_distance", 0.5).value
        self.max_speed = self.declare_parameter("max_speed", 0.8).value
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, reliable)
        self.command = self.create_publisher(Twist, "cmd_vel", 10)

    def on_scan(self, scan):
        nearest = min((r for r in scan.ranges if math.isfinite(r)), default=math.inf)
        twist = Twist()
        twist.linear.x = 0.0 if nearest < self.stop_distance else self.max_speed
        self.command.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(SafetyStop())
    rclpy.shutdown()
