import rclpy
from rclpy.node import Node


class Watchdog(Node):
    def __init__(self):
        super().__init__("fx_watchdog")
        self.declare_parameter("timeout_s", 2.0)


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    rclpy.spin(node)
    rclpy.shutdown()
