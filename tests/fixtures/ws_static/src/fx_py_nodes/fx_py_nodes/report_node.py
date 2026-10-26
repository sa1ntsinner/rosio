import rclpy
from rclpy.node import Node


class Report(Node):
    def __init__(self):
        super().__init__("fx_report")


def main(args=None):
    rclpy.init(args=args)
    node = Report()
    rclpy.spin(node)
    rclpy.shutdown()
