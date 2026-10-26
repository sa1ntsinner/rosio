from rclpy.node import Node


class DefaultNode(Node):
    def __init__(self, node_name="default_name"):
        super().__init__(node_name)


def main():
    DefaultNode()
