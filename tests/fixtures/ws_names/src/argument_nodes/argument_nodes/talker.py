from rclpy.lifecycle import LifecycleNode


class Talker(LifecycleNode):
    def __init__(self, node_name):
        super().__init__(node_name)


def main():
    Talker("argument_talker")
