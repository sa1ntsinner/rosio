import message_filters
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from message_filters import ApproximateTimeSynchronizer
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class WrappersNode(Node):
    def __init__(self):
        super().__init__("px_wrappers")

        self._updater = Updater(self)
        self._updater.setHardwareID("px-fixture")
        self._updater.add("cloud pair", self._report)

        left = message_filters.Subscriber(self, PointCloud2, "/px/left/points")
        right = message_filters.Subscriber(self, PointCloud2, "/px/right/points")
        self._sync = ApproximateTimeSynchronizer([left, right], 10, 0.05)
        self._sync.registerCallback(self._on_pair)

    def _report(self, status):
        status.summary(DiagnosticStatus.OK, "paired")
        return status

    def _on_pair(self, left, right):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WrappersNode()
    rclpy.spin(node)
    rclpy.shutdown()
