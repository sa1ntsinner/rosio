import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class LiteralNode(Node):
    def __init__(self):
        super().__init__("px_literal")

        self.points_sub = self.create_subscription(
            PointCloud2, "/px/points", self._on_points, qos_profile_sensor_data
        )

        self.odom_pub = self.create_publisher(Odometry, "/px/odom", 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, "/px/ready", latched_qos)

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.scan_sub = self.create_subscription(LaserScan, "/px/scan", self._on_scan, state_qos)

        self.reset_srv = self.create_service(Trigger, "/px/reset", self._on_reset)
        self.enable_client = self.create_client(SetBool, "/px/enable_motion")
        self.timer = self.create_timer(0.1, self._publish)

    def _on_points(self, msg):
        pass

    def _on_scan(self, msg):
        pass

    def _on_reset(self, request, response):
        return response

    def _publish(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LiteralNode()
    rclpy.spin(node)
    rclpy.shutdown()
