import os

import rclpy
from geometry_msgs.msg import Polygon, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32

from px_nodes.tuning import load_tuning

STATUS_TOPIC = "/px/status"

DEFAULT_LIMITS = {"max_speed": 3.0, "max_accel": 1.0}


class ParamNode(Node):
    def __init__(self):
        super().__init__("px_param")
        self._tuning = load_tuning()

        input_topic = str(self.declare_parameter("input_topic", "/px/points_raw").value)
        self.cloud_sub = self.create_subscription(PointCloud2, input_topic, self._on_cloud, 10)

        self._cmd_topic = str(self.declare_parameter("cmd_topic", "/px/cmd_vel_safe").value)
        self.cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        prefix = str(self.declare_parameter("topic_prefix", "/px").value)
        self.health_pub = self.create_publisher(Float32, f"{prefix}/health", 10)

        self.status_pub = self.create_publisher(Bool, STATUS_TOPIC, 1)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("queue_depth", 5)
        self.declare_parameter("publish_diagnostics", True)
        self.declare_parameter("watchdog_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("gain", self._tuning.gain)

        self.declare_parameters(
            namespace="limits",
            parameters=[
                ("max_speed", DEFAULT_LIMITS["max_speed"]),
                ("max_accel", DEFAULT_LIMITS["max_accel"]),
            ],
        )

        sensor_ns = "sensors.front"
        self.declare_parameter(f"{sensor_ns}.frame_id", "px_front")
        self.declare_parameter("diagnostics.hardware_id", "px-fixture")

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._queue_depth = int(self.get_parameter("queue_depth").value)

        self.add_on_set_parameters_callback(self._validate)

        footprint_topics = list(
            self.declare_parameter(
                "footprint_topics",
                ["local_costmap/footprint", "global_costmap/footprint", "/px/footprint"],
            ).value
        )
        self._footprint_pubs = [
            self.create_publisher(Polygon, topic, 1) for topic in footprint_topics
        ]

        env_topic = os.environ.get("PX_EXTRA_TOPIC", "")
        if env_topic:
            self.extra_sub = self.create_subscription(PointCloud2, env_topic, self._on_cloud, 10)

        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _validate(self, params):
        return SetParametersResult(successful=True)

    def _on_cloud(self, msg):
        pass

    def _publish(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    rclpy.shutdown()
