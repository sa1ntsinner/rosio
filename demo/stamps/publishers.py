"""One Imu publisher, clean or faulty depending on the environment."""

import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

ROLES = {
    "clean": {"topic": "/imu/clean", "mode": "fixed", "offset_s": "0"},
    "skewed": {"topic": "/imu/skewed", "mode": "fixed", "offset_s": "37"},
}


def setting(name, default):
    value = os.environ.get(name)
    return default if value is None or value == "" else value


class StampPublisher(Node):
    def __init__(self):
        role = ROLES.get(setting("ROLE", "clean"), ROLES["clean"])
        self.topic = setting("TOPIC", role["topic"])
        self.mode = setting("MODE", role["mode"])
        self.offset_s = float(setting("OFFSET_S", role["offset_s"]))
        self.drift_s_per_s = float(setting("DRIFT_S_PER_S", "0.005"))
        self.rate_hz = float(setting("RATE_HZ", "50"))
        super().__init__("stamp_publisher")
        self.started = time.time()
        self.publisher = self.create_publisher(Imu, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish)
        self.get_logger().info(
            f"{self.topic} at {self.rate_hz:.0f} Hz, mode {self.mode}, "
            f"offset {self.offset_s:.3f} s, drift {self.drift_s_per_s:.6f} s/s"
        )

    def stamp(self):
        now = time.time()
        if self.mode == "zero":
            return 0.0
        if self.mode == "drift":
            return now + self.drift_s_per_s * (now - self.started)
        return now + self.offset_s

    def publish(self):
        message = Imu()
        stamp = self.stamp()
        seconds = int(stamp)
        nanoseconds = round((stamp - seconds) * 1e9)
        if nanoseconds >= 1000000000:
            seconds += 1
            nanoseconds -= 1000000000
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = nanoseconds
        message.header.frame_id = "imu_link"
        message.orientation.w = 1.0
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = StampPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
