"""Write a small clean rosbag2 that the synthetic corpus entries mutate.

Three topics, all clean and all sharing one clock: /scan at 10 Hz, /imu at 50 Hz and
/gps at 5 Hz. inter_topic_skew only compares a bag with at least three topics, so the
skew entry needs all of them, one mutated topic against two clean ones.
"""

import argparse
import sys
from pathlib import Path

import numpy
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
SCAN_TYPE = "sensor_msgs/msg/LaserScan"
IMU_TYPE = "sensor_msgs/msg/Imu"
GPS_TYPE = "sensor_msgs/msg/NavSatFix"
BASE = 1750000000.0
COUNT = 120
PERIOD = 0.1
LATENCY = 0.02


def header(seconds, frame_id):
    stamp = TYPESTORE.types["builtin_interfaces/msg/Time"](
        sec=int(seconds), nanosec=round((seconds - int(seconds)) * 1e9)
    )
    return TYPESTORE.types["std_msgs/msg/Header"](stamp=stamp, frame_id=frame_id)


def scan(seconds):
    return TYPESTORE.types[SCAN_TYPE](
        header=header(seconds, "lidar"),
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=1.0,
        time_increment=0.0,
        scan_time=PERIOD,
        range_min=0.1,
        range_max=10.0,
        ranges=numpy.array([1.0, 2.0, 3.0], dtype=numpy.float32),
        intensities=numpy.array([], dtype=numpy.float32),
    )


def imu(seconds):
    zeros = numpy.zeros(9, dtype=numpy.float64)
    vector = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    return TYPESTORE.types[IMU_TYPE](
        header=header(seconds, "imu_link"),
        orientation=TYPESTORE.types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=zeros.copy(),
        angular_velocity=vector(x=0.0, y=0.0, z=0.0),
        angular_velocity_covariance=zeros.copy(),
        linear_acceleration=vector(x=0.0, y=0.0, z=9.81),
        linear_acceleration_covariance=zeros.copy(),
    )


def gps(seconds):
    return TYPESTORE.types[GPS_TYPE](
        header=header(seconds, "gnss"),
        status=TYPESTORE.types["sensor_msgs/msg/NavSatStatus"](status=0, service=1),
        latitude=51.5,
        longitude=7.4,
        altitude=90.0,
        position_covariance=numpy.zeros(9, dtype=numpy.float64),
        position_covariance_type=0,
    )


TOPICS = (
    ("/scan", SCAN_TYPE, 1.0 / PERIOD, scan),
    ("/imu", IMU_TYPE, 50.0, imu),
    ("/gps", GPS_TYPE, 5.0, gps),
)


def write(dest, count=COUNT):
    duration = count * PERIOD
    with Writer(dest, version=8) as writer:
        records = []
        for topic, msgtype, rate, build in TOPICS:
            connection = writer.add_connection(topic, msgtype, typestore=TYPESTORE)
            for i in range(round(duration * rate)):
                stamp = BASE + i / rate
                records.append((stamp + LATENCY, connection, msgtype, stamp, build))
        for receive, connection, msgtype, stamp, build in sorted(records, key=lambda r: r[0]):
            writer.write(
                connection, int(receive * 1e9), TYPESTORE.serialize_cdr(build(stamp), msgtype)
            )
    return dest


def main(argv=None):
    parser = argparse.ArgumentParser(description="write a clean synthetic rosbag2")
    parser.add_argument("dest")
    parser.add_argument("--count", type=int, default=COUNT, help="messages on /scan")
    args = parser.parse_args(argv)
    dest = Path(args.dest)
    if dest.exists():
        print(f"synthesize: {dest} already exists", file=sys.stderr)
        return 2
    write(dest, args.count)
    duration = args.count * PERIOD
    for topic, _, rate, _ in TOPICS:
        print(f"{round(duration * rate)} messages on {topic} at {rate:.0f} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
