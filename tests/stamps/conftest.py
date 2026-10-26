import sqlite3
import struct

import numpy
import pytest
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
ROS1_TYPESTORE = get_typestore(Stores.ROS1_NOETIC)
SCAN_TYPE = "sensor_msgs/msg/LaserScan"
STRING_TYPE = "std_msgs/msg/String"
ODD_TYPE = "test_msgs/msg/Odd"
ROTTEN_TYPE = "test_msgs/msg/Rotten"
BASE_S = 1750000000.0
STAMPS = [BASE_S + 10.0 + i * 0.1 for i in range(4)]
RECEIVE_TIMES = [stamp + 0.05 for stamp in STAMPS]
SHIFTED_STAMPS = [10000.0 + i * 0.1 for i in range(4)]
SEP = "=" * 80 + "\n"
# Damaged on purpose, no message definition grammar accepts that line. It stands for a
# recording whose embedded definition is corrupt, not for any one reader release.
ROTTEN_MSGDEF = (
    "std_msgs/Header header\n"
    "%%% not a message definition ###\n"
    + SEP
    + "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    + SEP
    + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)


def scan_message(seconds, typestore=TYPESTORE, seq=None):
    stamp = typestore.types["builtin_interfaces/msg/Time"](
        sec=int(seconds), nanosec=round((seconds - int(seconds)) * 1e9)
    )
    fields = {"stamp": stamp, "frame_id": "lidar"}
    if seq is not None:
        fields["seq"] = seq
    scan = typestore.types[SCAN_TYPE]
    return scan(
        header=typestore.types["std_msgs/msg/Header"](**fields),
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=1.0,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.1,
        range_max=10.0,
        ranges=numpy.array([1.0, 2.0, 3.0], dtype=numpy.float32),
        intensities=numpy.array([], dtype=numpy.float32),
    )


def write_bag(path, stamps, receive_times=None, version=8):
    receive_times = stamps if receive_times is None else receive_times
    with Writer(path, version=version) as writer:
        scan = writer.add_connection("/scan", SCAN_TYPE, typestore=TYPESTORE)
        text = writer.add_connection("/chatter", STRING_TYPE, typestore=TYPESTORE)
        for stamp, receive in zip(stamps, receive_times, strict=True):
            writer.write(
                scan, int(receive * 1e9), TYPESTORE.serialize_cdr(scan_message(stamp), SCAN_TYPE)
            )
        writer.write(
            text,
            int(receive_times[0] * 1e9),
            TYPESTORE.serialize_cdr(TYPESTORE.types[STRING_TYPE](data="hello"), STRING_TYPE),
        )
    return path


def write_ros1_bag(path, stamps, receive_times=None, odd=False):
    receive_times = stamps if receive_times is None else receive_times
    with Writer1(path) as writer:
        scan = writer.add_connection("/scan", SCAN_TYPE, typestore=ROS1_TYPESTORE)
        for seq, (stamp, receive) in enumerate(zip(stamps, receive_times, strict=True)):
            message = scan_message(stamp, typestore=ROS1_TYPESTORE, seq=seq)
            writer.write(
                scan, int(receive * 1e9), ROS1_TYPESTORE.serialize_ros1(message, SCAN_TYPE)
            )
        if odd:
            connection = writer.add_connection(
                "/odd", ODD_TYPE, msgdef="uint32 header\nuint32 value\n", md5sum="0" * 32
            )
            writer.write(connection, int(receive_times[0] * 1e9), struct.pack("<II", 1, 2))
    return path


@pytest.fixture
def clean_bag(tmp_path):
    return write_bag(tmp_path / "clean", STAMPS, RECEIVE_TIMES)


@pytest.fixture
def headless_bag(tmp_path):
    """Bag that opens fine but carries no topic with a header, so nothing can be analyzed."""
    path = tmp_path / "headless"
    with Writer(path, version=8) as writer:
        text = writer.add_connection("/chatter", STRING_TYPE, typestore=TYPESTORE)
        for i, receive in enumerate(RECEIVE_TIMES):
            message = TYPESTORE.types[STRING_TYPE](data=f"hello {i}")
            writer.write(text, int(receive * 1e9), TYPESTORE.serialize_cdr(message, STRING_TYPE))
    return path


@pytest.fixture
def backward_bag(tmp_path):
    return write_bag(tmp_path / "backward", [BASE_S + t for t in (10.0, 10.1, 9.6, 10.3)], STAMPS)


@pytest.fixture
def humble_bag(tmp_path):
    """Bag without embedded type definitions, as written by ROS 2 Humble."""
    path = write_bag(tmp_path / "humble", STAMPS, RECEIVE_TIMES, version=5)
    with sqlite3.connect(next(path.glob("*.db3"))) as db:
        db.execute("UPDATE schema SET schema_version = 3")
        db.execute("DROP TABLE message_definitions")
    return path


def write_rotten_mcap(path, stamps):
    """Bag with one topic whose embedded type definition is damaged beyond parsing."""
    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        scan = writer.add_connection("/scan", SCAN_TYPE, typestore=TYPESTORE)
        rotten = writer.add_connection(
            "/rotten", ROTTEN_TYPE, msgdef=ROTTEN_MSGDEF, rihs01="RIHS01_" + "0" * 64
        )
        for stamp in stamps:
            writer.write(
                scan, int(stamp * 1e9), TYPESTORE.serialize_cdr(scan_message(stamp), SCAN_TYPE)
            )
            writer.write(rotten, int(stamp * 1e9), _rotten_message(stamp))
    return path


def _rotten_message(seconds):
    sec = int(seconds)
    return b"\x00\x01\x00\x00" + struct.pack("<iIIB", sec, 0, 1, 0) + b"\x00"


@pytest.fixture
def shifted_bag(tmp_path):
    """Constant stamp shift that puts every stamp in 1970, implausible but readable."""
    return write_bag(tmp_path / "shifted", SHIFTED_STAMPS, SHIFTED_STAMPS)


@pytest.fixture
def truncated_bag(tmp_path):
    """Storage intact, metadata cut in half, so no reader version can open the bag."""
    path = write_bag(tmp_path / "truncated", STAMPS, RECEIVE_TIMES)
    metadata = path / "metadata.yaml"
    text = metadata.read_text()
    metadata.write_text(text[: len(text) // 2])
    return path


@pytest.fixture
def rotten_bag(tmp_path):
    return write_rotten_mcap(tmp_path / "rotten", STAMPS)


@pytest.fixture
def ros1_bag(tmp_path):
    return write_ros1_bag(tmp_path / "ros1.bag", STAMPS, RECEIVE_TIMES)


@pytest.fixture
def ros1_odd_bag(tmp_path):
    """Extra topic whose first field is a uint32 named header, so offset 4 holds no stamp."""
    return write_ros1_bag(tmp_path / "odd.bag", STAMPS[:2], odd=True)
