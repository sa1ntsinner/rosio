import json

import numpy
import pytest
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from rosio.stamps.cli import main

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
IMU_TYPE = "sensor_msgs/msg/Imu"
BASE_S = 1700000000.0
LATENCY_S = 0.005
PERIOD_S = 0.1
TAI_OFFSET_S = 37.0

TOPICS = ("/imu/clean", "/imu/tai", "/imu/frozen", "/imu/dropout")


def imu_message(seconds):
    header = TYPESTORE.types["std_msgs/msg/Header"](
        stamp=TYPESTORE.types["builtin_interfaces/msg/Time"](
            sec=int(seconds), nanosec=round((seconds - int(seconds)) * 1e9)
        ),
        frame_id="imu_link",
    )
    zeros = numpy.zeros(9, dtype=numpy.float64)
    vector = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    return TYPESTORE.types[IMU_TYPE](
        header=header,
        orientation=TYPESTORE.types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=zeros.copy(),
        angular_velocity=vector(x=0.0, y=0.0, z=0.0),
        angular_velocity_covariance=zeros.copy(),
        linear_acceleration=vector(x=0.0, y=0.0, z=9.81),
        linear_acceleration_covariance=zeros.copy(),
    )


def clean_track(count=60):
    return [(BASE_S + i * PERIOD_S + LATENCY_S, BASE_S + i * PERIOD_S) for i in range(count)]


def tai_track(count=60):
    return [(receive, stamp - TAI_OFFSET_S) for receive, stamp in clean_track(count)]


def frozen_track(count=40, start=15, run=16):
    track = clean_track(count)
    held = track[start][1]
    return [
        (receive, held if start <= i < start + run else stamp)
        for i, (receive, stamp) in enumerate(track)
    ]


def dropout_track(count=60, at=30, gap=2.0):
    shift = [0.0 if i < at else gap for i in range(count)]
    return [
        (receive + shift[i], stamp + shift[i])
        for i, (receive, stamp) in enumerate(clean_track(count))
    ]


@pytest.fixture
def mixed_bag(tmp_path):
    path = tmp_path / "mixed"
    tracks = {
        "/imu/clean": clean_track(),
        "/imu/tai": tai_track(),
        "/imu/frozen": frozen_track(),
        "/imu/dropout": dropout_track(),
    }
    records = []
    with Writer(path, version=8) as writer:
        for topic, track in tracks.items():
            connection = writer.add_connection(topic, IMU_TYPE, typestore=TYPESTORE)
            for receive, stamp in track:
                records.append((receive, connection, stamp))
        for receive, connection, stamp in sorted(records, key=lambda r: r[0]):
            writer.write(
                connection,
                int(receive * 1e9),
                TYPESTORE.serialize_cdr(imu_message(stamp), IMU_TYPE),
            )
    return path


def findings_of(capsys, bag, extra=()):
    code = main([str(bag), "--json", *extra])
    return code, json.loads(capsys.readouterr().out)


def test_mixed_bag_exits_one(capsys, mixed_bag):
    code, findings = findings_of(capsys, mixed_bag)
    assert code == 1
    assert findings


def test_tai_offset_classified(capsys, mixed_bag):
    _, findings = findings_of(capsys, mixed_bag)
    offsets = [f for f in findings if f["check"] == "epoch_offset"]
    assert [f["topic"] for f in offsets] == ["/imu/tai"]
    assert offsets[0]["severity"] == "error"
    assert offsets[0]["evidence"]["class"] == "tai utc offset"
    assert offsets[0]["evidence"]["median_offset_s"] == pytest.approx(
        TAI_OFFSET_S + LATENCY_S, abs=1e-6
    )


def test_frozen_and_gaps_present(capsys, mixed_bag):
    _, findings = findings_of(capsys, mixed_bag)
    frozen = [f for f in findings if f["check"] == "frozen_stamps"]
    assert [f["topic"] for f in frozen] == ["/imu/frozen"]
    assert frozen[0]["evidence"]["run_length"] == 16
    gaps = [f for f in findings if f["check"] == "stamp_gaps"]
    assert "/imu/dropout" in [f["topic"] for f in gaps]


def test_clean_topic_has_no_errors(capsys, mixed_bag):
    _, findings = findings_of(capsys, mixed_bag)
    clean = [f for f in findings if f["topic"] == "/imu/clean"]
    assert clean
    assert [f for f in clean if f["severity"] == "error"] == []


def test_text_output_is_sorted_by_severity(capsys, mixed_bag):
    assert main([str(mixed_bag)]) == 1
    lines = capsys.readouterr().out.split("\nverdict:")[0].strip().splitlines()
    ranks = [{"ERROR": 0, "WARNING": 1, "INFO": 2}[line.split()[0]] for line in lines]
    assert ranks == sorted(ranks)


def test_stats_prints_a_line_per_topic(capsys, mixed_bag):
    assert main([str(mixed_bag), "--stats", "--json"]) == 1
    out = capsys.readouterr().out
    stats = [line for line in out.splitlines() if line.startswith("/imu/")]
    assert sorted(line.split()[0] for line in stats) == sorted(TOPICS)
    for line in stats:
        assert "msgs=" in line and "duration=" in line and "rate=" in line and "offset=" in line


def test_every_registered_check_runs(capsys, mixed_bag):
    from rosio.stamps.checks import CHECKS

    _, findings = findings_of(capsys, mixed_bag)
    reported = {f["check"] for f in findings}
    assert reported <= set(CHECKS)
    assert {
        "epoch_offset",
        "inter_topic_skew",
        "frozen_stamps",
        "stamp_gaps",
        "stamp_rate",
    } <= reported
