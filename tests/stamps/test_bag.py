import struct

import pytest

from rosio.stamps import read as bag
from rosio.stamps.read import _stamp_ros1, read_topic_series, stamp_from_cdr

from .conftest import RECEIVE_TIMES, STAMPS


def cdr_little(sec, nanosec):
    return b"\x00\x01\x00\x00" + struct.pack("<iI", sec, nanosec)


def ros1_header(sec, nanosec, seq=0):
    return struct.pack("<III", seq, sec, nanosec)


def test_fast_path_little_endian():
    assert stamp_from_cdr(cdr_little(12, 500000000)) == pytest.approx(12.5)


def test_fast_path_big_endian():
    raw = b"\x00\x00\x00\x00" + struct.pack(">iI", 7, 250000000)
    assert stamp_from_cdr(raw) == pytest.approx(7.25)


def test_fast_path_negative_seconds():
    assert stamp_from_cdr(cdr_little(-3, 0)) == pytest.approx(-3.0)


def test_ros1_fast_path():
    assert _stamp_ros1(ros1_header(12, 500000000)) == pytest.approx(12.5)


def test_ros1_fast_path_ignores_seq():
    assert _stamp_ros1(ros1_header(7, 250000000, seq=4294967295)) == pytest.approx(7.25)


def test_read_topic_series(clean_bag):
    series = read_topic_series(clean_bag)
    assert list(series) == ["/scan"]
    entry = series["/scan"]
    assert entry.msgtype == "sensor_msgs/msg/LaserScan"
    assert entry.stamps == pytest.approx(STAMPS)
    assert entry.receive_times == pytest.approx(RECEIVE_TIMES)


def test_read_topic_series_limit(clean_bag):
    entry = read_topic_series(clean_bag, limit=2)["/scan"]
    assert len(entry.stamps) == 2
    assert len(entry.receive_times) == 2


def test_read_topic_series_topics_filter(clean_bag):
    assert read_topic_series(clean_bag, topics=["/chatter"]) == {}
    assert list(read_topic_series(clean_bag, topics=["/scan"])) == ["/scan"]


def test_fast_path_is_used(clean_bag, monkeypatch):
    calls = []
    original = bag.stamp_from_cdr
    monkeypatch.setattr(bag, "stamp_from_cdr", lambda raw: calls.append(1) or original(raw))
    assert len(read_topic_series(clean_bag)["/scan"].stamps) == 4
    assert len(calls) == 5  # one probe plus one per message


def test_slow_path_fallback(clean_bag, monkeypatch):
    monkeypatch.setattr(bag, "stamp_from_cdr", lambda raw: 0.0)
    entry = read_topic_series(clean_bag)["/scan"]
    assert entry.stamps == pytest.approx(STAMPS)


def test_read_topic_series_ros1(ros1_bag):
    series = read_topic_series(ros1_bag)
    assert list(series) == ["/scan"]
    entry = series["/scan"]
    assert entry.msgtype == "sensor_msgs/msg/LaserScan"
    assert entry.stamps == pytest.approx(STAMPS)
    assert entry.receive_times == pytest.approx(RECEIVE_TIMES)


def test_ros1_fast_path_is_used(ros1_bag, monkeypatch):
    fast, slow = [], []
    original = bag._stamp_ros1
    monkeypatch.setattr(bag, "_stamp_ros1", lambda raw: fast.append(1) or original(raw))
    deserialize = bag._stamp_slow
    monkeypatch.setattr(bag, "_stamp_slow", lambda *a: slow.append(1) or deserialize(*a))
    assert len(read_topic_series(ros1_bag)["/scan"].stamps) == 4
    assert len(fast) == 5  # one probe plus one per message
    assert len(slow) == 1  # only the probe deserializes


def test_ros1_probe_rejects_lying_stamp(ros1_bag, monkeypatch):
    monkeypatch.setattr(bag, "_stamp_ros1", lambda raw: _stamp_ros1(raw) + 1.0)
    entry = read_topic_series(ros1_bag)["/scan"]
    assert entry.stamps == pytest.approx(STAMPS)


def test_ros1_first_field_named_header_is_skipped(ros1_odd_bag):
    assert list(read_topic_series(ros1_odd_bag)) == ["/scan"]


def test_read_topic_series_missing_bag(tmp_path):
    with pytest.raises((OSError, RuntimeError, ValueError)):
        read_topic_series(tmp_path / "absent")
