"""Fallback readers, driven through their own entry points.

No test here asserts that rosbags refuses a bag. That claim expires the day upstream
fixes the defect, and it did on 0.11.4. The inputs below are damaged recordings, which
stay damaged whatever the reader version does.
"""

import struct

import pytest

from rosio.stamps.fallback import (
    UnsupportedEncapsulation,
    cdr_stamp_parts,
    encapsulation,
    read_sqlite3,
    tolerant_reader,
)
from rosio.stamps.read import read_topic_series, read_topic_series_report

from .conftest import RECEIVE_TIMES, ROTTEN_TYPE, SHIFTED_STAMPS, STAMPS, TYPESTORE


def test_sqlite3_reader_returns_stamps_receive_times_and_type(clean_bag):
    series, _ = read_sqlite3(clean_bag)
    assert series["/scan"].stamps == pytest.approx(STAMPS)
    assert series["/scan"].receive_times == pytest.approx(RECEIVE_TIMES)
    assert series["/scan"].msgtype == "sensor_msgs/msg/LaserScan"


def test_sqlite3_reader_drops_topics_without_a_header(clean_bag):
    series, notices = read_sqlite3(clean_bag)
    assert "/chatter" not in series
    assert notices == [
        "sqlite3 fallback dropped topic /chatter, no header stamp at the head of its messages"
    ]


def test_sqlite3_reader_keeps_implausible_stamps(shifted_bag):
    series, _ = read_sqlite3(shifted_bag)
    assert series["/scan"].stamps == pytest.approx(SHIFTED_STAMPS)


def test_sqlite3_reader_honours_topics_and_limit(clean_bag):
    series, _ = read_sqlite3(clean_bag, topics=["/scan"], limit=2)
    assert list(series) == ["/scan"]
    assert series["/scan"].stamps == pytest.approx(STAMPS[:2])


def test_sqlite3_reader_refuses_a_path_without_storage(tmp_path):
    with pytest.raises(ValueError):
        read_sqlite3(tmp_path)


def test_tolerant_reader_skips_only_the_damaged_topic(rotten_bag):
    with tolerant_reader(rotten_bag, TYPESTORE) as reader:
        assert reader.skipped == {"/rotten": ROTTEN_TYPE}
        assert [c.topic for c in reader.connections] == ["/scan"]


def test_tolerant_reader_notice_names_topic_and_type(rotten_bag):
    with tolerant_reader(rotten_bag, TYPESTORE) as reader:
        assert reader.notices() == [
            "skipped topic /rotten, rosbags cannot parse type " + ROTTEN_TYPE
        ]


def test_damaged_type_does_not_take_the_bag_down(rotten_bag):
    report = read_topic_series_report(rotten_bag)
    assert report.mode == "tolerant"
    assert list(report.series) == ["/scan"]
    assert report.series["/scan"].stamps == pytest.approx(STAMPS)


def test_read_topic_series_collects_notices(rotten_bag):
    notices = []
    series = read_topic_series(rotten_bag, notices=notices)
    assert list(series) == ["/scan"]
    assert len(notices) == 1


def test_truncated_bag_degrades_to_the_sqlite3_reader(truncated_bag):
    report = read_topic_series_report(truncated_bag)
    assert report.mode == "sqlite3"
    assert report.notices[0].startswith("sqlite3 fallback reader")
    assert report.series["/scan"].stamps == pytest.approx(STAMPS)
    assert report.series["/scan"].receive_times == pytest.approx(RECEIVE_TIMES)


def test_readable_bag_stays_on_the_plain_reader(clean_bag):
    report = read_topic_series_report(clean_bag)
    assert report.mode == "reader"
    assert report.notices == []


def test_missing_bag_still_raises(tmp_path):
    with pytest.raises(ValueError):
        read_topic_series(tmp_path / "nothing")


def encapsulated(identifier, sec=7, nanosec=250000000, dheader=False):
    order = "<" if identifier % 2 else ">"
    body = struct.pack(order + "I", 12) if dheader else b""
    return (
        struct.pack(">H", identifier) + b"\x00\x00" + body + struct.pack(order + "iI", sec, nanosec)
    )


@pytest.mark.parametrize("identifier", [0x0000, 0x0001, 0x0006, 0x0007])
def test_encapsulation_reads_a_stamp_at_offset_four(identifier):
    assert cdr_stamp_parts(encapsulated(identifier, 12, 500000000)) == (12, 500000000)
    assert encapsulation(encapsulated(identifier))[2] == 4


@pytest.mark.parametrize("identifier", [0x0008, 0x0009])
def test_encapsulation_delimited_xcdr2_reads_past_the_dheader(identifier):
    raw = encapsulated(identifier, 9, 125000000, dheader=True)
    assert cdr_stamp_parts(raw) == (9, 125000000)
    assert encapsulation(raw)[2] == 8


def test_encapsulation_endianness_comes_from_the_identifier():
    """Byte 1 is nonzero for both members of the XCDR2 pair, so its truthiness proves nothing."""
    body = struct.pack(">iI", 3, 1)
    assert cdr_stamp_parts(struct.pack(">H", 0x0006) + b"\x00\x00" + body) == (3, 1)
    assert cdr_stamp_parts(struct.pack(">H", 0x0007) + b"\x00\x00" + body) == struct.unpack(
        "<iI", body
    )


@pytest.mark.parametrize(
    "identifier,name",
    [
        (0x0002, "parameter list CDR"),
        (0x0003, "parameter list CDR"),
        (0x0004, "invalid identifier"),
        (0x0005, "invalid identifier"),
        (0x000A, "parameter list XCDR2"),
        (0x000B, "parameter list XCDR2"),
    ],
)
def test_encapsulation_refuses_an_unsupported_identifier_by_name(identifier, name):
    with pytest.raises(UnsupportedEncapsulation) as caught:
        cdr_stamp_parts(encapsulated(identifier))
    assert caught.value.identifier == identifier
    assert caught.value.name == name
    assert str(caught.value) == f"unsupported encapsulation {name} 0x{identifier:04x}"


def test_encapsulation_refuses_an_unknown_identifier():
    with pytest.raises(UnsupportedEncapsulation) as caught:
        cdr_stamp_parts(encapsulated(0x00FF))
    assert caught.value.identifier == 0x00FF
    assert str(caught.value) == "unsupported encapsulation unknown 0x00ff"
