import sqlite3
import struct
from dataclasses import dataclass, field

from .fallback import UnsupportedEncapsulation, cdr_stamp_parts, read_sqlite3, tolerant_reader
from .model import TopicSeries

HEADER_TYPE = "std_msgs/msg/Header"
PROBE_TOLERANCE_S = 1e-9
_STAMP_LE = struct.Struct("<iI")


@dataclass
class ReadReport:
    """Series read from a bag plus what the reader had to give up on."""

    series: dict
    mode: str
    notices: list = field(default_factory=list)


def read_topic_series(path, topics=None, limit=None, notices=None):
    report = read_topic_series_report(path, topics, limit)
    if notices is not None:
        notices.extend(report.notices)
    return report.series


def read_topic_series_report(path, topics=None, limit=None):
    typestore = _typestore()
    try:
        with tolerant_reader(path, typestore) as reader:
            series = _collect(reader, topics, limit)
            mode = "tolerant" if reader.skipped else "reader"
            return ReadReport(series, mode, reader.notices())
    except Exception as exc:
        opening = exc
    try:
        series, notices = read_sqlite3(path, topics, limit)
    except (OSError, ValueError, sqlite3.Error):
        raise ValueError(str(opening)) from opening
    notices.insert(0, f"sqlite3 fallback reader, rosbags could not open the bag: {opening}")
    return ReadReport(series, "sqlite3", notices)


def _collect(reader, topics, limit):
    wanted = None if topics is None else set(topics)
    series = {}
    fast = {}
    filled = set()
    stamp_fast = _stamp_parser(reader)

    connections = [
        c
        for c in reader.connections
        if starts_with_header(reader, c) and (wanted is None or c.topic in wanted)
    ]
    if not connections:
        return series
    total = len({c.topic for c in connections})
    for connection, timestamp, raw in reader.messages(connections=connections):
        topic = connection.topic
        entry = series.get(topic)
        if entry is None:
            entry = series[topic] = TopicSeries(topic, connection.msgtype, [], [])
            fast[topic] = probe_stamp_parser(reader, connection, raw, stamp_fast)
        if limit is not None and len(entry.stamps) >= limit:
            filled.add(topic)
            if len(filled) == total:
                break
            continue
        entry.stamps.append(
            stamp_fast(raw) if fast[topic] else _stamp_slow(reader, connection, raw)
        )
        entry.receive_times.append(timestamp * 1e-9)
    return series


def _typestore():
    try:
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "reading bags needs the rosbags package, install it with: pip install rosbags"
        ) from exc
    return get_typestore(Stores.LATEST)


def starts_with_header(reader, connection):
    """True when the first field of the connection type is a std_msgs/msg/Header."""
    try:
        fields = reader.typestore.get_msgdef(connection.msgtype).fields
    except (KeyError, AttributeError):
        return False
    if not fields:
        return False
    name, (_, target) = fields[0]
    return name == "header" and target == HEADER_TYPE


def _stamp_parser(reader):
    """CDR prefixes a 4 byte encapsulation, ROS 1 a uint32 seq.

    Sec sits at offset 4 either way.
    """
    return stamp_from_cdr if reader.is2 else _stamp_ros1


def stamp_from_cdr(raw):
    """Header stamp in seconds, read straight out of a serialized CDR message."""
    sec, nanosec = cdr_stamp_parts(raw)
    return sec + nanosec * 1e-9


def _stamp_ros1(raw):
    sec, nanosec = _STAMP_LE.unpack_from(raw, 4)
    return sec + nanosec * 1e-9


def _stamp_slow(reader, connection, raw):
    stamp = reader.deserialize(raw, connection.msgtype).header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def probe_stamp_parser(reader, connection, raw, stamp_fast=None):
    """True when the raw parser agrees with the deserialized stamp of this message."""
    stamp_fast = _stamp_parser(reader) if stamp_fast is None else stamp_fast
    try:
        guess = stamp_fast(raw)
    except (struct.error, IndexError, UnsupportedEncapsulation):
        return False
    return abs(guess - _stamp_slow(reader, connection, raw)) <= PROBE_TOLERANCE_S


_stamp_cdr = stamp_from_cdr
_starts_with_header = starts_with_header
_probe = probe_stamp_parser
