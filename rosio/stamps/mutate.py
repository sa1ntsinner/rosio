"""Inject known timestamp pathologies into a bag, one positive control per detector.

Default magnitudes come from reported failures:

- gps_leap, 18 s: GPS to UTC leap offset since 2017, a receiver serving GPS time as UTC.
- tai, 37 s: TAI to UTC offset, a PTP grandmaster left on the TAI timescale.
- ouster_ptp, 74986812 s: ouster-ros issue 366, sensor stamps stuck near the PTP epoch.
- septentrio, 9000 s: septentrio_gnss_driver issue 142, about 2.5 h of stamp offset.
- backward, 1.5 s: a single backward step, the shape of a clock correction mid recording.
- drift, 0.005 s per s: an offset growing linearly from --at onward, a free running source
  clock. Here --magnitude is the slope in seconds per second, not an amount of seconds.
- dropout, 2.0 s: a window of messages missing from the recording, Hilti 2021 issue 13.
- frozen, 25 messages, and zero, 5 messages: a driver that stops updating the stamp.

Stamps are patched in the raw CDR buffer through the shared encapsulation decoder, which
takes endianness and the stamp offset from the encapsulation identifier and refuses any
form it cannot read by name. That works for every message type whose first field is a
std_msgs/msg/Header. All other topics, connections and record timestamps are copied
untouched, only dropout removes whole records.
"""

import argparse
import sys
from pathlib import Path

from .fallback import encapsulation
from .read import probe_stamp_parser, starts_with_header

NS = 1000000000
PRESETS = {"gps_leap": 18.0, "tai": 37.0, "ouster_ptp": 74986812.0, "septentrio": 9000.0}
DEFAULT_MAGNITUDE = {
    "offset": 1.0,
    "backward": 1.5,
    "drift": 0.005,
    "dropout": 2.0,
    "frozen": 0.0,
    "zero": 0.0,
}
DEFAULT_LENGTH = {"frozen": 25, "zero": 5}
PATHOLOGIES = ("offset", "backward", "drift", "frozen", "dropout", "zero", *PRESETS)


def mutate(src, dest, pathology, topics=None, magnitude=None, at=0.5, length=None):
    """Write a mutated copy of src to dest, return per topic counts."""
    pathology, magnitude, length = resolve(pathology, magnitude, length)
    reader_type, typestore = _any_reader()
    try:
        with reader_type([Path(src)], default_typestore=typestore) as reader:
            wanted = _wanted(reader, topics)
            stamps = _scan(reader, wanted)
            plans = {
                topic: _plan(pathology, series, magnitude, at, length)
                for topic, series in stamps.items()
            }
            report = _report(stamps, plans)
            _refuse_empty(report)
            _write(reader, Path(dest), plans)
    except Exception as exc:
        raise ValueError(str(exc)) from exc
    return report


def _report(stamps, plans):
    return {
        topic: {
            "messages": len(stamps[topic]),
            "mutated": sum(
                1 for a, b in zip(stamps[topic], plan, strict=True) if b is not None and b != a
            ),
            "dropped": sum(1 for stamp in plan if stamp is None),
        }
        for topic, plan in plans.items()
    }


def _refuse_empty(report):
    """A positive control that injected nothing is a failure, not a success."""
    if not report:
        raise ValueError("no topic to mutate, none carries a readable header stamp")
    if not sum(counts["mutated"] + counts["dropped"] for counts in report.values()):
        raise ValueError("no message was mutated, the copy would carry no positive control")


def resolve(pathology, magnitude, length):
    if pathology in PRESETS:
        magnitude = PRESETS[pathology] if magnitude is None else magnitude
        pathology = "offset"
    if pathology not in DEFAULT_MAGNITUDE:
        raise ValueError(f"unknown pathology {pathology}")
    if magnitude is None:
        magnitude = DEFAULT_MAGNITUDE[pathology]
    if length is None:
        length = DEFAULT_LENGTH.get(pathology, 0)
    return pathology, magnitude, length


def _plan(pathology, stamps, magnitude, at, length):
    """New stamp in nanoseconds per message index, None marks a record to delete."""
    count = len(stamps)
    if count == 0:
        return []
    start = min(int(at * count), count - 1)
    shift = _to_ns(magnitude)
    if pathology == "offset":
        return [stamp + shift for stamp in stamps]
    if pathology == "backward":
        return [stamp - shift if i >= start else stamp for i, stamp in enumerate(stamps)]
    if pathology == "drift":
        return [
            stamp + round(magnitude * (stamp - stamps[start])) if i >= start else stamp
            for i, stamp in enumerate(stamps)
        ]
    if pathology == "frozen":
        return [
            stamps[start] if start <= i < start + length else stamp
            for i, stamp in enumerate(stamps)
        ]
    if pathology == "zero":
        return [0 if start <= i < start + length else stamp for i, stamp in enumerate(stamps)]
    end = stamps[start] + shift
    return [None if i >= start and stamp < end else stamp for i, stamp in enumerate(stamps)]


def _wanted(reader, topics):
    header_topics = {c.topic for c in reader.connections if starts_with_header(reader, c)}
    if topics is None:
        return header_topics
    missing = set(topics) - header_topics
    if missing:
        raise ValueError("no patchable header topic: {}".format(", ".join(sorted(missing))))
    return set(topics)


def _scan(reader, wanted):
    if not wanted:
        # An empty connection list reads every connection, so stop before the reader does.
        return {}
    stamps = {}
    patchable = {}
    connections = [c for c in reader.connections if c.topic in wanted]
    for connection, _, raw in reader.messages(connections=connections):
        topic = connection.topic
        data = _cdr(reader, connection, raw)
        encapsulation(data)
        if topic not in patchable:
            patchable[topic] = not reader.is2 or probe_stamp_parser(reader, connection, raw)
            if patchable[topic]:
                stamps[topic] = []
        if patchable[topic]:
            stamps[topic].append(_read_ns(data))
    return stamps


def _write(reader, dest, plans):
    from rosbags.rosbag2 import Writer

    seen = dict.fromkeys(plans, 0)
    with Writer(dest, version=8) as writer:
        outputs = {c.id: _add_connection(writer, reader, c) for c in reader.connections}
        for connection, timestamp, raw in reader.messages():
            topic = connection.topic
            data = _cdr(reader, connection, raw)
            plan = plans.get(topic)
            if plan is not None:
                stamp = plan[seen[topic]]
                seen[topic] += 1
                if stamp is None:
                    continue
                data = _patch(data, stamp)
            writer.write(outputs[connection.id], timestamp, data)


def _add_connection(writer, reader, connection):
    from rosbags.interfaces import MessageDefinitionFormat

    qos = list(getattr(connection.ext, "offered_qos_profiles", ()) or ())
    reusable = (
        reader.is2
        and connection.digest
        and connection.msgdef.format is not MessageDefinitionFormat.NONE
    )
    if reusable:
        return writer.add_connection(
            connection.topic,
            connection.msgtype,
            msgdef=connection.msgdef.data,
            rihs01=connection.digest,
            offered_qos_profiles=qos,
        )
    return writer.add_connection(
        connection.topic, connection.msgtype, typestore=reader.typestore, offered_qos_profiles=qos
    )


def _cdr(reader, connection, raw):
    if reader.is2:
        return raw
    message = reader.deserialize(raw, connection.msgtype)
    return reader.typestore.serialize_cdr(message, connection.msgtype)


def _read_ns(data):
    _, layout, offset = encapsulation(data)
    sec, nanosec = layout.unpack_from(data, offset)
    return sec * NS + nanosec


def _patch(data, ns):
    _, layout, offset = encapsulation(data)
    out = bytearray(data)
    layout.pack_into(out, offset, ns // NS, ns % NS)
    return bytes(out)


def _to_ns(seconds):
    return round(seconds * NS)


def _any_reader():
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "mutating bags needs the rosbags package, install it with: pip install rosbags"
        ) from exc
    return AnyReader, get_typestore(Stores.LATEST)


EXAMPLE_BASE_S = 1750000000.0
EXAMPLE_MESSAGES = 120
EXAMPLE_PERIOD_S = 0.1
EXAMPLE_LATENCY_S = 0.02


def example(dest):
    """A clean recording to put a known fault into: /scan and /imu at 10 Hz for 12 s, every
    message received 20 ms after its stamp. It is built on the reader's machine, so the
    README example runs without a recording of one's own and none ships with the package."""
    try:
        import numpy
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "writing a recording needs the rosbags package, install it with: "
            "pip install 'rosio[stamps]'"
        ) from exc
    types = get_typestore(Stores.ROS2_HUMBLE)
    store = types.types

    def header(seconds, frame):
        whole = int(seconds)
        stamp = store["builtin_interfaces/msg/Time"](
            sec=whole, nanosec=round((seconds - whole) * NS)
        )
        return store["std_msgs/msg/Header"](stamp=stamp, frame_id=frame)

    def scan(seconds):
        return store["sensor_msgs/msg/LaserScan"](
            header=header(seconds, "laser"),
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=1.0,
            time_increment=0.0,
            scan_time=EXAMPLE_PERIOD_S,
            range_min=0.1,
            range_max=10.0,
            ranges=numpy.array([1.0, 2.0, 3.0], dtype=numpy.float32),
            intensities=numpy.array([], dtype=numpy.float32),
        )

    def imu(seconds):
        still = store["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
        covariance = numpy.zeros(9, dtype=numpy.float64)
        return store["sensor_msgs/msg/Imu"](
            header=header(seconds, "imu"),
            orientation=store["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=covariance,
            angular_velocity=still,
            angular_velocity_covariance=covariance,
            linear_acceleration=still,
            linear_acceleration_covariance=covariance,
        )

    with Writer(Path(dest), version=8) as writer:
        topics = [
            (writer.add_connection("/imu", "sensor_msgs/msg/Imu", typestore=types), imu),
            (writer.add_connection("/scan", "sensor_msgs/msg/LaserScan", typestore=types), scan),
        ]
        for index in range(EXAMPLE_MESSAGES):
            stamp = EXAMPLE_BASE_S + index * EXAMPLE_PERIOD_S
            for connection, build in topics:
                data = types.serialize_cdr(build(stamp), connection.msgtype)
                writer.write(connection, round((stamp + EXAMPLE_LATENCY_S) * NS), data)
    return dict.fromkeys(("/imu", "/scan"), EXAMPLE_MESSAGES)


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="rosio-bagmutate", description="inject known timestamp pathologies into a bag"
    )
    parser.add_argument("src", nargs="?", help="path to a ROS 1 or ROS 2 bag")
    parser.add_argument("dest", nargs="?", help="path of the rosbag2 to write")
    parser.add_argument("--pathology", choices=PATHOLOGIES)
    parser.add_argument(
        "--example",
        metavar="DEST",
        help="write a clean synthetic recording to DEST to put a fault into, and stop",
    )
    parser.add_argument(
        "--topic", action="append", metavar="TOPIC", help="only mutate this topic, repeatable"
    )
    parser.add_argument(
        "--magnitude",
        type=float,
        metavar="X",
        help="seconds, or seconds per second for drift, default per pathology",
    )
    parser.add_argument("--at", type=float, default=0.5, metavar="FRACTION", help="where to start")
    parser.add_argument("--length", type=int, metavar="N", help="messages for frozen and zero")
    args = parser.parse_args(argv)

    if args.example:
        try:
            written = example(args.example)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"rosio-bagmutate: {exc}", file=sys.stderr)
            return 2
        for topic, count in sorted(written.items()):
            print(f"{topic} clean msgs={count}")
        return 0
    if not (args.src and args.dest and args.pathology):
        parser.error("a source bag, a destination and --pathology are needed, or --example DEST")
    if not 0.0 <= args.at < 1.0:
        print("rosio-bagmutate: --at needs a fraction in [0, 1)", file=sys.stderr)
        return 2
    if args.length is not None and args.length < 1:
        print("rosio-bagmutate: --length needs a positive number", file=sys.stderr)
        return 2

    try:
        report = mutate(
            args.src,
            args.dest,
            args.pathology,
            topics=args.topic,
            magnitude=args.magnitude,
            at=args.at,
            length=args.length,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"rosio-bagmutate: {exc}", file=sys.stderr)
        return 2

    for topic, counts in sorted(report.items()):
        print(
            "{} {} msgs={} mutated={} dropped={}".format(
                topic, args.pathology, counts["messages"], counts["mutated"], counts["dropped"]
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
