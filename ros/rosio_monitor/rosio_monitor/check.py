import argparse
import math
import sys
import time

import rclpy
from rclpy.exceptions import ParameterException
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args

from rosio.stamps.checks import CHECKS
from rosio.stamps.model import TopicSeries
from rosio.stamps.verdict import NOT_ANALYZED, diagnose
from rosio_monitor import time_sources
from rosio_monitor.monitor_node import MonitorNode, ParameterError, first_line

DEFAULT_DURATION_S = 10.0
MAX_DURATION_S = 3600.0
CAPTURE_WINDOW = 50000
DISCOVERY_PERIOD_S = 1.0
SPIN_TIMEOUT_S = 0.05
SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}
SOURCE_LEVEL_NAMES = {time_sources.OK: "OK", time_sources.WARN: "WARN", time_sources.ERROR: "ERROR"}
NOTHING_ANALYZED_CODE = 2


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="ros2 run rosio_monitor check",
        description="capture live topics for a few seconds and diagnose their timestamps",
    )
    parser.add_argument("--duration", type=float, metavar="SECONDS", help="capture length")
    parser.add_argument(
        "--topics", action="append", metavar="TOPIC", help="only watch this topic, repeatable"
    )
    return parser.parse_args(argv)


def validate_duration(duration_s):
    """A capture length the tool can actually honour, in seconds."""
    if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)):
        raise ValueError(f"duration must be a number of seconds, got {duration_s!r}")
    if not math.isfinite(duration_s):
        raise ValueError(f"duration must be finite, got {duration_s!r}")
    if duration_s <= 0.0:
        raise ValueError(f"duration must be positive, got {duration_s}")
    if duration_s > MAX_DURATION_S:
        raise ValueError(f"duration must not exceed {MAX_DURATION_S:.0f} s, got {duration_s}")
    return float(duration_s)


def to_series(windows, types):
    series = {}
    for topic in sorted(windows):
        window = windows[topic]
        series[topic] = TopicSeries(
            topic=topic,
            msgtype=types.get(topic, ""),
            stamps=list(window.stamps),
            receive_times=list(window.arrivals),
        )
    return series


def run_checks(series):
    findings = []
    for check in CHECKS.values():
        findings.extend(check(series))
    findings.sort(key=lambda f: (SEVERITY_ORDER.get(f.severity, 3), f.check, f.topic))
    return findings


def exit_code(findings, sources):
    if any(f.severity == "error" for f in findings):
        return 1
    if any(level >= time_sources.ERROR for level, _, _, _ in sources):
        return 1
    return 0


def format_report(duration_s, series, findings, verdict, sources):
    messages = sum(len(entry.stamps) for entry in series.values())
    lines = [f"capture: {duration_s:.1f} s, {len(series)} topics, {messages} messages"]
    if findings:
        lines.append("")
        for finding in findings:
            lines.append(
                f"{finding.severity.upper()} {finding.check} {finding.topic}: {finding.message}"
            )
    lines.append("")
    lines.append("verdict:")
    for hypothesis in verdict.hypotheses:
        lines.append(f"  {hypothesis}")
    lines.append("")
    lines.append("time sources:")
    for level, name, message, _ in sources:
        lines.append("  {} {}: {}".format(SOURCE_LEVEL_NAMES.get(level, "?"), name, message))
    return "\n".join(lines)


def merge_sources(start, end):
    """The end poll is the report, a level change during the capture earns its own line."""
    worst_start = max([level for level, _, _, _ in start], default=time_sources.OK)
    worst_end = max([level for level, _, _, _ in end], default=time_sources.OK)
    if worst_start == worst_end:
        return list(end)
    message = (
        "time source health changed during the capture, {} at the start and {} at the end".format(
            SOURCE_LEVEL_NAMES.get(worst_start, "?"), SOURCE_LEVEL_NAMES.get(worst_end, "?")
        )
    )
    return [*end, (max(worst_start, worst_end), "change", message, {})]


def poll_sources(node):
    """The node owns the poller settings, a zero period switches the poll off here too."""
    if node.time_source_period_s <= 0.0:
        return []
    return time_sources.assess(time_sources.poll(config=node.source_config), node.thresholds)


def capture(node, duration_s):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
    finally:
        executor.shutdown()
    node.prune()
    types = {topic: names[0] for topic, names in node.get_topic_names_and_types() if names}
    return to_series(node.windows, types)


def diagnose_capture(duration_s, series, sources):
    findings = run_checks(series)
    verdict = diagnose(series, findings)
    code = exit_code(findings, sources)
    if not series:
        verdict.hypotheses = [NOT_ANALYZED]
        verdict.level = "unknown"
        code = NOTHING_ANALYZED_CODE
    return format_report(duration_s, series, findings, verdict, sources), code


def main(args=None):
    argv = list(sys.argv if args is None else args)
    options = parse_args(remove_ros_args(argv)[1:])
    try:
        rclpy.init(args=argv)
    except Exception as exc:
        print(f"rosio_monitor: rclpy could not start: {exc}", file=sys.stderr)
        return 2

    node = None
    try:
        overrides = [
            Parameter("window_size", value=CAPTURE_WINDOW),
            Parameter("discovery_period_s", value=DISCOVERY_PERIOD_S),
        ]
        if options.topics:
            overrides.append(Parameter("topics", value=list(options.topics)))
        node = MonitorNode(parameter_overrides=overrides)
        node.declare_parameter("duration_s", DEFAULT_DURATION_S)
        duration_s = options.duration
        if duration_s is None:
            duration_s = node.get_parameter("duration_s").value
        duration_s = validate_duration(duration_s)
        start = poll_sources(node)
        series = capture(node, duration_s)
        sources = merge_sources(start, poll_sources(node))
    except (ParameterError, ParameterException, ValueError) as exc:
        print(f"rosio_monitor: {first_line(exc)}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"rosio_monitor: capture failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    report, code = diagnose_capture(duration_s, series, sources)
    print(report)
    return code


if __name__ == "__main__":
    sys.exit(main())
