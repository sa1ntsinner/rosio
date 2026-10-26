import math
import struct
import sys
import threading
import time
from collections import deque
from functools import partial
from itertools import pairwise
from statistics import median

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# rosio.stamps public API, the parser, the thresholds and the wording stay defined in one place
from rosio.stamps import (
    MAX_PLAUSIBLE_LATENCY_S,
    UnsupportedEncapsulation,
    classify_offset,
    largest_backward_step,
    longest_frozen_run,
)
from rosio.stamps import stamp_from_cdr as parse_cdr_stamp
from rosio_monitor import time_sources

DIAGNOSTICS_TOPIC = "/diagnostics"
TEN_YEARS_S = 10 * 365.25 * 86400.0
JITTER_FRACTION = 0.95
SEVERITY_LEVELS = {"error": DiagnosticStatus.ERROR, "warning": DiagnosticStatus.WARN}
HEADER_FIELD_TYPE = "std_msgs/Header"
MAX_ATTEMPTS = 5
CLOCK_STEP_S = 0.5
STALE_INTERVAL_FACTOR = 5.0
STALE_FLOOR_S = 0.5
STALE_CEILING_S = 30.0
STALE_ERROR_FACTOR = 10.0
JITTER_FLOOR_S = 0.05
DEFAULT_JITTER_WARN_FACTOR = 5.0
LISTED_UNWATCHED = 8
PERIOD_PARAMS = ("report_period_s", "discovery_period_s", "time_source_period_s")
ZERO_DISABLES = ("time_source_period_s",)
DEFAULT_EXCLUDED = ["/rosout", "/parameter_events"]
WAITING = "subscribed, waiting for the first message"
UNREAD_SOURCES = "the time sources have not been read yet"
POLLER_JOIN_TIMEOUT_S = 1.0


class ParameterError(ValueError):
    """A declared parameter carries a value the node refuses to run with."""


def first_line(text):
    """rclpy parameter exceptions are multi line, stderr gets one line."""
    for line in str(text).splitlines():
        line = line.strip()
        if line:
            return line
    return str(text)


def _number(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParameterError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ParameterError(f"{name} must be finite, got {value!r}")
    return float(value)


def _positive(name, value, zero_ok=False):
    number = _number(name, value)
    if number < 0.0 or (number == 0.0 and not zero_ok):
        wanted = "zero or positive" if zero_ok else "positive"
        raise ParameterError(f"{name} must be {wanted}, got {value}")


def _positive_int(name, value):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ParameterError(f"{name} must be a whole number, got {value!r}")
    if value < 1:
        raise ParameterError(f"{name} must be at least 1, got {value}")


def _string_list(name, value):
    if not isinstance(value, (list, tuple)) or not value:
        raise ParameterError(f"{name} must be a non empty list of strings, got {value!r}")
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise ParameterError(f"{name} entries must be non empty strings, got {item!r}")


def validate_parameters(values, thresholds=None, config=None):
    """Reject values the monitor cannot honour, before a single timer exists."""
    if config and "pmc_command" in config:
        _string_list("pmc_command", config["pmc_command"])
    for name in PERIOD_PARAMS:
        _positive(name, values[name], zero_ok=name in ZERO_DISABLES)
    _positive("plausible_window_s", values["plausible_window_s"])
    _positive("stale_after_s", values["stale_after_s"], zero_ok=True)
    _positive("jitter_warn_factor", values["jitter_warn_factor"])
    _positive_int("window_size", values["window_size"])
    for name in sorted(thresholds or {}):
        value = (thresholds or {})[name]
        if not isinstance(value, bool) and isinstance(value, (int, float)):
            _positive(name, value)


def has_leading_header(message_type):
    """True when the first field of the type is a std_msgs/Header named header."""
    fields = list(message_type.get_fields_and_field_types().items())
    if not fields:
        return False
    name, field_type = fields[0]
    return name == "header" and field_type == HEADER_FIELD_TYPE


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[min(max(index, 0), len(ordered) - 1)]


def is_frozen(stamps, arrivals):
    return len(stamps) > 1 and len(set(stamps)) == 1 and arrivals[-1] > arrivals[0]


def window_stats(stamps, arrivals):
    stamps = list(stamps)
    arrivals = list(arrivals)
    offsets = [arrival - stamp for stamp, arrival in zip(stamps, arrivals, strict=True)]
    center = median(offsets) if offsets else 0.0
    step = median(b - a for a, b in pairwise(stamps)) if len(stamps) > 1 else 0.0
    return {
        "messages": len(stamps),
        "median_offset_s": center,
        "stamp_interval_s": step,
        "rate_hz": 1.0 / step if step > 0 else 0.0,
        "p95_jitter_s": percentile([abs(offset - center) for offset in offsets], JITTER_FRACTION),
        "zero_stamps": sum(1 for stamp in stamps if stamp == 0.0),
        "frozen": is_frozen(stamps, arrivals),
        "backward_step_s": largest_backward_step(stamps)[0],
        "frozen_run": longest_frozen_run(stamps, arrivals),
    }


def level_for_offset(topic, offset, stamps):
    if abs(offset) <= MAX_PLAUSIBLE_LATENCY_S:
        message = (
            f"{topic} offset {offset:.3f} s is within the plausible latency of "
            f"{MAX_PLAUSIBLE_LATENCY_S:.1f} s"
        )
        return DiagnosticStatus.OK, message
    _, severity, message = classify_offset(topic, offset, stamps)
    return SEVERITY_LEVELS[severity], message


def stale_limit(interval_s, configured=0.0):
    """Silence budget: the parameter verbatim, else a clamped multiple of the stamp interval."""
    if configured > 0.0:
        return configured
    if interval_s <= 0.0:
        return STALE_CEILING_S
    return min(max(STALE_INTERVAL_FACTOR * interval_s, STALE_FLOOR_S), STALE_CEILING_S)


def stale_verdict(topic, stats, age_s, configured=0.0):
    limit = stale_limit(stats["stamp_interval_s"], configured)
    if age_s is None or age_s <= limit:
        return None
    level = DiagnosticStatus.ERROR if age_s > limit * STALE_ERROR_FACTOR else DiagnosticStatus.WARN
    message = (
        "{} has not published for {:.1f} s, past the {:.1f} s silence budget, "
        "it was running at {:.2f} Hz and the values below are that old"
    ).format(topic, age_s, limit, stats["rate_hz"])
    return level, message


def jitter_verdict(topic, stats, factor=DEFAULT_JITTER_WARN_FACTOR):
    limit = max(factor * stats["stamp_interval_s"], JITTER_FLOOR_S)
    if stats["p95_jitter_s"] <= limit:
        return None
    message = (
        "{} p95 jitter {:.3f} s exceeds {:.3f} s, judged against a {:.3f} s stamp period".format(
            topic, stats["p95_jitter_s"], limit, stats["stamp_interval_s"]
        )
    )
    return DiagnosticStatus.WARN, message


def zero_run_verdict(topic, stats):
    """A stamp that is only sometimes unset is not healthier than one that always is."""
    zeros = stats["zero_stamps"]
    if not zeros or zeros == stats["messages"]:
        return None
    message = "{} has {} of {} messages with a zero stamp".format(topic, zeros, stats["messages"])
    return DiagnosticStatus.ERROR, message


def frozen_run_verdict(topic, stats):
    """A run of one repeated stamp inside the window, offline thresholds verbatim."""
    if stats["frozen"] or not stats["frozen_run"]:
        return None
    run, span, stamp, _index = stats["frozen_run"]
    message = f"{topic} stamp frozen at {stamp:.9f} s for {run} messages over {span:.3f} s"
    return DiagnosticStatus.ERROR, message


def backward_verdict(topic, stats):
    """One backward step among healthy samples never moves the median, it is still a fault."""
    if stats["backward_step_s"] <= 0.0:
        return None
    message = "{} stamps go backwards, largest jump {:.9f} s".format(
        topic, stats["backward_step_s"]
    )
    return DiagnosticStatus.ERROR, message


def topic_verdict(
    topic,
    stats,
    stamps,
    age_s=None,
    stale_after_s=0.0,
    jitter_warn_factor=DEFAULT_JITTER_WARN_FACTOR,
):
    """Every finding is graded, the worst one wins, ties go to the more specific diagnosis."""
    all_zero = bool(stats["messages"]) and stats["zero_stamps"] == stats["messages"]
    findings = []
    if all_zero:
        message = f"{topic} stamps are all zero, the publisher never fills the header"
        findings.append((DiagnosticStatus.ERROR, message))
    findings.append(zero_run_verdict(topic, stats))
    if stats["frozen"]:
        message = f"{topic} stamp is frozen at {stamps[-1]:.3f} s while messages arrive"
        findings.append((DiagnosticStatus.ERROR, message))
    findings.append(frozen_run_verdict(topic, stats))
    if not all_zero and not stats["zero_stamps"]:
        findings.append(backward_verdict(topic, stats))
    findings.append(stale_verdict(topic, stats, age_s, stale_after_s))
    if not all_zero:
        findings.append(level_for_offset(topic, stats["median_offset_s"], stamps))
    findings.append(jitter_verdict(topic, stats, jitter_warn_factor))
    findings = [finding for finding in findings if finding is not None]
    return max(findings, key=lambda finding: finding[0])


def is_stale(stats, age_s, stale_after_s=0.0):
    return stale_verdict("", stats, age_s, stale_after_s) is not None


def build_status(topic, stats, level, message, age_s=None):
    status = DiagnosticStatus()
    status.name = f"rosio_monitor: {topic}"
    status.hardware_id = ""
    status.level = level
    status.message = message
    status.values = [
        KeyValue(key="median_offset_s", value="{:.6f}".format(stats["median_offset_s"])),
        KeyValue(key="rate_hz", value="{:.3f}".format(stats["rate_hz"])),
        KeyValue(key="p95_jitter_s", value="{:.6f}".format(stats["p95_jitter_s"])),
        KeyValue(key="messages", value=str(stats["messages"])),
        KeyValue(key="frozen", value=str(stats["frozen"]).lower()),
        KeyValue(key="zero_stamps", value=str(stats["zero_stamps"])),
        KeyValue(key="backward_step_s", value="{:.9f}".format(stats["backward_step_s"])),
        KeyValue(key="frozen_run", value=str(stats["frozen_run"][0] if stats["frozen_run"] else 0)),
    ]
    if age_s is not None:
        status.values.append(KeyValue(key="window_age_s", value=f"{age_s:.3f}"))
    return status


def build_time_source_status(level, name, message, values):
    status = DiagnosticStatus()
    status.name = f"rosio_monitor: time sources/{name}"
    status.hardware_id = ""
    status.level = bytes((level,))
    status.message = message
    status.values = [KeyValue(key=key, value=values[key]) for key in sorted(values)]
    return status


def build_unwatched_status(entries):
    """Every topic the monitor sees but does not measure, with the reason it does not."""
    listed = ", ".join(f"{topic} ({reason})" for topic, reason in entries[:LISTED_UNWATCHED])
    if len(entries) > LISTED_UNWATCHED:
        listed += ", ..."
    status = DiagnosticStatus()
    status.name = "rosio_monitor: unwatched"
    status.hardware_id = ""
    status.level = DiagnosticStatus.WARN
    status.message = f"{len(entries)} topics not watched: {listed}"
    status.values = [KeyValue(key=topic, value=reason) for topic, reason in entries]
    return status


def clock_step(anchor_offset, offset):
    """The wall to monotonic mapping moved far enough to call it a step, or None."""
    shift = offset - anchor_offset
    return shift if abs(shift) > CLOCK_STEP_S else None


def build_clock_step_status(shift_s):
    status = DiagnosticStatus()
    status.name = "rosio_monitor: host clock"
    status.hardware_id = ""
    status.level = DiagnosticStatus.WARN
    status.message = f"host clock stepped by {shift_s:.3f} s during monitoring"
    status.values = [KeyValue(key="step_s", value=f"{shift_s:.6f}")]
    return status


def build_summary(statuses, extra=(), unwatched=0, stale=0):
    """Counts topics only, but the level covers the time sources and the unwatched list.

    Watching nothing is never OK, an empty measurement is not a healthy one.
    """
    worst = max(
        [status.level for status in statuses] + [s.level for s in extra],
        default=DiagnosticStatus.OK,
    )
    findings = sum(1 for status in statuses if status.level != DiagnosticStatus.OK)
    if statuses:
        message = f"{len(statuses)} topics watched, {findings} with findings"
    else:
        worst = max(worst, DiagnosticStatus.WARN)
        message = "nothing is being watched"
    if stale:
        message += f", {stale} stale"
    if unwatched:
        message += f", {unwatched} not watched"
    status = DiagnosticStatus()
    status.name = "rosio_monitor: summary"
    status.hardware_id = ""
    status.level = worst
    status.message = message
    return status


class TopicWindow:
    def __init__(self, size):
        self.stamps = deque(maxlen=size)
        self.arrivals = deque(maxlen=size)
        self.last_arrival = None

    def add(self, stamp, arrival):
        self.stamps.append(stamp)
        self.arrivals.append(arrival)
        self.last_arrival = arrival

    def age(self, now):
        return None if self.last_arrival is None else now - self.last_arrival


class Attempt:
    """A topic that is not watched, why, and how often the monitor has tried."""

    def __init__(self):
        self.reason = ""
        self.attempts = 0

    @property
    def gave_up(self):
        return self.attempts >= MAX_ATTEMPTS

    def describe(self):
        return f"{self.reason}, gave up" if self.gave_up else self.reason


class SourcePoller(threading.Thread):
    """Polls the host time sources off the callback path and keeps the newest answer.

    poll runs external programs that can each take seconds on a degraded host. On the
    executor that stalls every subscription callback and distorts the arrival times the
    node measures, so it runs here and the report timer only ever reads the slot.
    """

    def __init__(self, period_s, poll):
        super().__init__(name="rosio_monitor_time_sources", daemon=True)
        self.period_s = period_s
        self.poll = poll
        self.result = None
        self.stopping = threading.Event()

    def run(self):
        while not self.stopping.is_set():
            self.result = self.poll()
            self.stopping.wait(self.period_s)

    def stop(self, timeout_s=POLLER_JOIN_TIMEOUT_S):
        self.stopping.set()
        if self.is_alive():
            self.join(timeout=timeout_s)


class MonitorNode(Node):
    def __init__(self, wall_clock=time.time, monotonic_clock=time.monotonic, **kwargs):
        super().__init__("rosio_monitor", **kwargs)
        self.wall_clock = wall_clock
        self.monotonic_clock = monotonic_clock
        self.source_poller = None
        self.declare_parameter("topics", [""])
        self.declare_parameter("exclude_topics", list(DEFAULT_EXCLUDED))
        self.declare_parameter("discovery_period_s", 5.0)
        self.declare_parameter("report_period_s", 2.0)
        self.declare_parameter("window_size", 256)
        self.declare_parameter("plausible_window_s", TEN_YEARS_S)
        self.declare_parameter("stale_after_s", 0.0)
        self.declare_parameter("jitter_warn_factor", DEFAULT_JITTER_WARN_FACTOR)
        self.declare_parameter("time_source_period_s", 10.0)
        self.declare_parameter("ptp_domain", 0)
        self.declare_parameter("pmc_socket", "")
        self.declare_parameter("pmc_command", list(time_sources.DEFAULT_PMC_COMMAND))
        self.declare_parameter("expect_no_time_sources", False)
        for key, default in time_sources.DEFAULT_THRESHOLDS.items():
            self.declare_parameter(key, default)

        periods = {name: self.get_parameter(name).value for name in PERIOD_PARAMS}
        periods["window_size"] = self.get_parameter("window_size").value
        periods["plausible_window_s"] = self.get_parameter("plausible_window_s").value
        periods["stale_after_s"] = self.get_parameter("stale_after_s").value
        periods["jitter_warn_factor"] = self.get_parameter("jitter_warn_factor").value
        self.thresholds = {
            key: self.get_parameter(key).value for key in time_sources.DEFAULT_THRESHOLDS
        }
        self.source_config = {
            "ptp_domain": self.get_parameter("ptp_domain").value,
            "pmc_socket": self.get_parameter("pmc_socket").value or None,
            "pmc_command": list(self.get_parameter("pmc_command").value or []),
        }
        validate_parameters(periods, self.thresholds, self.source_config)
        self.thresholds["expect_none"] = bool(self.get_parameter("expect_no_time_sources").value)

        self.wanted = {t for t in (self.get_parameter("topics").value or []) if t}
        excluded = {t for t in (self.get_parameter("exclude_topics").value or []) if t}
        self.excluded = excluded - self.wanted
        self.window_size = periods["window_size"]
        self.plausible_window_s = periods["plausible_window_s"]
        self.stale_after_s = float(periods["stale_after_s"])
        self.jitter_warn_factor = float(periods["jitter_warn_factor"])
        self.time_source_period_s = periods["time_source_period_s"]

        self.anchor_offset = self.clock_offset()
        self.windows = {}
        self.watched = {}
        self.unwatched = {}
        self.dropping = set()

        self.subscription_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
        self.discovery_timer = self.create_timer(periods["discovery_period_s"], self.discover)
        self.report_timer = self.create_timer(periods["report_period_s"], self.report)
        if self.time_source_period_s > 0.0:
            self.source_poller = SourcePoller(self.time_source_period_s, self.poll_time_sources)
            self.source_poller.start()
        self.discover()

    def poll_time_sources(self):
        """Runs on the poller thread, never on the executor."""
        try:
            observations = time_sources.poll(config=self.source_config)
            verdicts = time_sources.assess(observations, self.thresholds)
        except Exception as exc:
            message = f"the time source poll failed: {exc}"
            return [build_time_source_status(time_sources.WARN, "poll", message, {})]
        return [build_time_source_status(*verdict) for verdict in verdicts]

    @property
    def time_source_statuses(self):
        """The last poll result, never a wait, and never silence when nothing landed yet."""
        if self.source_poller is None:
            return []
        result = self.source_poller.result
        if result is None:
            return [build_time_source_status(time_sources.WARN, "poll", UNREAD_SOURCES, {})]
        return list(result)

    def destroy_node(self):
        if self.source_poller is not None:
            self.source_poller.stop()
            self.source_poller = None
        return super().destroy_node()

    def clock_offset(self):
        return self.wall_clock() - self.monotonic_clock()

    def arrival_now(self):
        return self.anchor_offset + self.monotonic_clock()

    def detect_clock_step(self):
        """A wall clock step invalidates every offset measured against the old anchor."""
        offset = self.clock_offset()
        shift = clock_step(self.anchor_offset, offset)
        if shift is None:
            return None
        self.anchor_offset = offset
        self.windows.clear()
        return build_clock_step_status(shift)

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if topic == DIAGNOSTICS_TOPIC or topic in self.watched or topic in self.excluded:
                continue
            if self.wanted and topic not in self.wanted:
                continue
            entry = self.unwatched.get(topic)
            if entry is not None and entry.gave_up:
                continue
            if len(types) != 1:
                self.fail(topic, f"type ambiguous, {len(types)} types announced")
                continue
            self.qualify(topic, types[0])

    def qualify(self, topic, type_name):
        try:
            message_type = get_message(type_name)
        except (AttributeError, ImportError, ModuleNotFoundError, ValueError):
            self.fail(topic, f"type unresolved ({type_name})")
            return
        if not has_leading_header(message_type):
            self.fail(topic, "no leading header field")
            return
        self.pending(topic, WAITING)
        self.watched[topic] = self.create_subscription(
            message_type, topic, partial(self.on_raw, topic), self.subscription_qos, raw=True
        )

    def pending(self, topic, reason):
        """Introspection is not measurement, the record stands until a sample is accepted."""
        entry = self.unwatched.get(topic)
        if entry is None:
            entry = self.unwatched[topic] = Attempt()
        entry.reason = reason
        return entry

    def fail(self, topic, reason):
        entry = self.pending(topic, reason)
        entry.attempts += 1
        self.get_logger().info(f"{topic} not watched, {entry.describe()}")

    def on_raw(self, topic, raw):
        arrival = self.arrival_now()
        if topic in self.dropping:
            return
        try:
            stamp = parse_cdr_stamp(raw)
        except UnsupportedEncapsulation as exc:
            self.drop(topic, str(exc))
            return
        except (struct.error, IndexError):
            self.drop(topic, "payload shorter than a header stamp")
            return
        window = self.windows.get(topic)
        if window is None:
            if not self.plausible(stamp, arrival):
                self.drop(topic, "leading bytes are not a header stamp")
                return
            window = self.windows[topic] = TopicWindow(self.window_size)
        window.add(stamp, arrival)
        self.unwatched.pop(topic, None)

    def plausible(self, stamp, arrival):
        return stamp == 0.0 or abs(arrival - stamp) <= self.plausible_window_s

    def drop(self, topic, reason):
        self.dropping.add(topic)
        self.fail(topic, reason)

    def prune(self):
        for topic in self.dropping:
            subscription = self.watched.pop(topic, None)
            if subscription is not None:
                self.destroy_subscription(subscription)
            self.windows.pop(topic, None)
        self.dropping.clear()

    def unwatched_entries(self):
        return [(topic, self.unwatched[topic].describe()) for topic in sorted(self.unwatched)]

    def report(self):
        self.prune()
        extra = list(self.time_source_statuses)
        stepped = self.detect_clock_step()
        if stepped is not None:
            extra.append(stepped)
        statuses = []
        stale = 0
        now = self.arrival_now()
        for topic in sorted(self.windows):
            window = self.windows[topic]
            stamps = list(window.stamps)
            stats = window_stats(stamps, window.arrivals)
            age = window.age(now)
            stale += is_stale(stats, age, self.stale_after_s)
            level, message = topic_verdict(
                topic, stats, stamps, age, self.stale_after_s, self.jitter_warn_factor
            )
            statuses.append(build_status(topic, stats, level, message, age))
        entries = self.unwatched_entries()
        if entries:
            extra.append(build_unwatched_status(entries))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses + extra + [build_summary(statuses, extra, len(entries), stale)]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MonitorNode()
    except (ParameterError, ParameterException) as exc:
        print(f"rosio_monitor: {first_line(exc)}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
