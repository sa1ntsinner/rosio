import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from rosio_monitor.monitor_node import (  # noqa: E402
    MAX_ATTEMPTS,
    UNREAD_SOURCES,
    WAITING,
    MonitorNode,
    ParameterError,
    TopicWindow,
    main,
)
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import String  # noqa: E402

TEST_TOPIC = "/rosio_monitor_test/imu"
TIMEOUT_S = 20.0


@pytest.fixture
def ros():
    try:
        rclpy.init()
    except Exception as exc:
        pytest.skip(f"rclpy could not start: {exc}")
    yield
    if rclpy.ok():
        rclpy.shutdown()


def imu_bytes(stamp_s):
    message = Imu()
    message.header.frame_id = "imu_link"
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    return serialize_message(message)


def test_monitor_reports_a_live_topic(ros):
    overrides = [
        Parameter("topics", value=[TEST_TOPIC]),
        Parameter("discovery_period_s", value=0.5),
        Parameter("report_period_s", value=0.2),
    ]
    monitor = MonitorNode(parameter_overrides=overrides)
    driver = rclpy.create_node("rosio_monitor_test_driver")
    publisher = driver.create_publisher(Imu, TEST_TOPIC, 10)
    seen = []
    driver.create_subscription(DiagnosticArray, "/diagnostics", seen.append, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(driver)

    deadline = time.monotonic() + TIMEOUT_S
    reports = []
    try:
        while time.monotonic() < deadline and not reports:
            publisher.publish(imu_bytes(time.time()))
            executor.spin_once(timeout_sec=0.1)
            reports = [
                array for array in seen if any(TEST_TOPIC in status.name for status in array.status)
            ]
    finally:
        executor.shutdown()
        driver.destroy_node()
        monitor.destroy_node()

    assert reports, f"no diagnostics carrying {TEST_TOPIC} arrived"
    array = reports[-1]
    expected = f"rosio_monitor: {TEST_TOPIC}"
    status = next(item for item in array.status if item.name == expected)
    values = {item.key: item.value for item in status.values}
    assert status.level == DiagnosticStatus.OK
    assert int(values["messages"]) >= 1
    assert abs(float(values["median_offset_s"])) < 1.0
    assert values["frozen"] == "false"
    assert values["zero_stamps"] == "0"
    summary = next(item for item in array.status if item.name == "rosio_monitor: summary")
    assert "topics watched" in summary.message


HEADERLESS_TOPIC = "/rosio_monitor_test/chatter"


class FakeClock:
    def __init__(self, wall=1754800000.0, mono=1000.0):
        self.wall = wall
        self.mono = mono

    def time(self):
        return self.wall

    def monotonic(self):
        return self.mono


def monitor(**overrides):
    values = {"time_source_period_s": 0.0, "discovery_period_s": 0.5, "report_period_s": 0.2}
    values.update(overrides)
    clocks = {key: values.pop(key) for key in ("wall_clock", "monotonic_clock") if key in values}
    parameters = [Parameter(key, value=value) for key, value in values.items()]
    return MonitorNode(parameter_overrides=parameters, **clocks)


def test_headerless_topic_is_listed_as_unwatched(ros):
    node = monitor(topics=[HEADERLESS_TOPIC])
    driver = rclpy.create_node("rosio_monitor_headerless_driver")
    driver.create_publisher(String, HEADERLESS_TOPIC, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    try:
        deadline = time.monotonic() + TIMEOUT_S
        while time.monotonic() < deadline and HEADERLESS_TOPIC not in node.unwatched:
            executor.spin_once(timeout_sec=0.1)
        node.report()
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()

    assert HEADERLESS_TOPIC in node.unwatched
    assert node.unwatched[HEADERLESS_TOPIC].reason == "no leading header field"
    array = published[-1]
    status = next(item for item in array.status if item.name == "rosio_monitor: unwatched")
    assert status.level == DiagnosticStatus.WARN
    assert HEADERLESS_TOPIC in status.message
    summary = next(item for item in array.status if item.name == "rosio_monitor: summary")
    assert summary.level == DiagnosticStatus.WARN
    assert "not watched" in summary.message


def test_qualification_is_retried_then_parked(ros):
    node = monitor(topics=["/ghost"])
    node.get_topic_names_and_types = lambda: [("/ghost", ["no_such_pkg/msg/Nope"])]
    try:
        for _ in range(MAX_ATTEMPTS + 3):
            node.discover()
        entry = node.unwatched["/ghost"]
        assert entry.attempts == MAX_ATTEMPTS
        assert entry.gave_up is True
        assert node.unwatched_entries() == [
            ("/ghost", "type unresolved (no_such_pkg/msg/Nope), gave up")
        ]
    finally:
        node.destroy_node()


def test_raw_rejection_spends_the_same_attempt_budget(ros):
    """A topic that introspects fine but never delivers a usable stamp still parks."""
    node = monitor(topics=["/implausible"], plausible_window_s=1.0)
    node.get_topic_names_and_types = lambda: [("/implausible", ["sensor_msgs/msg/Imu"])]
    node.publisher = SimpleNamespace(publish=lambda array: None)
    try:
        for _ in range(MAX_ATTEMPTS + 3):
            node.discover()
            if "/implausible" in node.watched:
                node.on_raw("/implausible", imu_bytes(1.0))
            node.report()
        entry = node.unwatched["/implausible"]
        assert entry.attempts == MAX_ATTEMPTS
        assert entry.gave_up is True
        assert node.unwatched_entries() == [
            ("/implausible", "leading bytes are not a header stamp, gave up")
        ]
    finally:
        node.destroy_node()


def test_a_qualified_topic_stays_listed_until_its_first_sample(ros):
    node = monitor(topics=[TEST_TOPIC], plausible_window_s=1000000.0)
    node.get_topic_names_and_types = lambda: [(TEST_TOPIC, ["sensor_msgs/msg/Imu"])]
    try:
        node.discover()
        assert node.unwatched[TEST_TOPIC].reason == WAITING
        assert node.unwatched[TEST_TOPIC].attempts == 0
        node.on_raw(TEST_TOPIC, imu_bytes(time.time()))
        assert TEST_TOPIC not in node.unwatched
    finally:
        node.destroy_node()


INFRASTRUCTURE = [
    ("/rosout", ["rcl_interfaces/msg/Log"]),
    ("/parameter_events", ["rcl_interfaces/msg/ParameterEvent"]),
]


def test_infrastructure_topics_are_never_discovered(ros):
    node = monitor()
    node.get_topic_names_and_types = lambda: list(INFRASTRUCTURE)
    try:
        node.discover()
        assert node.unwatched == {}
        assert node.watched == {}
    finally:
        node.destroy_node()


def test_an_excluded_topic_that_is_requested_is_still_watched(ros):
    node = monitor(topics=["/rosout"])
    node.get_topic_names_and_types = lambda: list(INFRASTRUCTURE)
    try:
        node.discover()
        assert list(node.unwatched) == ["/rosout"]
        assert node.unwatched["/rosout"].reason == "no leading header field"
    finally:
        node.destroy_node()


def test_clock_step_resets_the_windows_and_reanchors(ros):
    clock = FakeClock()
    node = monitor(wall_clock=clock.time, monotonic_clock=clock.monotonic)
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    try:
        node.windows["/imu"] = TopicWindow(8)
        node.windows["/imu"].add(clock.wall, clock.wall)
        clock.mono += 1.0
        clock.wall += 1.0
        assert node.detect_clock_step() is None

        clock.wall += 7.5
        node.report()
        assert node.windows == {}
        status = next(
            item for item in published[-1].status if item.name == "rosio_monitor: host clock"
        )
        assert status.level == DiagnosticStatus.WARN
        assert status.message == "host clock stepped by 7.500 s during monitoring"
        assert node.detect_clock_step() is None
        assert node.arrival_now() == pytest.approx(clock.wall)
    finally:
        node.destroy_node()


def test_bad_parameter_is_refused_at_startup(ros):
    with pytest.raises(ParameterError) as caught:
        monitor(window_size=0)
    assert "window_size" in str(caught.value)


def test_bad_period_is_refused_at_startup(ros):
    with pytest.raises(ParameterError):
        monitor(report_period_s=-1.0)


def test_main_refuses_a_wrongly_typed_parameter(capsys):
    code = main(["monitor", "--ros-args", "-p", "plausible_window_s:=not_a_number"])
    error = capsys.readouterr().err.strip()
    assert code == 2
    assert error.startswith("rosio_monitor: ")
    assert len(error.splitlines()) == 1
    assert "plausible_window_s" in error


def fill(window, wall, count=10, interval=0.1, offset=0.01):
    for i in range(count):
        stamp = wall - (count - 1 - i) * interval
        window.add(stamp, stamp + offset)


def advance(clock, seconds):
    clock.wall += seconds
    clock.mono += seconds


def stale_setup():
    clock = FakeClock()
    node = monitor(wall_clock=clock.time, monotonic_clock=clock.monotonic)
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    window = node.windows["/odom"] = TopicWindow(64)
    fill(window, clock.wall)
    return clock, node, window, published


def odom_status(published):
    return next(item for item in published[-1].status if item.name == "rosio_monitor: /odom")


def test_report_warns_when_a_topic_stops_publishing(ros):
    clock, node, _, published = stale_setup()
    try:
        node.report()
        assert odom_status(published).level == DiagnosticStatus.OK
        advance(clock, 3.0)
        node.report()
        status = odom_status(published)
        values = {item.key: item.value for item in status.values}
        assert status.level == DiagnosticStatus.WARN
        assert "has not published for 3.0 s" in status.message
        assert values["window_age_s"] == "2.990"
        assert values["messages"] == "10"
        summary = next(
            item for item in published[-1].status if item.name == "rosio_monitor: summary"
        )
        assert "1 stale" in summary.message
    finally:
        node.destroy_node()


def test_report_errors_when_the_silence_grows(ros):
    clock, node, _, published = stale_setup()
    try:
        advance(clock, 30.0)
        node.report()
        status = odom_status(published)
        assert status.level == DiagnosticStatus.ERROR
        assert "has not published for 30.0 s" in status.message
    finally:
        node.destroy_node()


def test_a_recovered_topic_reports_ok_again(ros):
    clock, node, window, published = stale_setup()
    try:
        advance(clock, 30.0)
        node.report()
        assert odom_status(published).level == DiagnosticStatus.ERROR
        fill(window, clock.wall)
        node.report()
        status = odom_status(published)
        assert status.level == DiagnosticStatus.OK
        assert float({item.key: item.value for item in status.values}["window_age_s"]) < 0.1
    finally:
        node.destroy_node()


SLOW_POLL_S = 1.0
CADENCE_REPORT_PERIOD_S = 0.1
CADENCE_RUN_S = 2.0


class SlowPollMonitor(MonitorNode):
    """A host whose time source tools all hang, the shape finding A describes."""

    polls = 0

    def poll_time_sources(self):
        SlowPollMonitor.polls += 1
        time.sleep(SLOW_POLL_S)
        return []


def test_a_slow_time_source_poll_does_not_stall_the_reports(ros):
    SlowPollMonitor.polls = 0
    node = SlowPollMonitor(
        parameter_overrides=[
            Parameter("topics", value=["/nothing_here"]),
            Parameter("discovery_period_s", value=5.0),
            Parameter("report_period_s", value=CADENCE_REPORT_PERIOD_S),
            Parameter("time_source_period_s", value=0.2),
        ]
    )
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        deadline = time.monotonic() + CADENCE_RUN_S
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
    finally:
        executor.shutdown()
        node.destroy_node()

    expected = CADENCE_RUN_S / CADENCE_REPORT_PERIOD_S
    assert len(published) >= expected * 0.8, f"reports fell behind: {len(published)}"
    assert SlowPollMonitor.polls >= 1


def test_time_sources_report_as_unread_until_the_first_poll_lands(ros):
    node = SlowPollMonitor(
        parameter_overrides=[
            Parameter("topics", value=["/nothing_here"]),
            Parameter("report_period_s", value=CADENCE_REPORT_PERIOD_S),
            Parameter("time_source_period_s", value=10.0),
        ]
    )
    try:
        status = node.time_source_statuses[0]
        assert status.name == "rosio_monitor: time sources/poll"
        assert status.message == UNREAD_SOURCES
        assert status.level == bytes((1,))
    finally:
        node.destroy_node()


def test_the_poller_thread_stops_with_the_node(ros):
    node = SlowPollMonitor(
        parameter_overrides=[
            Parameter("topics", value=["/nothing_here"]),
            Parameter("time_source_period_s", value=0.2),
        ]
    )
    poller = node.source_poller
    assert poller.is_alive()
    node.destroy_node()
    assert node.source_poller is None
    assert poller.stopping.is_set()


def test_no_poller_thread_when_the_poll_is_disabled(ros):
    node = monitor()
    try:
        assert node.source_poller is None
        assert node.time_source_statuses == []
    finally:
        node.destroy_node()


def test_stale_after_s_overrides_the_automatic_budget(ros):
    clock = FakeClock()
    node = monitor(wall_clock=clock.time, monotonic_clock=clock.monotonic, stale_after_s=10.0)
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    fill(node.windows.setdefault("/odom", TopicWindow(64)), clock.wall)
    try:
        advance(clock, 5.0)
        node.report()
        assert odom_status(published).level == DiagnosticStatus.OK
        advance(clock, 8.0)
        node.report()
        assert odom_status(published).level == DiagnosticStatus.WARN
    finally:
        node.destroy_node()
