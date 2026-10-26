import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from rosio_monitor import time_sources  # noqa: E402
from rosio_monitor.check import (  # noqa: E402
    MAX_DURATION_S,
    diagnose_capture,
    exit_code,
    format_report,
    main,
    merge_sources,
    parse_args,
    poll_sources,
    run_checks,
    to_series,
    validate_duration,
)
from rosio_monitor.monitor_node import MonitorNode, TopicWindow  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from rosio.stamps.model import Finding  # noqa: E402
from rosio.stamps.verdict import NOT_ANALYZED, Verdict  # noqa: E402

TEST_TOPIC = "/rosio_monitor_check_test/imu"
TAI_OFFSET_S = 37.0
TIMEOUT_S = 20.0
WANTED_MESSAGES = 12


def window(stamps, arrivals):
    item = TopicWindow(256)
    for stamp, arrival in zip(stamps, arrivals, strict=True):
        item.add(stamp, arrival)
    return item


def clean_series():
    stamps = [1754800000.0 + i * 0.1 for i in range(20)]
    arrivals = [stamp + 0.01 for stamp in stamps]
    return to_series({"/imu": window(stamps, arrivals)}, {"/imu": "sensor_msgs/msg/Imu"})


def test_parse_args_defaults():
    options = parse_args([])
    assert options.duration is None
    assert options.topics is None


def test_parse_args_reads_duration_and_topics():
    options = parse_args(["--duration", "3.5", "--topics", "/a", "--topics", "/b"])
    assert options.duration == 3.5
    assert options.topics == ["/a", "/b"]


def test_to_series_converts_windows():
    series = to_series(
        {"/b": window([2.0], [2.5]), "/a": window([1.0, 1.1], [1.2, 1.3])},
        {"/a": "sensor_msgs/msg/Imu"},
    )
    assert list(series) == ["/a", "/b"]
    assert series["/a"].stamps == [1.0, 1.1]
    assert series["/a"].receive_times == [1.2, 1.3]
    assert series["/a"].msgtype == "sensor_msgs/msg/Imu"
    assert series["/b"].msgtype == ""


def test_run_checks_finds_a_tai_offset():
    stamps = [1754800000.0 + i * 0.1 for i in range(20)]
    arrivals = [stamp + TAI_OFFSET_S for stamp in stamps]
    series = to_series({"/lidar": window(stamps, arrivals)}, {})
    findings = run_checks(series)
    epoch = [f for f in findings if f.check == "epoch_offset"]
    assert len(epoch) == 1
    assert epoch[0].evidence["class"] == "tai utc offset"
    assert epoch[0].severity == "error"


def test_run_checks_stays_silent_on_a_clean_topic():
    assert [f for f in run_checks(clean_series()) if f.severity != "info"] == []


def test_exit_code_is_zero_when_clean():
    assert exit_code([], [(time_sources.OK, "ptp", "fine", {})]) == 0


def test_exit_code_is_one_on_an_error_finding():
    finding = Finding(check="epoch_offset", topic="/imu", severity="error", message="bad")
    assert exit_code([finding], []) == 1


def test_exit_code_ignores_warnings():
    finding = Finding(check="epoch_offset", topic="/imu", severity="warning", message="odd")
    assert exit_code([finding], [(time_sources.WARN, "chrony", "drifting", {})]) == 0


def test_exit_code_is_one_on_an_error_time_source():
    assert exit_code([], [(time_sources.ERROR, "ptp", "no grandmaster", {})]) == 1


def test_merge_sources_keeps_the_end_poll():
    end = [(time_sources.OK, "ptp", "fine", {})]
    assert merge_sources([(time_sources.OK, "ptp", "fine", {})], end) == end


def test_merge_sources_notes_a_change():
    start = [(time_sources.OK, "ptp", "fine", {})]
    end = [(time_sources.ERROR, "ptp", "no grandmaster", {})]
    merged = merge_sources(start, end)
    assert len(merged) == 2
    assert merged[1][1] == "change"
    assert "OK at the start and ERROR at the end" in merged[1][2]
    assert exit_code([], merged) == 1


def test_format_report_has_every_section():
    finding = Finding(check="epoch_offset", topic="/lidar", severity="error", message="37 s off")
    verdict = Verdict(hypotheses=["the lidar runs on TAI"], level="error")
    report = format_report(
        4.0, clean_series(), [finding], verdict, [(time_sources.OK, "ptp", "slaved", {})]
    )
    lines = report.splitlines()
    assert lines[0] == "capture: 4.0 s, 1 topics, 20 messages"
    assert "ERROR epoch_offset /lidar: 37 s off" in lines
    assert "verdict:" in lines
    assert "  the lidar runs on TAI" in lines
    assert "time sources:" in lines
    assert "  OK ptp: slaved" in lines


def test_diagnose_capture_on_a_clean_series():
    report, code = diagnose_capture(5.0, clean_series(), [(time_sources.OK, "ptp", "slaved", {})])
    assert code == 0
    assert "No timestamp problems found" in report


def test_diagnose_capture_without_topics_is_unknown():
    report, code = diagnose_capture(5.0, {}, [(time_sources.OK, "ptp", "slaved", {})])
    assert code == 2
    assert NOT_ANALYZED in report
    assert "capture: 5.0 s, 0 topics, 0 messages" in report


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


def test_capture_of_a_tai_topic_names_the_class(ros):
    overrides = [
        Parameter("topics", value=[TEST_TOPIC]),
        Parameter("discovery_period_s", value=0.5),
        Parameter("report_period_s", value=0.5),
        Parameter("time_source_period_s", value=0.0),
    ]
    monitor = MonitorNode(parameter_overrides=overrides)
    driver = rclpy.create_node("rosio_monitor_check_test_driver")
    publisher = driver.create_publisher(Imu, TEST_TOPIC, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(driver)

    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            publisher.publish(imu_bytes(time.time() - TAI_OFFSET_S))
            executor.spin_once(timeout_sec=0.1)
            window = monitor.windows.get(TEST_TOPIC)
            if window is not None and len(window.stamps) >= WANTED_MESSAGES:
                break
        series = to_series(monitor.windows, {})
    finally:
        executor.shutdown()
        driver.destroy_node()
        monitor.destroy_node()

    assert TEST_TOPIC in series, f"the monitor never saw {TEST_TOPIC}"
    report, code = diagnose_capture(8.0, series, [(time_sources.OK, "ptp", "slaved", {})])
    assert code == 1
    assert "TAI to UTC offset" in report
    assert "PTP timescale" in report


def test_poll_sources_is_skipped_when_the_period_is_zero(ros):
    node = MonitorNode(parameter_overrides=[Parameter("time_source_period_s", value=0.0)])
    try:
        assert poll_sources(node) == []
    finally:
        node.destroy_node()


def test_main_refuses_a_wrongly_typed_parameter(capsys):
    code = main(["check", "--ros-args", "-p", "plausible_window_s:=not_a_number"])
    error = capsys.readouterr().err.strip()
    assert code == 2
    assert error.startswith("rosio_monitor: ")
    assert len(error.splitlines()) == 1
    assert "plausible_window_s" in error
    assert "capture failed" not in error


def test_validate_duration_accepts_a_normal_capture():
    assert validate_duration(10) == 10.0


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf"), 3600.5, "ten", None, True])
def test_validate_duration_rejects_unusable_values(bad):
    with pytest.raises(ValueError):
        validate_duration(bad)


def test_validate_duration_accepts_the_upper_bound():
    assert validate_duration(MAX_DURATION_S) == MAX_DURATION_S
