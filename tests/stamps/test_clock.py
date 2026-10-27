import pytest

from rosio.stamps.checks import CHECKS
from rosio.stamps.clock import check_clock_drift, check_epoch_offset, check_inter_topic_skew
from rosio.stamps.model import TopicSeries

BASE = 1785000000.0
LATENCY = 0.01


def make(topic, offset, count=5, start=BASE):
    stamps = [start + i * 0.1 for i in range(count)]
    receive_times = [s + offset for s in stamps]
    return TopicSeries(topic, "sensor_msgs/msg/LaserScan", stamps, receive_times)


def bag(*entries):
    return {entry.topic: entry for entry in entries}


def test_clean_bag_has_no_findings():
    series = bag(make("/scan", LATENCY), make("/imu", 0.02))
    assert check_epoch_offset(series) == []


@pytest.mark.parametrize("offset", [18.0, -18.0])
def test_gps_utc_leap_offset(offset):
    findings = check_epoch_offset(bag(make("/gnss/fix", offset)))
    assert len(findings) == 1
    finding = findings[0]
    assert finding.severity == "error"
    assert finding.evidence["class"] == "gps utc leap offset"
    assert finding.evidence["median_offset_s"] == pytest.approx(offset)
    assert finding.evidence["sample_stamps"] == [BASE, BASE + 0.1, BASE + 0.2]


@pytest.mark.parametrize("offset", [37.0, -37.0])
def test_tai_utc_offset(offset):
    findings = check_epoch_offset(bag(make("/lidar/points", offset)))
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].evidence["class"] == "tai utc offset"
    assert findings[0].evidence["median_offset_s"] == pytest.approx(offset)


def test_offset_direction_is_reported():
    ahead = check_epoch_offset(bag(make("/lidar/points", -37.0)))[0]
    behind = check_epoch_offset(bag(make("/lidar/points", 37.0)))[0]
    assert ahead.message.endswith("stamps ahead of the receiver")
    assert behind.message.endswith("stamps behind the receiver")
    # the number carries the sign the atlas states, receive minus stamp
    assert "offset -37.000 s" in ahead.message
    assert "offset 37.000 s" in behind.message


def test_unsynced_epoch():
    entry = TopicSeries(
        "/camera", "sensor_msgs/msg/Image", [0.5, 0.6, 0.7], [BASE, BASE + 0.1, BASE + 0.2]
    )
    findings = check_epoch_offset(bag(entry))
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].evidence["class"] == "unsynced epoch"


def test_shifted_epoch_without_offset():
    shifted = make("/camera", 0.0, start=10000.0)
    series = bag(shifted, make("/imu", LATENCY))
    findings = check_epoch_offset(series)
    assert [f.topic for f in findings] == ["/camera"]
    finding = findings[0]
    assert finding.severity == "error"
    assert finding.evidence["class"] == "unsynced epoch"
    assert finding.evidence["median_offset_s"] == pytest.approx(0.0)
    assert finding.evidence["first_stamp_s"] == pytest.approx(10000.0)
    assert "1970" in finding.message and "2005" in finding.message


def test_all_zero_stamps_stay_silent():
    entry = TopicSeries("/camera", "sensor_msgs/msg/Image", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert check_epoch_offset(bag(entry)) == []


def test_zero_stamps_before_a_shifted_one_still_fire():
    entry = TopicSeries(
        "/camera", "sensor_msgs/msg/Image", [0.0, 10000.0, 10000.1], [0.0, 10000.0, 10000.1]
    )
    findings = check_epoch_offset(bag(entry))
    assert [f.evidence["first_stamp_s"] for f in findings] == [pytest.approx(10000.0)]


def test_future_stamps():
    findings = check_epoch_offset(bag(make("/scan", -5.0)))
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].evidence["class"] == "stamps from the future"


def test_unclassified_offset_is_warning():
    findings = check_epoch_offset(bag(make("/scan", 3.0)))
    assert len(findings) == 1
    assert findings[0].severity == "warning"
    assert findings[0].evidence["class"] == "unclassified offset"


def test_one_finding_per_offending_topic():
    series = bag(make("/scan", LATENCY), make("/gnss/fix", 18.0), make("/points", 37.0))
    findings = check_epoch_offset(series)
    assert [f.topic for f in findings] == ["/gnss/fix", "/points"]


def test_skew_clean_topics():
    series = bag(
        make("/a", LATENCY),
        make("/b", 0.02),
        make("/c", 0.03),
        make("/d", 0.04),
    )
    assert check_inter_topic_skew(series) == []


def test_skew_outlier_among_four_topics():
    series = bag(
        make("/a", LATENCY),
        make("/b", 0.02),
        make("/c", 0.03),
        make("/d", 0.25),
    )
    findings = check_inter_topic_skew(series)
    assert len(findings) == 1
    finding = findings[0]
    assert finding.topic == "/d"
    assert finding.severity == "warning"
    assert finding.evidence["reference_offset_s"] == pytest.approx(0.025)
    assert finding.evidence["delta_s"] == pytest.approx(0.225)


def test_skew_large_outlier_is_error():
    series = bag(make("/a", LATENCY), make("/b", 0.02), make("/c", 0.03), make("/d", 1.0))
    findings = check_inter_topic_skew(series)
    assert [f.severity for f in findings] == ["error"]
    assert findings[0].evidence["topic_offset_s"] == pytest.approx(1.0)


def test_skew_needs_three_topics():
    series = bag(make("/a", LATENCY), make("/b", 5.0))
    assert check_inter_topic_skew(series) == []


def drifting(topic, rate, count=600, period=0.1):
    receive_times = [BASE + i * period for i in range(count)]
    stamps = [t - LATENCY - rate * (t - BASE) for t in receive_times]
    return TopicSeries(topic, "sensor_msgs/msg/Imu", stamps, receive_times)


def test_drift_free_topic_stays_silent():
    assert check_clock_drift(bag(drifting("/imu", 0.0))) == []


def test_linear_drift_is_error():
    findings = check_clock_drift(bag(drifting("/imu", 0.005)))
    assert len(findings) == 1
    finding = findings[0]
    assert finding.check == "clock_drift"
    assert finding.severity == "error"
    assert finding.evidence["slope_s_per_s"] == pytest.approx(0.005)
    assert finding.evidence["span_s"] == pytest.approx(59.9)
    assert finding.evidence["messages"] == 600
    assert finding.evidence["mean_abs_residual_s"] == pytest.approx(0.0, abs=1e-6)
    assert "5.000 ms/s" in finding.message


def test_slow_drift_is_warning():
    findings = check_clock_drift(bag(drifting("/imu", 0.0003)))
    assert [f.severity for f in findings] == ["warning"]


def test_step_offset_stays_silent():
    receive_times = [BASE + i * 0.1 for i in range(300)]
    receive_times += [t + 5.0 for t in [BASE + 30.0 + i * 0.1 for i in range(300)]]
    stamps = [t - LATENCY for t in receive_times[:300]]
    stamps += [t - LATENCY - 0.5 for t in receive_times[300:]]
    entry = TopicSeries("/imu", "sensor_msgs/msg/Imu", stamps, receive_times)
    assert check_clock_drift(bag(entry)) == []


def test_short_topic_is_skipped():
    assert check_clock_drift(bag(drifting("/imu", 0.005, count=20))) == []


def test_brief_topic_is_skipped():
    assert check_clock_drift(bag(drifting("/imu", 0.005, count=100, period=0.01))) == []


def test_clock_checks_registered():
    assert CHECKS["epoch_offset"] is check_epoch_offset
    assert CHECKS["inter_topic_skew"] is check_inter_topic_skew
    assert CHECKS["clock_drift"] is check_clock_drift
