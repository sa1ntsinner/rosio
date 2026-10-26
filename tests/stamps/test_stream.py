import pytest

from rosio.stamps.checks import CHECKS
from rosio.stamps.model import TopicSeries
from rosio.stamps.stream import (
    check_frozen_stamps,
    check_stamp_gaps,
    check_stamp_rate,
    check_zero_stamps,
)


def series(topic, stamps, receive_times=None):
    if receive_times is None:
        receive_times = list(stamps)
    return {
        topic: TopicSeries(topic, "sensor_msgs/msg/LaserScan", list(stamps), list(receive_times))
    }


def steady(count, dt=0.1, start=100.0):
    return [start + i * dt for i in range(count)]


def test_clean_stream_only_reports_rate():
    data = series("/scan", steady(50))
    assert check_frozen_stamps(data) == []
    assert check_zero_stamps(data) == []
    assert check_stamp_gaps(data) == []
    findings = check_stamp_rate(data)
    assert len(findings) == 1
    assert findings[0].severity == "info"
    assert findings[0].evidence["rate_hz"] == pytest.approx(10.0)
    assert findings[0].evidence["p95_jitter_s"] == pytest.approx(0.0, abs=1e-9)


def test_frozen_run_flagged():
    stamps = steady(10) + [100.95] * 12 + steady(10, start=102.2)
    receive = steady(len(stamps), start=100.0)
    findings = check_frozen_stamps(series("/scan", stamps, receive))
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].evidence["run_length"] == 12
    assert findings[0].evidence["stamp"] == pytest.approx(100.95)
    assert findings[0].evidence["receive_span_s"] == pytest.approx(1.1)


def test_frozen_run_ignored_when_receive_span_short():
    stamps = [100.0] * 12
    receive = [200.0 + i * 0.01 for i in range(12)]
    assert check_frozen_stamps(series("/scan", stamps, receive)) == []


def test_frozen_skips_short_topic():
    stamps = [100.0] * 9
    receive = [200.0 + i for i in range(9)]
    assert check_frozen_stamps(series("/scan", stamps, receive)) == []


def test_zero_stamps_mixed():
    stamps = [0.0, 0.0, 100.2, 100.3]
    receive = [200.0, 200.1, 200.2, 200.3]
    findings = check_zero_stamps(series("/scan", stamps, receive))
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].evidence["count"] == 2
    assert findings[0].evidence["all_zero"] is False
    assert findings[0].evidence["first_receive_time"] == pytest.approx(200.0)


def test_zero_stamps_all():
    stamps = [0.0] * 5
    receive = [200.0 + i * 0.1 for i in range(5)]
    findings = check_zero_stamps(series("/scan", stamps, receive))
    assert findings[0].evidence["all_zero"] is True
    assert "never sets the stamp" in findings[0].message


def test_zero_stamps_clean():
    assert check_zero_stamps(series("/scan", steady(20))) == []


def test_dropout_flagged_as_gaps():
    stamps = steady(20) + steady(20, start=104.0)
    data = series("/scan", stamps)
    gaps = check_stamp_gaps(data)
    assert len(gaps) == 1
    assert gaps[0].severity == "warning"
    assert "kind" not in gaps[0].evidence
    assert gaps[0].evidence["max_gap_s"] == pytest.approx(2.1)
    assert gaps[0].evidence["gap_count"] == 1
    assert gaps[0].evidence["median_dt_s"] == pytest.approx(0.1)
    assert gaps[0].evidence["worst_gap_receive_time"] == pytest.approx(104.0)
    assert len(check_stamp_rate(data)) == 1


def test_gaps_and_rate_skip_short_topic():
    data = series("/scan", steady(19))
    assert check_stamp_gaps(data) == []
    assert check_stamp_rate(data) == []


def test_gaps_and_rate_skip_non_advancing_stamps():
    data = series("/scan", [100.0] * 30)
    assert check_stamp_gaps(data) == []
    assert check_stamp_rate(data) == []


def test_stream_checks_registered():
    assert CHECKS["frozen_stamps"] is check_frozen_stamps
    assert CHECKS["zero_stamps"] is check_zero_stamps
    assert CHECKS["stamp_gaps"] is check_stamp_gaps
    assert CHECKS["stamp_rate"] is check_stamp_rate
