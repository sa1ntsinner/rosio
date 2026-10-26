import pytest

from rosio.stamps.checks import CHECKS
from rosio.stamps.model import TopicSeries
from rosio.stamps.stream import check_monotonic


def series(topic, stamps):
    return {topic: TopicSeries(topic, "sensor_msgs/msg/LaserScan", stamps, list(stamps))}


def test_monotonic_clean():
    assert check_monotonic(series("/scan", [1.0, 1.1, 1.2])) == []


def test_monotonic_backward_jump():
    findings = check_monotonic(series("/scan", [1.0, 1.1, 0.6, 1.3]))
    assert len(findings) == 1
    finding = findings[0]
    assert finding.topic == "/scan"
    assert finding.severity == "error"
    assert "/scan" in finding.message
    assert finding.evidence["largest_backward_jump"] == pytest.approx(0.5)
    assert finding.evidence["index"] == 2


def test_monotonic_registered():
    assert CHECKS["monotonic"] is check_monotonic
