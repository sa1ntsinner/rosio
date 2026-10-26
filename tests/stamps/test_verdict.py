import json

from rosio.stamps.cli import main
from rosio.stamps.model import Finding, TopicSeries
from rosio.stamps.verdict import diagnose

TOPICS = ("/scan", "/imu", "/gps")


def series(*topics):
    return {t: TopicSeries(topic=t, msgtype="x", stamps=[0.0], receive_times=[0.0]) for t in topics}


def epoch(topic, offset, name):
    return Finding(
        check="epoch_offset",
        topic=topic,
        severity="error",
        message=f"{topic} offset {offset:.3f} s",
        evidence={"median_offset_s": offset, "class": name},
    )


def gap(topic, moment, size=4.0):
    return Finding(
        check="stamp_gaps",
        topic=topic,
        severity="warning",
        message=f"{topic} has stamp gaps, largest {size:.3f} s against a median 0.100 s",
        evidence={"median_dt_s": 0.1, "max_gap_s": size, "worst_gap_receive_time": moment},
    )


def test_global_timescale():
    findings = [epoch(t, 37.0, "tai utc offset") for t in TOPICS]
    verdict = diagnose(series(*TOPICS), findings)
    assert verdict.level == "error"
    assert len(verdict.hypotheses) == 1
    assert "whole system shares one clock problem" in verdict.hypotheses[0]
    assert "tai utc offset" in verdict.hypotheses[0]
    assert "37.000 s" in verdict.hypotheses[0]
    assert "recorder host clock" in verdict.hypotheses[0]


def test_global_timescale_needs_one_class():
    findings = [
        epoch("/scan", 37.0, "tai utc offset"),
        epoch("/imu", 37.0, "tai utc offset"),
        epoch("/gps", 18.0, "gps utc leap offset"),
    ]
    hypotheses = diagnose(series(*TOPICS), findings).hypotheses
    assert len(hypotheses) == 2
    assert all("whole system" not in h for h in hypotheses)


def test_single_source_epoch():
    verdict = diagnose(series(*TOPICS), [epoch("/gps", 18.0, "gps utc leap offset")])
    assert len(verdict.hypotheses) == 1
    assert verdict.hypotheses[0].startswith("/gps carries a gps utc leap offset")
    assert "the rest of the system is clean" in verdict.hypotheses[0]


def test_single_source_tai_points_at_ptp():
    verdict = diagnose(series(*TOPICS), [epoch("/scan", 37.0, "tai utc offset")])
    assert "ptp_utc_tai_offset" in verdict.hypotheses[0]


def test_global_timescale_outranks_single_source():
    findings = [epoch(t, 37.0, "tai utc offset") for t in TOPICS]
    findings.append(gap("/scan", 100.0))
    hypotheses = diagnose(series(*TOPICS), findings).hypotheses
    assert "whole system shares one clock problem" in hypotheses[0]
    assert len(hypotheses) == 2


def test_drift():
    finding = Finding(
        check="clock_drift",
        topic="/imu",
        severity="warning",
        message="/imu drifts",
        evidence={"slope_s_per_s": 0.001},
    )
    verdict = diagnose(series("/imu"), [finding])
    assert verdict.level == "warning"
    assert "/imu drifts against the receiving clock" in verdict.hypotheses[0]
    assert "1.000e-03 s per s" in verdict.hypotheses[0]
    assert "0.600 s of error after ten minutes" in verdict.hypotheses[0]


def test_correlated_gaps():
    findings = [gap("/scan", 100.0), gap("/imu", 100.7), gap("/gps", 101.5)]
    hypotheses = diagnose(series(*TOPICS), findings).hypotheses
    assert len(hypotheses) == 1
    assert "the recorder or the host stalled around t=100.700" in hypotheses[0]
    assert "/gps, /imu and /scan" in hypotheses[0]


def test_isolated_gaps_stay_separate():
    findings = [gap("/scan", 100.0), gap("/imu", 140.0), gap("/gps", 180.0)]
    hypotheses = diagnose(series(*TOPICS), findings).hypotheses
    assert len(hypotheses) == 3
    assert all("stalled" not in h for h in hypotheses)
    assert hypotheses[0].endswith("look at that source alone.")


def test_correlated_gaps_keep_the_outlier():
    findings = [gap("/scan", 100.0), gap("/imu", 100.5), gap("/gps", 101.0), gap("/odom", 400.0)]
    hypotheses = diagnose(series(*TOPICS, "/odom"), findings).hypotheses
    assert len(hypotheses) == 2
    assert "stalled" in hypotheses[0]
    assert hypotheses[1].startswith("/odom has stamp gaps")


def test_condensed_findings():
    findings = [
        Finding("frozen_stamps", "/scan", "error", "/scan stamp frozen at 5.000 s", {}),
        Finding("zero_stamps", "/imu", "error", "/imu never sets the stamp", {}),
        Finding("monotonic", "/gps", "error", "/gps stamps go backwards", {}),
        Finding("inter_topic_skew", "/odom", "warning", "/odom offset deviates 0.400 s", {}),
    ]
    hypotheses = diagnose(series(*TOPICS, "/odom"), findings).hypotheses
    assert len(hypotheses) == 4
    assert "the driver stopped refreshing the header stamp" in hypotheses[0]
    assert "leaves the header stamp unset" in hypotheses[1]
    assert "stepped backwards" in hypotheses[2]
    assert "offset from the rest of the system" in hypotheses[3]


def test_clean():
    verdict = diagnose(series(*TOPICS), [])
    assert verdict.level == "ok"
    assert verdict.hypotheses == ["No timestamp problems found across 3 topics."]


def test_nothing_analyzed_is_unknown():
    verdict = diagnose({}, [])
    assert verdict.level == "unknown"
    assert verdict.hypotheses == ["no topics with headers were found, nothing was analyzed"]


def test_findings_without_series_still_diagnose():
    verdict = diagnose({}, [epoch("/gps", 18.0, "gps utc leap offset")])
    assert verdict.level == "error"
    assert verdict.hypotheses[0].startswith("/gps carries a gps utc leap offset")


def test_info_only_is_clean():
    finding = Finding("stamp_rate", "/scan", "info", "/scan rate 10.00 Hz", {})
    verdict = diagnose(series(*TOPICS), [finding])
    assert verdict.level == "info"
    assert verdict.hypotheses == ["No timestamp problems found across 3 topics."]


def test_cli_prints_verdict(capsys, backward_bag):
    assert main([str(backward_bag)]) == 1
    lines = capsys.readouterr().out.splitlines()
    assert lines[-2] == "verdict:"
    assert lines[-3] == ""
    assert lines[-1].startswith("  /scan stamps go backwards")


def test_cli_json_verdict(capsys, backward_bag):
    assert main([str(backward_bag), "--json-verdict"]) == 1
    payload = json.loads(capsys.readouterr().out)
    assert [f["check"] for f in payload["findings"]] == ["monotonic"]
    assert payload["verdict"]["level"] == "error"
    assert len(payload["verdict"]["hypotheses"]) == 1


def test_cli_json_shape_unchanged(capsys, backward_bag):
    assert main([str(backward_bag), "--json"]) == 1
    assert isinstance(json.loads(capsys.readouterr().out), list)
