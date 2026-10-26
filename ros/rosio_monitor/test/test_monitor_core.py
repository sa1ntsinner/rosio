import struct

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from rosio_monitor.monitor_node import (
    JITTER_FLOOR_S,
    STALE_CEILING_S,
    STALE_ERROR_FACTOR,
    STALE_FLOOR_S,
    STALE_INTERVAL_FACTOR,
    Attempt,
    ParameterError,
    build_clock_step_status,
    build_status,
    build_summary,
    build_unwatched_status,
    clock_step,
    has_leading_header,
    is_frozen,
    level_for_offset,
    parse_cdr_stamp,
    percentile,
    stale_limit,
    topic_verdict,
    validate_parameters,
    window_stats,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from rosio.stamps.fallback import UnsupportedEncapsulation


def cdr(sec, nanosec, little_endian=True):
    prefix = b"\x00\x01" if little_endian else b"\x00\x00"
    layout = "<iI" if little_endian else ">iI"
    return prefix + b"\x00\x00" + struct.pack(layout, sec, nanosec) + b"payload"


def test_parse_cdr_stamp_little_endian():
    assert parse_cdr_stamp(cdr(1754800000, 500000000)) == pytest.approx(1754800000.5)


def test_parse_cdr_stamp_big_endian():
    assert parse_cdr_stamp(cdr(1754800000, 250000000, False)) == pytest.approx(1754800000.25)


def test_parse_cdr_stamp_short_payload_raises():
    with pytest.raises(struct.error):
        parse_cdr_stamp(b"\x00\x01\x00\x00\x01\x02")


def test_percentile_picks_upper_tail():
    assert percentile([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0], 0.95) == 9.0
    assert percentile([], 0.95) == 0.0


def test_window_stats_on_a_clean_stream():
    stamps = [100.0 + i * 0.1 for i in range(10)]
    arrivals = [stamp + 0.01 for stamp in stamps]
    stats = window_stats(stamps, arrivals)
    assert stats["messages"] == 10
    assert stats["median_offset_s"] == pytest.approx(0.01)
    assert stats["rate_hz"] == pytest.approx(10.0)
    assert stats["p95_jitter_s"] == pytest.approx(0.0, abs=1e-9)
    assert stats["zero_stamps"] == 0
    assert stats["frozen"] is False


def test_window_stats_reports_jitter():
    stamps = [100.0 + i * 0.1 for i in range(10)]
    arrivals = [stamp + 0.01 for stamp in stamps]
    arrivals[-1] += 0.2
    stats = window_stats(stamps, arrivals)
    assert stats["p95_jitter_s"] == pytest.approx(0.2)


def test_window_stats_counts_zero_stamps():
    stats = window_stats([0.0, 0.0, 0.0], [10.0, 10.1, 10.2])
    assert stats["zero_stamps"] == 3
    assert stats["rate_hz"] == 0.0


def test_is_frozen_needs_advancing_arrivals():
    assert is_frozen([5.0, 5.0, 5.0], [10.0, 10.1, 10.2]) is True
    assert is_frozen([5.0, 5.0], [10.0, 10.0]) is False
    assert is_frozen([5.0], [10.0]) is False
    assert is_frozen([5.0, 5.1], [10.0, 10.1]) is False


def test_level_for_offset_ok_within_latency():
    level, message = level_for_offset("/imu", 0.02, [1754800000.0])
    assert level == DiagnosticStatus.OK
    assert "plausible latency" in message


def test_level_for_offset_warns_on_unclassified():
    level, message = level_for_offset("/imu", 1.5, [1754800000.0])
    assert level == DiagnosticStatus.WARN
    assert "beyond the plausible latency" in message


def test_level_for_offset_errors_on_leap_offset():
    level, message = level_for_offset("/gnss", 18.0, [1754800000.0])
    assert level == DiagnosticStatus.ERROR
    assert "GPS" in message


def test_level_for_offset_errors_on_tai_offset():
    level, _ = level_for_offset("/lidar", 37.0, [1754800000.0])
    assert level == DiagnosticStatus.ERROR


def test_level_for_offset_errors_on_unsynced_epoch():
    level, message = level_for_offset("/camera", 1754800000.0, [12.0])
    assert level == DiagnosticStatus.ERROR
    assert "never set" in message


def test_level_for_offset_errors_on_future_stamps():
    level, message = level_for_offset("/imu", -3.0, [1754800000.0])
    assert level == DiagnosticStatus.ERROR
    assert "ahead" in message


def test_topic_verdict_flags_zero_stamps():
    stamps = [0.0, 0.0, 0.0]
    stats = window_stats(stamps, [10.0, 10.1, 10.2])
    level, message = topic_verdict("/imu", stats, stamps)
    assert level == DiagnosticStatus.ERROR
    assert "all zero" in message


def test_topic_verdict_flags_frozen_stamps():
    stamps = [1754800000.0] * 4
    arrivals = [1754800000.0 + i * 0.1 for i in range(4)]
    stats = window_stats(stamps, arrivals)
    level, message = topic_verdict("/imu", stats, stamps)
    assert level == DiagnosticStatus.ERROR
    assert "is frozen at" in message


def test_topic_verdict_ok_on_healthy_window():
    stamps = [1754800000.0 + i * 0.1 for i in range(4)]
    arrivals = [stamp + 0.005 for stamp in stamps]
    stats = window_stats(stamps, arrivals)
    level, _ = topic_verdict("/imu", stats, stamps)
    assert level == DiagnosticStatus.OK


def test_build_summary_counts_findings():
    ok = DiagnosticStatus(level=DiagnosticStatus.OK, name="rosio_monitor: /a")
    bad = DiagnosticStatus(level=DiagnosticStatus.ERROR, name="rosio_monitor: /b")
    summary = build_summary([ok, bad])
    assert summary.name == "rosio_monitor: summary"
    assert summary.level == DiagnosticStatus.ERROR
    assert summary.message == "2 topics watched, 1 with findings"


def test_build_summary_without_topics_warns():
    summary = build_summary([])
    assert summary.level == DiagnosticStatus.WARN
    assert summary.message == "nothing is being watched"


def test_build_summary_takes_the_level_from_a_time_source():
    ok = DiagnosticStatus(level=DiagnosticStatus.OK, name="rosio_monitor: /a")
    source = DiagnosticStatus(level=DiagnosticStatus.ERROR, name="rosio_monitor: time sources/ptp")
    summary = build_summary([ok], [source])
    assert summary.level == DiagnosticStatus.ERROR
    assert summary.message == "1 topics watched, 0 with findings"


def encapsulated(identifier, sec, nanosec, dheader=False):
    order = "<" if identifier % 2 else ">"
    body = struct.pack(order + "I", 12) if dheader else b""
    return (
        struct.pack(">H", identifier) + b"\x00\x00" + body + struct.pack(order + "iI", sec, nanosec)
    )


def test_parse_cdr_stamp_reads_xcdr2_final():
    assert parse_cdr_stamp(encapsulated(0x0007, 12, 500000000)) == pytest.approx(12.5)


def test_parse_cdr_stamp_reads_xcdr2_delimited():
    raw = encapsulated(0x0009, 9, 125000000, dheader=True)
    assert parse_cdr_stamp(raw) == pytest.approx(9.125)


def test_parse_cdr_stamp_refuses_a_parameter_list_encapsulation():
    with pytest.raises(UnsupportedEncapsulation) as caught:
        parse_cdr_stamp(encapsulated(0x0003, 1, 0))
    assert str(caught.value) == "unsupported encapsulation parameter list CDR 0x0003"


def test_has_leading_header_accepts_a_stamped_type():
    assert has_leading_header(Imu) is True


def test_has_leading_header_rejects_a_headerless_type():
    assert has_leading_header(String) is False


def test_has_leading_header_rejects_a_header_that_is_not_first():
    assert has_leading_header(TFMessage) is False


def test_attempt_gives_up_after_five_tries():
    entry = Attempt()
    for _ in range(4):
        entry.attempts += 1
    entry.reason = "type unresolved (pkg/msg/Nope)"
    assert entry.gave_up is False
    assert entry.describe() == "type unresolved (pkg/msg/Nope)"
    entry.attempts += 1
    assert entry.gave_up is True
    assert entry.describe() == "type unresolved (pkg/msg/Nope), gave up"


def test_build_unwatched_status_lists_every_topic():
    status = build_unwatched_status(
        [
            ("/a", "unsupported encapsulation 0x0003"),
            ("/b", "type unresolved"),
            ("/c", "no leading header field, gave up"),
        ]
    )
    assert status.name == "rosio_monitor: unwatched"
    assert status.level == DiagnosticStatus.WARN
    assert status.message == (
        "3 topics not watched: /a (unsupported encapsulation 0x0003), "
        "/b (type unresolved), /c (no leading header field, gave up)"
    )
    assert {item.key for item in status.values} == {"/a", "/b", "/c"}


def test_build_unwatched_status_truncates_the_message_only():
    entries = [(f"/t{i}", "no leading header field") for i in range(12)]
    status = build_unwatched_status(entries)
    assert status.message.startswith("12 topics not watched: ")
    assert status.message.endswith(", ...")
    assert len(status.values) == 12


def test_build_summary_warns_when_nothing_is_watched():
    summary = build_summary([], [], unwatched=3)
    assert summary.level == DiagnosticStatus.WARN
    assert summary.message == "nothing is being watched, 3 not watched"


def test_build_summary_stays_ok_with_watched_topics_and_no_unwatched():
    ok = DiagnosticStatus(level=DiagnosticStatus.OK, name="rosio_monitor: /a")
    assert build_summary([ok], [], unwatched=0).level == DiagnosticStatus.OK


def test_clock_step_ignores_drift():
    assert clock_step(1000.0, 1000.4) is None


def test_clock_step_reports_a_jump():
    assert clock_step(1000.0, 1005.0) == pytest.approx(5.0)
    assert clock_step(1000.0, 990.0) == pytest.approx(-10.0)


def test_build_clock_step_status_names_the_shift():
    status = build_clock_step_status(-3.25)
    assert status.name == "rosio_monitor: host clock"
    assert status.level == DiagnosticStatus.WARN
    assert status.message == "host clock stepped by -3.250 s during monitoring"


def good_parameters():
    return {
        "report_period_s": 2.0,
        "discovery_period_s": 5.0,
        "time_source_period_s": 10.0,
        "window_size": 256,
        "plausible_window_s": 1000.0,
        "stale_after_s": 0.0,
        "jitter_warn_factor": 5.0,
    }


def test_validate_parameters_accepts_the_defaults():
    validate_parameters(good_parameters(), {"ptp_offset_warn_ns": 1000})


def test_validate_parameters_allows_a_disabled_time_source_poll():
    values = good_parameters()
    values["time_source_period_s"] = 0.0
    validate_parameters(values)


@pytest.mark.parametrize("name", ["report_period_s", "discovery_period_s", "plausible_window_s"])
def test_validate_parameters_rejects_zero_where_it_does_not_disable(name):
    values = good_parameters()
    values[name] = 0.0
    with pytest.raises(ParameterError) as caught:
        validate_parameters(values)
    assert name in str(caught.value)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0])
def test_validate_parameters_rejects_unusable_periods(bad):
    values = good_parameters()
    values["report_period_s"] = bad
    with pytest.raises(ParameterError):
        validate_parameters(values)


def test_validate_parameters_rejects_a_negative_time_source_period():
    values = good_parameters()
    values["time_source_period_s"] = -1.0
    with pytest.raises(ParameterError):
        validate_parameters(values)


@pytest.mark.parametrize("bad", [0, -5, 2.5])
def test_validate_parameters_rejects_a_bad_window_size(bad):
    values = good_parameters()
    values["window_size"] = bad
    with pytest.raises(ParameterError) as caught:
        validate_parameters(values)
    assert "window_size" in str(caught.value)


def test_validate_parameters_rejects_a_bad_threshold():
    with pytest.raises(ParameterError) as caught:
        validate_parameters(good_parameters(), {"chrony_offset_warn_s": -0.1})
    assert "chrony_offset_warn_s" in str(caught.value)


def test_validate_parameters_ignores_non_numeric_thresholds():
    validate_parameters(good_parameters(), {"expect_none": True, "pmc_socket": ""})


def test_validate_parameters_accepts_a_pmc_command():
    validate_parameters(good_parameters(), None, {"pmc_command": ["sudo", "-n", "pmc"]})


@pytest.mark.parametrize("bad", [[], [""], ["pmc", " "], "pmc", [None]])
def test_validate_parameters_rejects_a_bad_pmc_command(bad):
    with pytest.raises(ParameterError) as caught:
        validate_parameters(good_parameters(), None, {"pmc_command": bad})
    assert "pmc_command" in str(caught.value)


def stream(interval_s, count=10, offset_s=0.01, base=1754800000.0):
    stamps = [base + i * interval_s for i in range(count)]
    return stamps, [stamp + offset_s for stamp in stamps]


def stats_for(interval_s, **kwargs):
    stamps, arrivals = stream(interval_s, **kwargs)
    return window_stats(stamps, arrivals), stamps


def test_stale_limit_uses_the_floor_for_a_fast_topic():
    assert stale_limit(0.01) == STALE_FLOOR_S


def test_stale_limit_follows_the_period_of_a_slow_topic():
    assert stale_limit(3.0) == pytest.approx(STALE_INTERVAL_FACTOR * 3.0)


def test_stale_limit_clamps_to_the_ceiling():
    assert stale_limit(60.0) == STALE_CEILING_S
    assert stale_limit(0.0) == STALE_CEILING_S


def test_stale_limit_takes_the_override_verbatim():
    assert stale_limit(0.01, 4.0) == 4.0
    assert stale_limit(60.0, 4.0) == 4.0


def test_topic_verdict_warns_when_a_topic_goes_silent():
    stats, stamps = stats_for(0.1)
    level, message = topic_verdict("/odom", stats, stamps, age_s=3.0)
    assert level == DiagnosticStatus.WARN
    assert "has not published for 3.0 s" in message
    assert "10.00 Hz" in message


def test_topic_verdict_errors_on_a_long_silence():
    stats, stamps = stats_for(0.1)
    age = STALE_FLOOR_S * STALE_ERROR_FACTOR + 1.0
    level, message = topic_verdict("/odom", stats, stamps, age_s=age)
    assert level == DiagnosticStatus.ERROR
    assert "has not published" in message


def test_topic_verdict_clears_the_stale_finding_on_recovery():
    stats, stamps = stats_for(0.1)
    level, message = topic_verdict("/odom", stats, stamps, age_s=0.11)
    assert level == DiagnosticStatus.OK
    assert "plausible latency" in message


def test_a_slow_topic_is_not_stale_inside_its_own_period():
    stats, stamps = stats_for(3.0)
    level, _ = topic_verdict("/global_costmap", stats, stamps, age_s=4.0)
    assert level == DiagnosticStatus.OK


def test_stale_after_s_override_replaces_the_automatic_budget():
    stats, stamps = stats_for(0.1)
    assert (
        topic_verdict("/odom", stats, stamps, age_s=3.0, stale_after_s=10.0)[0]
        == DiagnosticStatus.OK
    )
    assert (
        topic_verdict("/odom", stats, stamps, age_s=3.0, stale_after_s=1.0)[0]
        == DiagnosticStatus.WARN
    )


def test_topic_verdict_warns_on_a_jitter_spike():
    stamps, arrivals = stream(0.1)
    arrivals[-1] += 21.8
    stats = window_stats(stamps, arrivals)
    level, message = topic_verdict("/lidar", stats, stamps, age_s=0.11)
    assert level == DiagnosticStatus.WARN
    assert "p95 jitter 21.800 s" in message
    assert "0.100 s stamp period" in message


def test_topic_verdict_ignores_ordinary_jitter():
    stamps, arrivals = stream(0.1)
    arrivals[-1] += 0.02
    stats = window_stats(stamps, arrivals)
    level, _ = topic_verdict("/lidar", stats, stamps, age_s=0.11)
    assert level == DiagnosticStatus.OK


def test_a_slow_topic_does_not_warn_on_scheduling_noise():
    stamps, arrivals = stream(3.0)
    arrivals[-1] += 0.3
    stats = window_stats(stamps, arrivals)
    assert topic_verdict("/global_costmap", stats, stamps, age_s=1.0)[0] == DiagnosticStatus.OK
    fast_stamps, fast_arrivals = stream(0.01)
    fast_arrivals[-1] += 0.3
    fast_stats = window_stats(fast_stamps, fast_arrivals)
    assert fast_stats["p95_jitter_s"] > JITTER_FLOOR_S
    assert topic_verdict("/imu", fast_stats, fast_stamps, age_s=0.02)[0] == DiagnosticStatus.WARN


def test_jitter_warn_factor_moves_the_threshold():
    stamps, arrivals = stream(0.1)
    arrivals[-1] += 0.6
    stats = window_stats(stamps, arrivals)
    assert topic_verdict("/lidar", stats, stamps, age_s=0.11)[0] == DiagnosticStatus.WARN
    level, _ = topic_verdict("/lidar", stats, stamps, age_s=0.11, jitter_warn_factor=50.0)
    assert level == DiagnosticStatus.OK


def test_an_epoch_error_outranks_a_stale_warning():
    stamps, arrivals = stream(0.1)
    arrivals = [arrival + 37.0 for arrival in arrivals]
    stats = window_stats(stamps, arrivals)
    level, message = topic_verdict("/lidar", stats, stamps, age_s=3.0)
    assert level == DiagnosticStatus.ERROR
    assert "TAI" in message


def test_a_stale_error_outranks_an_epoch_warning():
    stamps, arrivals = stream(0.1)
    arrivals = [arrival + 1.5 for arrival in arrivals]
    stats = window_stats(stamps, arrivals)
    assert topic_verdict("/lidar", stats, stamps, age_s=0.11)[0] == DiagnosticStatus.WARN
    level, message = topic_verdict("/lidar", stats, stamps, age_s=60.0)
    assert level == DiagnosticStatus.ERROR
    assert "has not published" in message


def test_build_status_publishes_the_window_age():
    stats, _ = stats_for(0.1)
    status = build_status("/odom", stats, DiagnosticStatus.WARN, "silent", 3.25)
    values = {item.key: item.value for item in status.values}
    assert values["window_age_s"] == "3.250"


def test_build_summary_counts_stale_topics():
    stale = DiagnosticStatus(level=DiagnosticStatus.WARN, name="rosio_monitor: /odom")
    summary = build_summary([stale], [], unwatched=0, stale=1)
    assert summary.message == "1 topics watched, 1 with findings, 1 stale"


def test_a_single_backward_step_is_an_error_while_the_median_stays_healthy():
    stamps, arrivals = stream(0.01, count=512)
    stamps[300] = stamps[299] - 0.2
    stats = window_stats(stamps, arrivals)
    assert abs(stats["median_offset_s"]) <= 0.5
    assert level_for_offset("/imu", stats["median_offset_s"], stamps)[0] == DiagnosticStatus.OK
    level, message = topic_verdict("/imu", stats, stamps, age_s=0.02)
    assert level == DiagnosticStatus.ERROR
    assert "stamps go backwards, largest jump 0.2" in message
    assert stats["backward_step_s"] == pytest.approx(0.2)


def test_a_clean_stream_reports_no_backward_step():
    stats, stamps = stats_for(0.01, count=512)
    assert stats["backward_step_s"] == 0.0
    assert topic_verdict("/imu", stats, stamps, age_s=0.02)[0] == DiagnosticStatus.OK


def test_a_partial_zero_run_is_an_error():
    stamps, arrivals = stream(0.1, count=40)
    for i in range(10, 15):
        stamps[i] = 0.0
    stats = window_stats(stamps, arrivals)
    level, message = topic_verdict("/imu", stats, stamps, age_s=0.11)
    assert level == DiagnosticStatus.ERROR
    assert message == "/imu has 5 of 40 messages with a zero stamp"


def test_a_partial_frozen_run_is_an_error():
    stamps, arrivals = stream(0.1, count=60)
    for i in range(20, 40):
        stamps[i] = stamps[20]
    stats = window_stats(stamps, arrivals)
    assert stats["frozen"] is False
    level, message = topic_verdict("/lidar", stats, stamps, age_s=0.11)
    assert level == DiagnosticStatus.ERROR
    assert "stamp frozen at" in message
    assert "for 20 messages over 1.900 s" in message


def test_a_short_repeat_is_not_called_frozen():
    stamps, arrivals = stream(0.1, count=60)
    for i in range(20, 25):
        stamps[i] = stamps[20]
    stats = window_stats(stamps, arrivals)
    assert stats["frozen_run"] is None
    assert topic_verdict("/lidar", stats, stamps, age_s=0.11)[0] == DiagnosticStatus.OK


def test_a_frozen_window_keeps_its_own_message():
    stamps = [1754800000.0] * 20
    arrivals = [1754800000.0 + i * 0.1 for i in range(20)]
    stats = window_stats(stamps, arrivals)
    level, message = topic_verdict("/imu", stats, stamps)
    assert level == DiagnosticStatus.ERROR
    assert "is frozen at" in message


def test_build_status_publishes_the_intermittent_evidence():
    stamps, arrivals = stream(0.01, count=512)
    stamps[300] = stamps[299] - 0.2
    stats = window_stats(stamps, arrivals)
    status = build_status("/imu", stats, DiagnosticStatus.ERROR, "backwards", 0.02)
    values = {item.key: item.value for item in status.values}
    assert float(values["backward_step_s"]) == pytest.approx(0.2)
    assert values["frozen_run"] == "0"


@pytest.mark.parametrize("name,bad", [("stale_after_s", -1.0), ("jitter_warn_factor", 0.0)])
def test_validate_parameters_rejects_bad_grading_parameters(name, bad):
    values = good_parameters()
    values[name] = bad
    with pytest.raises(ParameterError) as caught:
        validate_parameters(values)
    assert name in str(caught.value)
