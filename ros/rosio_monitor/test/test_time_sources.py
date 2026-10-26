import pytest
from rosio_monitor.time_sources import (
    ERROR,
    OK,
    WARN,
    assess,
    chrony_status,
    poll,
    ptp_status,
    timesyncd_status,
)

PMC_SLAVE = """sending: GET TIME_STATUS_NP
sending: GET PORT_DATA_SET
\tac3ae2.fffe.217ed6-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP\x20
\t\tmaster_offset              31337
\t\tingress_time               1754800000123456789
\t\tcumulativeScaledRateOffset +0.000000000
\t\tscaledLastGmPhaseChange    0
\t\tgmTimeBaseIndicator        0
\t\tlastGmPhaseChange          0x0000'0000000000000000.0000
\t\tgmPresent                  true
\t\tgmIdentity                 001b19.fffe.0a1b2c
\tac3ae2.fffe.217ed6-1 seq 1 RESPONSE MANAGEMENT PORT_DATA_SET\x20
\t\tportIdentity            ac3ae2.fffe.217ed6-1
\t\tportState               SLAVE
\t\tlogMinDelayReqInterval  0
\t\tpeerMeanPathDelay       0
\t\tlogAnnounceInterval     1
\t\tannounceReceiptTimeout  3
\t\tlogSyncInterval         0
\t\tdelayMechanism          1
\t\tlogMinPdelayReqInterval 0
\t\tversionNumber           2
"""

PMC_MASTER = (
    PMC_SLAVE.replace("SLAVE", "MASTER")
    .replace("gmPresent                  true", "gmPresent                  false")
    .replace("master_offset              31337", "master_offset              0")
)

PMC_FAULTY = PMC_SLAVE.replace("portState               SLAVE", "portState               FAULTY")

PMC_NO_GRANDMASTER = PMC_SLAVE.replace(
    "gmPresent                  true", "gmPresent                  false"
)

PMC_UDS_DENIED = "pmc[68339.918]: uds: bind failed: Permission denied\nfailed to create pmc\n"

PMC_TIMEOUT = "pmc[70112.004]: timed out while polling for tid 1\n"

PMC_NOISE = "sending: GET TIME_STATUS_NP\n"

PMC_BAD_OFFSET = PMC_SLAVE.replace(
    "master_offset              31337", "master_offset              unknown"
)

CHRONY_SYNCED = (
    "C0000201,192.0.2.1,2,1754800000.123456789,0.000123456,0.000023456,"
    "0.000045678,-1.234,0.005,0.123,0.001234,0.005678,64.2,Normal\n"
)

CHRONY_OFFSET = CHRONY_SYNCED.replace("0.000123456", "0.350000000")

CHRONY_UNSYNCED = (
    "00000000,,0,0.000000000,0.000000000,0.000000000,0.000000000,0.000,0.000,"
    "1.000,1.000000000,1.000000000,0.0,Not synchronised\n"
)

CHRONY_LEAP_INSERT = CHRONY_SYNCED.replace("Normal", "Insert second")

CHRONY_BAD_OFFSET = CHRONY_SYNCED.replace("0.000123456", "nan_offset")

TIMESYNC_SHOW_TIMESYNC = """FallbackNTPServers=0.pool.ntp.org 1.pool.ntp.org
ServerName=0.pool.ntp.org
ServerAddress=198.51.100.10
RootDistanceMaxUSec=5s
PollIntervalUSec=32s
NTPMessage={ Leap=0, Version=4, Mode=4, Stratum=1, Precision=-25, RootDelay=0, \
RootDispersion=0, Reference=GPS, Ignored=yes PacketCount=5, Jitter=34.213ms }
Frequency=-17618769
"""

TIMESYNC_SHOW = """Timezone=Europe/Berlin
LocalRTC=no
CanNTP=yes
NTP=yes
NTPSynchronized=yes
TimeUSec=Tue 2026-08-11 09:25:43 CEST
"""

TIMESYNC_NOT_A_UNIT = (
    "Failed to query server: Unit dbus-org.freedesktop.timesync1.service not found.\n"
)


def runner_for(replies, missing=()):
    def runner(argv):
        program = argv[0].rsplit("/", 1)[-1]
        if program in missing:
            raise FileNotFoundError(2, "No such file or directory", argv[0])
        return replies[(program, *argv[1:])]

    return runner


def pmc_runner(stdout, code=0, stderr="", missing=(), seen=None):
    def runner(argv):
        if seen is not None:
            seen.append(tuple(argv))
        if argv[0].rsplit("/", 1)[-1] in missing:
            raise FileNotFoundError(2, "No such file or directory", argv[0])
        return code, stdout, stderr

    return runner


def data_of(source):
    assert source["state"] == "ok", source
    return source["data"]


def named(verdicts, name):
    return next(verdict for verdict in verdicts if verdict[1] == name)


def ptp_data(stdout):
    return {"ptp": {"state": "ok", "data": _parsed_ptp(stdout), "detail": ""}}


def _parsed_ptp(stdout):
    return data_of(ptp_status(pmc_runner(stdout)))


def chrony_ok(text):
    return chrony_status(runner_for({("chronyc", "-c", "tracking"): (0, text, "")}))


def timesyncd_ok(show=TIMESYNC_SHOW):
    return timesyncd_status(
        runner_for(
            {
                ("timedatectl", "show-timesync"): (0, TIMESYNC_SHOW_TIMESYNC, ""),
                ("timedatectl", "show"): (0, show, ""),
            }
        )
    )


def test_ptp_status_parses_a_slave_port():
    data = data_of(ptp_status(pmc_runner(PMC_SLAVE)))
    assert data["master_offset_ns"] == 31337
    assert data["gm_present"] is True
    assert data["gm_identity"] == "001b19.fffe.0a1b2c"
    assert data["port_state"] == "SLAVE"


def test_ptp_status_parses_a_master_port():
    data = data_of(ptp_status(pmc_runner(PMC_MASTER)))
    assert data["port_state"] == "MASTER"
    assert data["gm_present"] is False
    assert data["master_offset_ns"] == 0


def test_ptp_status_absent_when_pmc_is_missing():
    source = ptp_status(pmc_runner("", missing=("pmc",)))
    assert source["state"] == "absent"
    assert "pmc" in source["detail"]


def test_ptp_status_permission_denied_on_the_uds_bind_failure():
    source = ptp_status(pmc_runner("", code=255, stderr=PMC_UDS_DENIED))
    assert source["state"] == "permission_denied"
    assert "bind failed" in source["detail"]


def test_ptp_status_failed_carries_the_first_stderr_line():
    source = ptp_status(pmc_runner("", code=1, stderr=PMC_TIMEOUT))
    assert source["state"] == "failed"
    assert source["detail"] == "pmc[70112.004]: timed out while polling for tid 1"


def test_ptp_status_unparseable_when_pmc_exits_clean_with_noise():
    source = ptp_status(pmc_runner(PMC_NOISE))
    assert source["state"] == "unparseable"
    assert source["detail"]


def test_ptp_status_unparseable_when_a_slave_offset_is_not_a_number():
    source = ptp_status(pmc_runner(PMC_BAD_OFFSET))
    assert source["state"] == "unparseable"
    assert "master_offset" in source["detail"]
    assert "unknown" in source["detail"]


def test_ptp_status_falls_back_to_the_bare_name():
    seen = []

    def runner(argv):
        seen.append(argv[0])
        if argv[0] == "/usr/sbin/pmc":
            raise FileNotFoundError(2, "No such file or directory", argv[0])
        return 0, PMC_SLAVE, ""

    assert data_of(ptp_status(runner))["port_state"] == "SLAVE"
    assert seen == ["/usr/sbin/pmc", "pmc"]


def test_ptp_status_passes_the_domain_flag():
    seen = []
    ptp_status(pmc_runner(PMC_SLAVE, seen=seen), {"ptp_domain": 24})
    argv = seen[0]
    assert argv[argv.index("-d") + 1] == "24"
    assert "-s" not in argv


def test_ptp_status_passes_the_socket_flag():
    seen = []
    ptp_status(pmc_runner(PMC_SLAVE, seen=seen), {"pmc_socket": "/var/run/ptp4l.sock"})
    argv = seen[0]
    assert argv[argv.index("-s") + 1] == "/var/run/ptp4l.sock"
    assert argv[argv.index("-d") + 1] == "0"


def test_ptp_status_uses_the_configured_command():
    seen = []
    ptp_status(pmc_runner(PMC_SLAVE, seen=seen), {"pmc_command": ["/usr/local/bin/ptp-status"]})
    assert len(seen) == 1
    assert seen[0][0] == "/usr/local/bin/ptp-status"
    assert seen[0][-2:] == ("GET TIME_STATUS_NP", "GET PORT_DATA_SET")


def test_ptp_status_keeps_a_command_prefix_in_front_of_the_query():
    seen = []
    source = ptp_status(
        pmc_runner(PMC_SLAVE, seen=seen), {"pmc_command": ["sudo", "-n", "pmc"], "ptp_domain": 7}
    )
    argv = seen[0]
    assert argv[:3] == ("sudo", "-n", "pmc")
    assert argv[argv.index("-d") + 1] == "7"
    assert data_of(source)["port_state"] == "SLAVE"


def test_ptp_status_does_not_fall_back_when_a_command_is_configured():
    seen = []
    source = ptp_status(
        pmc_runner("", missing=("ptp-status",), seen=seen), {"pmc_command": ["ptp-status"]}
    )
    assert len(seen) == 1
    assert source["state"] == "absent"


def test_ptp_status_default_command_still_tries_the_sbin_path_first():
    seen = []
    ptp_status(pmc_runner(PMC_SLAVE, seen=seen), {"pmc_command": ["pmc"]})
    assert seen[0][0] == "/usr/sbin/pmc"


def test_assess_ptp_slave_is_ok_below_the_warn_threshold():
    level, name, message, values = named(assess(ptp_data(PMC_SLAVE)), "ptp")
    assert (level, name) == (OK, "ptp")
    assert "slaved to 001b19.fffe.0a1b2c" in message
    assert values["master_offset_ns"] == "31337"


def master_with(**sources):
    observations = ptp_data(PMC_MASTER)
    observations.update(sources)
    return assess(observations)


def test_assess_ptp_master_is_ok_when_chrony_disciplines_the_system_clock():
    level, _, message, values = named(master_with(chrony=chrony_ok(CHRONY_SYNCED)), "ptp")
    assert level == OK
    assert "serves the clock, disciplined by chrony" in message
    assert values["system_clock"] == "chrony"


def test_assess_ptp_master_is_ok_when_timesyncd_disciplines_the_system_clock():
    level, _, message, values = named(master_with(timesyncd=timesyncd_ok()), "ptp")
    assert level == OK
    assert "disciplined by timesyncd" in message
    assert values["system_clock"] == "timesyncd"


def test_assess_ptp_master_warns_when_nothing_disciplines_it():
    level, _, message, values = named(master_with(), "ptp")
    assert level == WARN
    assert "nothing disciplines it" in message
    assert "no healthy chrony or timesyncd observation" in message
    assert values["system_clock"] == "none"


def test_assess_ptp_master_warns_when_chrony_is_denied():
    denied = {"state": "permission_denied", "data": {}, "detail": "Permission denied"}
    verdicts = master_with(chrony=denied)
    assert named(verdicts, "ptp")[0] == WARN
    assert named(verdicts, "chrony")[0] == WARN


def test_assess_ptp_master_warns_when_chrony_is_unsynchronized():
    level, _, message, _ = named(master_with(chrony=chrony_ok(CHRONY_UNSYNCED)), "ptp")
    assert level == WARN
    assert "nothing disciplines it" in message


def test_assess_ptp_slave_grading_is_unchanged_by_the_system_clock():
    alone = named(assess(ptp_data(PMC_SLAVE)), "ptp")
    with_chrony = named(assess(dict(ptp_data(PMC_SLAVE), chrony=chrony_ok(CHRONY_SYNCED))), "ptp")
    assert alone[0] == with_chrony[0] == OK
    assert alone[2] == with_chrony[2]
    assert "system_clock" not in alone[3]


def test_assess_ptp_warns_on_a_faulty_port():
    level, _, message, _ = named(assess(ptp_data(PMC_FAULTY)), "ptp")
    assert level == WARN
    assert "FAULTY" in message


def test_assess_ptp_errors_when_the_grandmaster_is_absent():
    level, _, message, _ = named(assess(ptp_data(PMC_NO_GRANDMASTER)), "ptp")
    assert level == ERROR
    assert "no grandmaster" in message


def test_assess_ptp_warns_above_the_offset_warn_threshold():
    sources = ptp_data(PMC_SLAVE)
    sources["ptp"]["data"]["master_offset_ns"] = 2000000
    level, _, message, _ = named(assess(sources), "ptp")
    assert level == WARN
    assert "warn threshold" in message


def test_assess_ptp_errors_above_the_offset_error_threshold():
    sources = ptp_data(PMC_SLAVE)
    sources["ptp"]["data"]["master_offset_ns"] = -25000000
    level, _, message, _ = named(assess(sources), "ptp")
    assert level == ERROR
    assert "error threshold" in message


def test_assess_ptp_honours_custom_thresholds():
    level, _, _, _ = named(assess(ptp_data(PMC_SLAVE), {"ptp_offset_warn_ns": 1000}), "ptp")
    assert level == WARN


def test_assess_warns_on_permission_denied_with_the_detail():
    sources = {"ptp": ptp_status(pmc_runner("", code=255, stderr=PMC_UDS_DENIED))}
    level, _, message, values = named(assess(sources), "ptp")
    assert level == WARN
    assert "bind failed: Permission denied" in message
    assert values["state"] == "permission_denied"


def test_assess_warns_on_a_failed_check_with_the_stderr_detail():
    sources = {"ptp": ptp_status(pmc_runner("", code=1, stderr=PMC_TIMEOUT))}
    level, _, message, _ = named(assess(sources), "ptp")
    assert level == WARN
    assert "timed out while polling" in message


def test_assess_warns_on_an_unparseable_answer():
    sources = {"ptp": ptp_status(pmc_runner(PMC_NOISE))}
    level, _, message, _ = named(assess(sources), "ptp")
    assert level == WARN
    assert "does not understand" in message


def test_chrony_status_parses_tracking_csv():
    data = data_of(chrony_ok(CHRONY_SYNCED))
    assert data["reference_id"] == "C0000201"
    assert data["reference_name"] == "192.0.2.1"
    assert data["stratum"] == 2
    assert data["offset_s"] == pytest.approx(0.000123456)
    assert data["leap_status"] == "Normal"


def test_chrony_status_absent_when_chronyc_is_missing():
    assert chrony_status(runner_for({}, missing=("chronyc",)))["state"] == "absent"


def test_chrony_status_absent_when_the_daemon_is_not_running():
    runner = runner_for({("chronyc", "-c", "tracking"): (1, "", "506 Cannot talk to daemon\n")})
    source = chrony_status(runner)
    assert source["state"] == "absent"
    assert "Cannot talk to daemon" in source["detail"]


def test_chrony_status_failed_on_any_other_error():
    runner = runner_for({("chronyc", "-c", "tracking"): (1, "", "Could not open socket\n")})
    source = chrony_status(runner)
    assert source["state"] == "failed"
    assert source["detail"] == "Could not open socket"


def test_chrony_status_unparseable_on_a_short_line():
    runner = runner_for({("chronyc", "-c", "tracking"): (0, "C0000201,192.0.2.1\n", "")})
    assert chrony_status(runner)["state"] == "unparseable"


def test_chrony_status_unparseable_when_the_offset_is_not_a_number():
    runner = runner_for({("chronyc", "-c", "tracking"): (0, CHRONY_BAD_OFFSET, "")})
    source = chrony_status(runner)
    assert source["state"] == "unparseable"
    assert "chrony offset" in source["detail"]


def test_assess_chrony_warns_on_an_unreadable_offset():
    runner = runner_for({("chronyc", "-c", "tracking"): (0, CHRONY_BAD_OFFSET, "")})
    level, _, message, _ = named(assess({"chrony": chrony_status(runner)}), "chrony")
    assert level == WARN
    assert "does not understand" in message


def test_assess_chrony_ok_when_synchronized():
    level, _, message, values = named(assess({"chrony": chrony_ok(CHRONY_SYNCED)}), "chrony")
    assert level == OK
    assert "192.0.2.1" in message
    assert values["stratum"] == "2"


def test_assess_chrony_warns_on_a_large_offset():
    level, _, message, _ = named(assess({"chrony": chrony_ok(CHRONY_OFFSET)}), "chrony")
    assert level == WARN
    assert "warn threshold" in message


def test_assess_chrony_errors_when_unsynchronized():
    level, _, message, _ = named(assess({"chrony": chrony_ok(CHRONY_UNSYNCED)}), "chrony")
    assert level == ERROR
    assert "not synchronized" in message


def test_assess_chrony_warns_on_a_leap_second():
    level, _, message, _ = named(assess({"chrony": chrony_ok(CHRONY_LEAP_INSERT)}), "chrony")
    assert level == WARN
    assert "Insert second" in message


def test_timesyncd_status_parses_show_timesync():
    data = data_of(timesyncd_ok())
    assert data["server"] == "0.pool.ntp.org"
    assert data["server_address"] == "198.51.100.10"
    assert data["stratum"] == 1
    assert data["jitter"] == "34.213ms"
    assert data["synchronized"] is True


def test_timesyncd_status_absent_when_timedatectl_is_missing():
    assert timesyncd_status(runner_for({}, missing=("timedatectl",)))["state"] == "absent"


def test_timesyncd_status_absent_when_the_unit_is_not_installed():
    runner = runner_for({("timedatectl", "show-timesync"): (1, "", TIMESYNC_NOT_A_UNIT)})
    source = timesyncd_status(runner)
    assert source["state"] == "absent"
    assert "timesync1.service" in source["detail"]


def test_timesyncd_status_permission_denied():
    runner = runner_for(
        {("timedatectl", "show-timesync"): (1, "", "Failed to query server: Access denied\n")}
    )
    assert timesyncd_status(runner)["state"] == "permission_denied"


def test_assess_timesyncd_warns_when_not_synchronized():
    source = timesyncd_ok(TIMESYNC_SHOW.replace("NTPSynchronized=yes", "NTPSynchronized=no"))
    level, _, message, _ = named(assess({"timesyncd": source}), "timesyncd")
    assert level == WARN
    assert "not synchronized" in message


def test_assess_reports_every_source_present():
    sources = {"ptp": ptp_data(PMC_SLAVE)["ptp"], "chrony": chrony_ok(CHRONY_SYNCED)}
    assert [verdict[1] for verdict in assess(sources)] == ["ptp", "chrony"]


def test_assess_absent_sources_alone_do_not_hide_a_working_one():
    sources = {
        "ptp": {"state": "absent", "data": {}, "detail": "pmc is not installed"},
        "chrony": chrony_ok(CHRONY_SYNCED),
        "timesyncd": None,
    }
    verdicts = assess(sources)
    assert [verdict[1] for verdict in verdicts] == ["chrony"]


def test_assess_warns_when_every_source_is_absent():
    absent = {"state": "absent", "data": {}, "detail": "not installed"}
    verdicts = assess({"ptp": absent, "chrony": absent, "timesyncd": absent})
    level, name, message, values = verdicts[0]
    assert len(verdicts) == 1
    assert (level, name) == (WARN, "none")
    assert message == "no time sources observed, the clock is unmonitored"
    assert values == {"ptp": "absent", "chrony": "absent", "timesyncd": "absent"}


def test_assess_all_absent_is_ok_when_expect_none_is_set():
    absent = {"state": "absent", "data": {}, "detail": "not installed"}
    sources = {"ptp": absent, "chrony": absent, "timesyncd": absent}
    assert assess(sources, {"expect_none": True})[0][0] == OK
    assert assess(sources, expect_none=True)[0][0] == OK
    assert assess(sources, {"expect_none": True}, expect_none=False)[0][0] == WARN


def test_assess_warns_when_nothing_was_attempted():
    level, name, _, values = assess({"ptp": None, "chrony": None, "timesyncd": None})[0]
    assert (level, name) == (WARN, "none")
    assert values["ptp"] == "not attempted"


def test_poll_skips_timesyncd_only_when_chrony_answers():
    runner = runner_for({("chronyc", "-c", "tracking"): (0, CHRONY_SYNCED, "")}, missing=("pmc",))
    sources = poll(runner)
    assert sources["ptp"]["state"] == "absent"
    assert sources["chrony"]["state"] == "ok"
    assert sources["timesyncd"] is None


def test_poll_still_tries_timesyncd_when_chrony_is_denied():
    runner = runner_for(
        {
            ("chronyc", "-c", "tracking"): (1, "", "Permission denied\n"),
            ("timedatectl", "show-timesync"): (0, TIMESYNC_SHOW_TIMESYNC, ""),
            ("timedatectl", "show"): (0, TIMESYNC_SHOW, ""),
        },
        missing=("pmc",),
    )
    sources = poll(runner)
    assert sources["chrony"]["state"] == "permission_denied"
    assert sources["timesyncd"]["state"] == "ok"
    assert named(assess(sources), "chrony")[0] == WARN
