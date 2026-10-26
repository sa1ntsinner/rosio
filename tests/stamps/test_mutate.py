import struct

import pytest
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from rosio.stamps.checks import CHECKS
from rosio.stamps.fallback import UnsupportedEncapsulation
from rosio.stamps.mutate import (
    DEFAULT_MAGNITUDE,
    NS,
    PATHOLOGIES,
    PRESETS,
    _patch,
    _read_ns,
    main,
    mutate,
)
from rosio.stamps.read import read_topic_series

from .conftest import write_bag

BASE = 1750000000.0
COUNT = 100
PERIOD = 0.1
LATENCY = 0.02


@pytest.fixture
def source(tmp_path):
    stamps = [BASE + i * PERIOD for i in range(COUNT)]
    return write_bag(tmp_path / "source", stamps, [stamp + LATENCY for stamp in stamps])


def findings(path, topic="/scan"):
    series = read_topic_series(path)
    found = []
    for check in CHECKS.values():
        found.extend(f for f in check(series) if f.topic == topic)
    return found


def by_check(path, name, topic="/scan"):
    return [f for f in findings(path, topic) if f.check == name]


def raw_messages(path, topic):
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        return [
            (timestamp, bytes(raw))
            for _, timestamp, raw in reader.messages(connections=connections)
        ]


def test_clean_source_has_no_error(source):
    assert [f for f in findings(source) if f.severity == "error"] == []


def test_offset_preset_magnitudes():
    assert PRESETS["gps_leap"] == 18.0
    assert PRESETS["tai"] == 37.0
    assert PRESETS["ouster_ptp"] == 74986812.0
    assert PRESETS["septentrio"] == 9000.0


def test_offset_tai_fires_epoch_offset(source, tmp_path):
    dest = tmp_path / "tai"
    report = mutate(source, dest, "tai")
    assert report["/scan"]["mutated"] == COUNT
    found = by_check(dest, "epoch_offset")
    assert len(found) == 1
    assert found[0].severity == "error"
    assert found[0].evidence["median_offset_s"] == pytest.approx(
        -PRESETS["tai"] + LATENCY, abs=1e-3
    )


@pytest.mark.parametrize("magnitude", [PRESETS["tai"], -PRESETS["tai"]])
def test_offset_tai_class_both_signs(source, tmp_path, magnitude):
    dest = tmp_path / "tai_{}".format("ahead" if magnitude > 0 else "lag")
    mutate(source, dest, "tai", magnitude=magnitude)
    found = by_check(dest, "epoch_offset")
    assert [f.evidence["class"] for f in found] == ["tai utc offset"]


def test_backward_fires_monotonic(source, tmp_path):
    dest = tmp_path / "backward"
    mutate(source, dest, "backward")
    found = by_check(dest, "monotonic")
    assert len(found) == 1
    assert found[0].evidence["largest_backward_jump"] == pytest.approx(1.5 - PERIOD, abs=1e-6)
    assert found[0].evidence["index"] == COUNT // 2


def test_drift_fires_clock_drift(source, tmp_path):
    dest = tmp_path / "drift"
    mutate(source, dest, "drift", magnitude=0.02, at=0.0)
    found = by_check(dest, "clock_drift")
    assert len(found) == 1
    assert found[0].severity == "error"
    assert found[0].evidence["slope_s_per_s"] == pytest.approx(-0.02, abs=1e-4)


def test_drift_grows_with_time(source, tmp_path):
    dest = tmp_path / "drift_growth"
    mutate(source, dest, "drift", magnitude=0.02, at=0.0)
    stamps = read_topic_series(dest)["/scan"].stamps
    assert stamps[0] == pytest.approx(BASE, abs=1e-6)
    for index in (COUNT // 2, COUNT - 1):
        assert stamps[index] - BASE == pytest.approx(index * PERIOD * 1.02, abs=1e-3)


def test_drift_starts_at_the_fraction(source, tmp_path):
    dest = tmp_path / "drift_late"
    mutate(source, dest, "drift", magnitude=0.02, at=0.5)
    stamps = read_topic_series(dest)["/scan"].stamps
    assert stamps[COUNT // 2] == pytest.approx(BASE + COUNT // 2 * PERIOD, abs=1e-6)
    assert stamps[-1] > BASE + (COUNT - 1) * PERIOD


def test_drift_default_magnitude_is_a_slope():
    assert DEFAULT_MAGNITUDE["drift"] == 0.005


def test_frozen_fires_frozen_stamps(source, tmp_path):
    dest = tmp_path / "frozen"
    mutate(source, dest, "frozen")
    found = by_check(dest, "frozen_stamps")
    assert len(found) == 1
    assert found[0].evidence["run_length"] == 25
    assert found[0].evidence["start_index"] == COUNT // 2


def test_zero_fires_zero_stamps(source, tmp_path):
    dest = tmp_path / "zero"
    mutate(source, dest, "zero")
    found = by_check(dest, "zero_stamps")
    assert len(found) == 1
    assert found[0].evidence["count"] == 5
    assert found[0].evidence["all_zero"] is False


def test_dropout_fires_gaps(source, tmp_path):
    dest = tmp_path / "dropout"
    report = mutate(source, dest, "dropout")
    assert report["/scan"]["dropped"] == 20
    found = [f for f in findings(dest) if f.check == "stamp_gaps" and f.severity == "warning"]
    assert len(found) == 1
    assert found[0].evidence["max_gap_s"] == pytest.approx(2.0 + PERIOD, abs=1e-6)


def test_dropout_keeps_the_other_messages(source, tmp_path):
    dest = tmp_path / "dropout_count"
    mutate(source, dest, "dropout")
    assert len(read_topic_series(dest)["/scan"].stamps) == COUNT - 20


def test_length_and_at_are_honoured(source, tmp_path):
    dest = tmp_path / "frozen_early"
    mutate(source, dest, "frozen", at=0.1, length=40)
    found = by_check(dest, "frozen_stamps")
    assert found[0].evidence["run_length"] == 40
    assert found[0].evidence["start_index"] == 10


def test_non_header_topic_is_byte_identical(source, tmp_path):
    dest = tmp_path / "offset"
    mutate(source, dest, "septentrio")
    assert raw_messages(dest, "/chatter") == raw_messages(source, "/chatter")


def test_receive_times_are_untouched(source, tmp_path):
    dest = tmp_path / "receive"
    mutate(source, dest, "offset", magnitude=5.0)
    assert [t for t, _ in raw_messages(dest, "/scan")] == [
        t for t, _ in raw_messages(source, "/scan")
    ]


def test_topic_selection_rejects_unknown_topic(source, tmp_path):
    with pytest.raises(ValueError):
        mutate(source, tmp_path / "unknown", "offset", topics=["/chatter"])


def test_main_writes_a_bag(source, tmp_path, capsys):
    dest = tmp_path / "cli"
    assert main([str(source), str(dest), "--pathology", "gps_leap", "--topic", "/scan"]) == 0
    assert "mutated=100" in capsys.readouterr().out
    assert by_check(dest, "epoch_offset")


def test_main_rejects_a_bad_fraction(source, tmp_path):
    assert main([str(source), str(tmp_path / "bad"), "--pathology", "frozen", "--at", "1.5"]) == 2


def test_main_writes_a_drift_bag(source, tmp_path, capsys):
    dest = tmp_path / "drift_cli"
    argv = [str(source), str(dest), "--pathology", "drift", "--at", "0", "--magnitude", "0.02"]
    assert main(argv) == 0
    assert "drift" in capsys.readouterr().out
    assert by_check(dest, "clock_drift")


def test_drift_is_offered_on_the_command_line():
    assert "drift" in PATHOLOGIES


def record(identifier, sec, nanosec, dheader=False):
    order = "<" if identifier % 2 else ">"
    body = struct.pack(order + "I", 12) if dheader else b""
    return (
        struct.pack(">H", identifier) + b"\x00\x00" + body + struct.pack(order + "iI", sec, nanosec)
    )


def test_headerless_input_exits_two(headless_bag, tmp_path, capsys):
    assert main([str(headless_bag), str(tmp_path / "empty"), "--pathology", "offset"]) == 2
    assert "no topic to mutate" in capsys.readouterr().err


def test_mutating_nothing_exits_two(source, tmp_path, capsys):
    argv = [str(source), str(tmp_path / "nothing"), "--pathology", "offset", "--magnitude", "0"]
    assert main(argv) == 2
    assert "no message was mutated" in capsys.readouterr().err


def test_delimited_xcdr2_record_is_patched_at_the_shifted_offset():
    raw = record(0x0009, 5, 0, dheader=True)
    assert _read_ns(raw) == 5 * NS
    patched = _patch(raw, 7 * NS + 250000000)
    assert patched[:8] == raw[:8]
    assert struct.unpack_from("<iI", patched, 8) == (7, 250000000)
    assert _read_ns(patched) == 7 * NS + 250000000


def test_final_xcdr2_record_is_patched_at_offset_four():
    raw = record(0x0006, 5, 0)
    patched = _patch(raw, 8 * NS)
    assert struct.unpack_from(">iI", patched, 4) == (8, 0)


def test_parameter_list_record_is_refused_by_name():
    with pytest.raises(UnsupportedEncapsulation) as caught:
        _read_ns(record(0x0003, 5, 0))
    assert str(caught.value) == "unsupported encapsulation parameter list CDR 0x0003"
