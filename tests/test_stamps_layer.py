"""Recorded stamp checks through extraction, with real serialized bag messages."""

from __future__ import annotations

import builtins
from pathlib import Path

import pytest

from rosio import extract, ids, model, schema
from rosio.collect import stamps
from rosio.config import Config, RootSpec
from rosio.drift import rules
from rosio.errors import UsageError
from tests.stamps.conftest import BASE_S, SCAN_TYPE, TYPESTORE, Writer, scan_message, write_bag


@pytest.fixture
def recordings(tmp_path: Path) -> Path:
    root = tmp_path / "recordings"
    root.mkdir()
    times = [BASE_S + i * 0.2 for i in range(40)]
    write_bag(root / "clean", times, [stamp + 0.05 for stamp in times])
    with Writer(root / "broken", version=8) as writer:
        for topic in ("/tai", "/backward", "/reference"):
            connection = writer.add_connection(topic, SCAN_TYPE, typestore=TYPESTORE)
            for index, receive in enumerate(times):
                stamp = receive + 37 if topic == "/tai" else receive
                if topic == "/backward" and index == 20:
                    stamp -= 0.7
                writer.write(
                    connection,
                    int((receive + 0.05) * 1e9),
                    TYPESTORE.serialize_cdr(scan_message(stamp), SCAN_TYPE),
                )
    return root


def _extract(tmp_path: Path, root: Path | None) -> model.Atlas:
    from rosio.workspace import Workspace

    source = tmp_path / "src"
    source.mkdir(exist_ok=True)
    roots = (RootSpec("src", str(source)),)
    if root is not None:
        roots += (RootSpec("bag", str(root)),)
    cfg = Config(roots=roots, profiles=(), out_dir=str(tmp_path / "out"), deterministic=True)
    return extract.run(cfg, Workspace.discover(roots), collectors=("bag", "stamps"))


def _passes(atlas: model.Atlas) -> list[dict]:
    return [
        row
        for entity in atlas.entities.values()
        if entity.kind == "run"
        for row in entity.attrs.get("passes", [])
        if row["collector"] == "stamps"
    ]


def test_recorded_stamp_findings_validate_and_repeat(recordings: Path, tmp_path: Path) -> None:
    first = _extract(tmp_path, recordings)
    second = _extract(tmp_path, recordings)
    assert schema.validate(first.to_json()) == []
    assert first.canonical_bytes() == second.canonical_bytes()
    findings = [row for row in first.findings if row.rule.startswith("time.")]
    subjects = {(row.rule, row.severity, row.subject) for row in findings}
    assert ("time.epoch_offset", "error", ids.build_bagtopic("broken", "/tai")) in subjects
    assert ("time.backward", "error", ids.build_bagtopic("broken", "/backward")) in subjects
    assert ("time.topic_skew", "error", ids.build_bagtopic("broken", "/tai")) in subjects
    assert all(ids.parse_bagtopic(row.subject).bag == "broken" for row in findings)
    assert [row.id for row in findings] == [
        row.id for row in second.findings if row.rule.startswith("time.")
    ]
    assert all(row.where and all(site.root == "bag" for site in row.where) for row in findings)
    assert all("broken" in row.say and " s" in row.say for row in findings)
    epoch = next(row for row in findings if row.rule == "time.epoch_offset")
    assert "PTP" in epoch.fix
    clean = first.entities[ids.build_bagtopic("clean", "/scan")].fact("recorded")
    assert clean.values["stamps"] == {"messages": 40, "checks": [], "offset_s": 0.049999952}
    assert clean.values["volatile"]["stamps"]["rate_hz"] > 0
    assert clean.values["volatile"]["message_count"] == 40
    assert "stamp_rate" not in str(clean.values["stamps"])
    assert not [row for row in first.findings if row.rule == "extract.pass_incomplete"]
    assert _passes(first)[0]["state"] == "completed"


def test_clean_recording_has_no_time_findings(recordings: Path, tmp_path: Path) -> None:
    atlas = _extract(tmp_path, recordings / "clean")
    assert not [row for row in atlas.findings if row.rule.startswith("time.")]
    assert (
        atlas.entities[ids.build_bag("clean")].fact("recorded").values["stamp_verdict"]["level"]
        == "info"
    )


def test_missing_reader_refuses_only_the_stamp_pass(recordings, tmp_path, monkeypatch) -> None:
    original = builtins.__import__

    def missing(name, *args, **kwargs):
        if name == "rosbags" or name.startswith("rosbags."):
            raise ImportError("not installed")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", missing)
    atlas = _extract(tmp_path, recordings)
    assert _passes(atlas)[0]["state"] == "refused"
    assert "stamps extra" in _passes(atlas)[0]["reason"]
    assert any(
        row.rule == "extract.pass_incomplete" and row.severity == "error" for row in atlas.findings
    )
    assert ids.build_bagtopic("broken", "/tai") in atlas.entities


@pytest.mark.parametrize("empty_root", [False, True])
def test_absent_inputs_never_touch_the_reader(tmp_path, monkeypatch, empty_root) -> None:
    def unexpected():
        pytest.fail("the optional reader was reached without recordings")

    monkeypatch.setattr(stamps, "require_reader", unexpected)
    atlas = _extract(tmp_path, tmp_path if empty_root else None)
    assert _passes(atlas)[0]["state"] == "not_attempted"
    assert not [row for row in atlas.findings if row.rule == "extract.pass_incomplete"]


def test_reader_notices_are_portable_run_exceptions(recordings, tmp_path, monkeypatch) -> None:
    read = stamps.read_topic_series_report

    def degraded(path):
        report = read(path)
        report.notices.append(f"degraded read of {path}/storage.db3")
        return report

    monkeypatch.setattr(stamps, "read_topic_series_report", degraded)
    atlas = _extract(tmp_path, recordings)
    problems = extract.exceptions(atlas)
    assert any(row.get("exc_type") == "ReaderNotice" for row in problems)
    assert str(tmp_path) not in atlas.canonical_bytes().decode()
    assert all(
        "degraded read" not in str(fact.values)
        for entity in atlas.entities.values()
        for fact in entity.facts
    )


@pytest.mark.parametrize(
    "rule,check,key,severity,expected",
    [
        ("time.backward", "monotonic", "largest_backward_jump", "error", "error"),
        ("time.epoch_offset", "epoch_offset", "median_offset_s", "warning", "warn"),
        ("time.topic_skew", "inter_topic_skew", "delta_s", "warning", "warn"),
        ("time.topic_skew", "inter_topic_skew", "delta_s", "error", "error"),
        ("time.frozen", "frozen_stamps", "receive_span_s", "error", "error"),
        ("time.zero", "zero_stamps", "count", "error", "error"),
        ("time.gap", "stamp_gaps", "max_gap_s", "warning", "warn"),
        ("time.drift", "clock_drift", "slope_s_per_s", "warning", "warn"),
        ("time.drift", "clock_drift", "slope_s_per_s", "error", "error"),
    ],
)
@pytest.mark.parametrize("layer", ["recorded", "observed"])
def test_each_rule_and_variant(rule, check, key, severity, expected, layer) -> None:
    builder = model.AtlasBuilder()
    subject = (
        ids.build_topic("/scan") if layer == "observed" else ids.build_bagtopic("run", "/scan")
    )
    fact = builder.fact(
        subject,
        layer,
        values={
            "stamps": {
                "checks": [
                    {
                        "check": check,
                        "severity": severity,
                        "message": "measured stamp failure",
                        "evidence": {key: 2.0},
                    }
                ]
            }
        },
    )
    if layer == "observed":
        fact.values = {"volatile": fact.values}
    atlas = builder.build()
    found = rules.findings_for(atlas, only=[rule])
    assert len(found) == 1
    assert found[0].subject == subject
    assert found[0].severity == expected
    assert found[0].fix
    fact.confidence = "unknown"
    assert rules.findings_for(atlas, only=[rule]) == []
    fact.confidence = "resolved"
    values = fact.values.get("volatile", fact.values)
    values["stamps"]["checks"] = []
    assert rules.findings_for(atlas, only=[rule]) == []


def test_unreadable_stamps_leave_a_refusal_and_continue(recordings, tmp_path, monkeypatch):
    read = stamps.read_topic_series_report

    def broken(path):
        if Path(path).name == "broken":
            raise ValueError(f"cannot read payload at {path}/broken.db3")
        return read(path)

    monkeypatch.setattr(stamps, "read_topic_series_report", broken)
    atlas = _extract(tmp_path, recordings)
    refused = [
        row
        for row in atlas.entities.values()
        if row.attrs.get("refused_because") == "stamps_unreadable"
    ]
    assert len(refused) == 1
    assert refused[0].facts == []
    assert refused[0].unresolved
    assert any(
        row.rule == "extract.input_refused" and row.subject == refused[0].id
        for row in atlas.findings
    )
    assert _passes(atlas)[0]["state"] == "completed"
    assert (
        atlas.entities[ids.build_bagtopic("clean", "/scan")]
        .fact("recorded")
        .values["stamps"]["messages"]
        == 40
    )
    assert str(tmp_path) not in atlas.canonical_bytes().decode()


def test_stamp_precision_and_dependency_range():
    assert stamps.rounded({"offset_s": 1.1234567894, "rate_hz": 2.123456789}) == {
        "offset_s": 1.123456789,
        "rate_hz": 2.123457,
    }
    packaging = Path(__file__).parents[1] / "pyproject.toml"
    assert f'stamps = ["{stamps.STAMPS_EXTRA}"]' in packaging.read_text(encoding="utf-8")


def test_independent_nested_facts_extend_without_overwriting():
    from rosio.errors import ContractError

    builder = model.AtlasBuilder()
    subject = ids.build_bagtopic("run", "/scan")
    builder.fact(subject, "recorded", values={"volatile": {"message_count": 20}})
    fact = builder.fact(subject, "recorded", values={"volatile": {"stamps": {"rate_hz": 10.0}}})
    assert fact.values["volatile"] == {"message_count": 20, "stamps": {"rate_hz": 10.0}}
    with pytest.raises(ContractError):
        builder.fact(subject, "recorded", values={"volatile": {"stamps": {"rate_hz": 11.0}}})


def test_the_stamps_tool_answers_from_the_atlas(recordings: Path, tmp_path: Path) -> None:
    from rosio.serve import tools

    atlas = _extract(tmp_path, recordings)
    answer = tools.stamps(atlas)
    assert answer.startswith("2 recordings with header stamp checks")
    assert "broken: error" in answer
    assert "/backward" in answer and "/tai" in answer
    assert len(answer.encode()) <= tools.MAX_BYTES
    assert "/backward" not in tools.stamps(atlas, bag="clean")
    with pytest.raises(UsageError, match="no recording named absent in this atlas"):
        tools.stamps(atlas, bag="absent")
    (tmp_path / "none").mkdir()
    assert tools.stamps(_extract(tmp_path / "none", None)).startswith("no recording in this atlas")


def test_a_recorded_offset_title_prints_seconds_to_the_millisecond() -> None:
    builder = model.AtlasBuilder()
    subject = ids.build_bagtopic("run", "/scan")
    check = {
        "check": "epoch_offset",
        "severity": "warning",
        "message": "measured stamp failure",
        "evidence": {"median_offset_s": -36.979999781},
    }
    builder.fact(subject, "recorded", values={"stamps": {"checks": [check]}})
    (found,) = rules.findings_for(builder.build(), only=["time.epoch_offset"])
    assert found.title == "/scan: stamp epoch offset of -36.980 s"
    assert "-36.980 s" in found.say


def test_skew_takes_one_vote_per_publishing_node() -> None:
    builder = model.AtlasBuilder()
    for topic, offset in (("/image", 0.4), ("/info", 0.4), ("/joint_states", 0.0)):
        builder.fact(
            ids.build_bagtopic("run", topic),
            "recorded",
            values={"stamps": {"offset_s": offset, "checks": []}},
        )
    for topic, node in (("/image", "/camera"), ("/info", "/camera"), ("/joint_states", "/arm")):
        builder.node(ids.build_node(node))
        builder.node(ids.build_topic(topic))
        builder.edge(
            "publishes", ids.build_node(node), ids.build_topic(topic), key="k", layer="coded"
        )
    builder.node(ids.build_node("/imu"))
    builder.fact(
        ids.build_bagtopic("run", "/imu"), "recorded", values={"stamps": {"offset_s": 0.0}}
    )
    found = rules.findings_for(builder.build(), only=["time.topic_skew"])
    assert sorted(item.subject for item in found) == [
        ids.build_bagtopic("run", "/image"),
        ids.build_bagtopic("run", "/info"),
    ]
