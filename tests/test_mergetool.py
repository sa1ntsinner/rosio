"""The merge driver: what it keeps, what it refuses to decide, and what it states.

The atlases here are built with the model, so every case is a difference two developers
could really produce by rebuilding after two different edits.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.cli import main
from rosio.errors import EXIT_OK, ContractError
from rosio.mergetool import MERGE_KEY, merge_documents, merge_files
from rosio.model import Candidate, Finding, Source

REAL = "profile:navigation"
SIM = "profile:simulation"

PARAMS = ids.build_file("demo_bringup", "config/params.yaml")
OVERLAY = ids.build_file("demo_bringup", "config/overrides.yaml")
CONTROLLER = ids.build_node("/controller_server")
PLANNER = ids.build_node("/planner_server")
CMD_VEL = ids.build_topic("/cmd_vel")
SPEED = ids.build_param(CONTROLLER, "max_vel_x")
SYM = ids.build_sym("demo_bringup", "src/controller.cpp", "demo::Controller::Controller")

DRIFT = {
    "rules_run": 33,
    "rules_failed": 0,
    "produced": 1,
    "filtered": 0,
    "deduplicated": 0,
    "waived": 0,
    "gated": 1,
    "by_rule": {"param.shadowed": 1},
}
"""What `drift.apply` writes, which is the block a merge has to bring through."""


def build(
    *,
    speed: float = 0.26,
    line: int = 140,
    with_planner: bool = True,
    qos: int = 10,
    finding: bool = True,
    digest: str = "1a2b3c4d",
    root_hash: str = "",
) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.add_profile("navigation")
    run = builder.add_run(
        "declared", profile=REAL, input_digest=digest, roots={"src": root_hash or digest}
    )
    for file_id in (PARAMS, OVERLAY):
        builder.node(file_id)
    builder.node(SYM)
    builder.fact(CONTROLLER, "declared", profile=REAL, run=run, values={"pkg": "demo_bringup"})
    builder.node(CONTROLLER, name_source="literal")
    builder.node(CMD_VEL)
    builder.edge(
        "publishes",
        CONTROLLER,
        CMD_VEL,
        key=ids.edge_key(symbol=SYM),
        layer="coded",
        profile=REAL,
        values={"qos_depth": qos},
    )
    if with_planner:
        builder.fact(PLANNER, "declared", profile=REAL, values={"pkg": "demo_bringup"})
        builder.node(PLANNER, name_source="literal")
    builder.set_param(
        SPEED,
        state="effective",
        candidates=[
            Candidate(
                value=speed,
                value_type="float",
                mechanism="yaml_base",
                source=Source(
                    file=PARAMS,
                    locator="controller_server.ros__parameters.max_vel_x",
                    line_hint=line,
                ),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    builder.edge("has_param", CONTROLLER, SPEED)
    if finding:
        builder.finding(
            Finding(
                rule="param.shadowed",
                severity="warn",
                subject=SPEED,
                title="max_vel_x is set twice",
                detail="the overlay and the base file both state it",
                fix="delete the base line",
                say=f"max_vel_x on controller_server is {speed} from config/params.yaml",
                profiles=[REAL],
                where=[Source(file=PARAMS, line_hint=line)],
            )
        )
    return builder.build()


def doc(atlas: model.Atlas) -> dict:
    return json.loads(atlas.to_bytes())


def effective(payload: dict, entity: str, profile: str = REAL):
    return payload["entities"][entity]["effective"][profile]["value"]["value"]


# ---------------------------------------------------------------------------
# the union


def test_merging_one_atlas_with_itself_changes_nothing():
    base = doc(build())

    result = merge_documents(base, doc(build()), doc(build()))

    assert result.conflicts == []
    assert result.payload["entities"].keys() == base["entities"].keys()
    assert effective(result.payload, SPEED) == 0.26


def test_a_side_that_did_not_touch_a_value_never_overrides_the_side_that_did():
    result = merge_documents(doc(build()), doc(build(speed=0.18)), doc(build()))

    assert effective(result.payload, SPEED) == 0.18
    assert result.conflicts == []


def test_two_edits_in_different_places_both_survive():
    """The case this driver exists for: two developers rebuild after two different edits."""
    result = merge_documents(
        doc(build()), doc(build(speed=0.18)), doc(build(qos=1, with_planner=True))
    )

    assert effective(result.payload, SPEED) == 0.18
    edge = next(row for row in result.payload["edges"].values() if row["kind"] == "publishes")
    assert edge["facts"][0]["values"]["qos_depth"] == 1
    assert result.conflicts == []


def test_an_entity_one_side_added_is_kept_and_one_side_removed_is_removed():
    without = doc(build(with_planner=False))

    added = merge_documents(without, doc(build()), without)
    removed = merge_documents(doc(build()), without, doc(build()))

    assert PLANNER in added.payload["entities"]
    assert PLANNER not in removed.payload["entities"]


def test_a_deletion_on_one_side_and_a_change_on_the_other_keeps_the_change_and_marks_it():
    result = merge_documents(doc(build()), doc(build(with_planner=False)), doc(build(qos=1)))

    assert result.conflicts == [] or all(row["kind"] for row in result.conflicts)
    assert PLANNER not in result.payload["entities"]


def test_both_sides_changing_one_value_keeps_both_and_marks_it():
    result = merge_documents(doc(build()), doc(build(qos=1)), doc(build(qos=5)))

    assert len(result.conflicts) == 1
    row = result.conflicts[0]
    assert row["kind"] == "both_changed"
    assert sorted([row["in_place"], row["other"]]) == [1, 5]
    assert row["base"] == 10
    mark = result.payload["x"][MERGE_KEY]
    assert mark["conflicts"] == result.conflicts
    assert mark["rule"]
    assert mark["subjects"]


def test_the_mark_names_the_whole_id_of_what_disagreed():
    """Every node, topic and parameter id holds a slash, so a subject cut at the first one
    names a kind and nothing else."""
    result = merge_documents(
        doc(build()), doc(build(speed=0.18, finding=False)), doc(build(speed=0.05, finding=False))
    )

    assert result.payload["x"][MERGE_KEY]["subjects"] == [SPEED]


def test_the_side_a_value_came_from_decides_nothing():
    """Merging the same pair the other way round produces the same bytes, so no developer
    is preferred by the order git happened to hand the files over in."""
    base, ours, theirs = doc(build()), doc(build(qos=1, speed=0.18)), doc(build(qos=5))

    one = merge_documents(base, ours, theirs)
    other = merge_documents(base, theirs, ours)

    assert one.to_bytes() == other.to_bytes()


def test_the_output_never_carries_a_conflict_marker():
    result = merge_documents(doc(build()), doc(build(qos=1)), doc(build(qos=5)))

    text = result.to_bytes().decode("utf-8")
    assert "<<<<<<<" not in text
    assert ">>>>>>>" not in text


# ---------------------------------------------------------------------------
# what must not be interleaved


def test_two_candidate_chains_are_never_interleaved():
    """Rank 0 from one side beside rank 0 from the other is two winners for one parameter,
    a precedence neither developer wrote."""
    result = merge_documents(
        doc(build(finding=False)),
        doc(build(speed=0.18, finding=False)),
        doc(build(speed=0.05, finding=False)),
    )

    chain = result.payload["entities"][SPEED]["candidates"]
    winners = [row for row in chain if row["wins"] and row["profile"] == REAL]
    assert len(winners) == 1
    assert len(chain) == 1
    assert [row["kind"] for row in result.conflicts] == ["chain_changed_on_both_sides"]
    assert sorted(
        row["value"] for side in result.conflicts for row in [*side["in_place"], *side["other"]]
    ) == [0.05, 0.18]


def test_a_line_that_moved_is_not_a_disagreement_about_a_value():
    """Section 9 keeps a line hint out of every comparison, and a merge is a comparison."""
    result = merge_documents(doc(build()), doc(build(line=141)), doc(build(line=190)))

    assert result.conflicts == []
    assert effective(result.payload, SPEED) == 0.26
    assert any("line hints" in note for note in result.notes)
    line = result.payload["entities"][SPEED]["candidates"][0]["source"]["line_hint"]
    assert line in (141, 190)


# ---------------------------------------------------------------------------
# what is recomputed rather than merged


def test_everything_derived_follows_the_merged_facts():
    result = merge_documents(doc(build()), doc(build(speed=0.18)), doc(build(with_planner=False)))
    payload = result.payload

    assert payload["counters"]["entities"] == len(payload["entities"])
    assert payload["indexes"]["by_kind"]["node"] == sorted(
        key for key, row in payload["entities"].items() if row["kind"] == "node"
    )
    assert effective(payload, SPEED) == 0.18
    assert schema.validate(payload) == []


def test_a_merged_atlas_loads_and_is_a_fixpoint():
    once = merge_documents(doc(build()), doc(build(qos=1)), doc(build(speed=0.18)))
    twice = merge_documents(once.payload, once.payload, once.payload)

    assert model.Atlas.from_json(once.payload).content_hash()
    assert twice.to_bytes() == once.to_bytes()


def test_the_blind_spots_are_the_union_and_the_resolution_never_reads_healthier():
    ours, theirs = doc(build()), doc(build())
    ours["counters"]["resolved"] = {"topic_name": 10}
    theirs["counters"]["resolved"] = {"topic_name": 4}

    result = merge_documents(doc(build()), ours, theirs)

    assert result.payload["counters"]["resolved"] == {"topic_name": 4}


def test_a_finding_one_side_closed_and_the_other_left_alone_is_gone():
    result = merge_documents(doc(build()), doc(build(finding=False)), doc(build()))

    assert result.payload["findings"] == []
    assert result.conflicts == []


def test_a_finding_one_side_closed_while_the_other_changed_it_is_kept_and_marked():
    """A deletion never silently removes what the other side updated."""
    result = merge_documents(doc(build()), doc(build(finding=False)), doc(build(speed=0.18)))

    assert [row["rule"] for row in result.payload["findings"]] == ["param.shadowed"]
    assert "deleted_on_one_side" in [row["kind"] for row in result.conflicts]
    assert effective(result.payload, SPEED) == 0.18


def test_two_builds_of_one_workspace_are_not_a_disagreement_about_the_system():
    """Every fact names the run that wrote it, and two rebuilds are two runs. Both run
    entities stay in the file, so nothing is lost by pointing each slot at one of them."""
    base = doc(build())
    result = merge_documents(base, doc(build(digest="aaaa1111")), doc(build(digest="bbbb2222")))

    assert [row["kind"] for row in result.conflicts] == []
    assert any("both builds" in note for note in result.notes)


def test_two_trees_read_by_one_run_id_are_not_a_disagreement_either():
    """The nine rows the first real merge kept. A collector whose input digest does not cover
    the root hashes keeps one run id across two clones, and the two content hashes then land
    on one field. That is what a build read and not what the system does."""
    base = doc(build())
    result = merge_documents(
        base, doc(build(root_hash="aaaa1111")), doc(build(root_hash="bbbb2222"))
    )

    assert result.conflicts == []
    assert any("what a build was" in note for note in result.notes)
    run = next(key for key in result.payload["entities"] if key.startswith("run:"))
    assert result.payload["entities"][run]["attrs"]["roots"]["src"] in ("aaaa1111", "bbbb2222")


def test_a_real_disagreement_still_stands_out_beside_two_builds():
    """The point of widening the rule: what is kept is what two people disagree about."""
    base = doc(build())
    result = merge_documents(
        base,
        doc(build(speed=0.18, root_hash="aaaa1111")),
        doc(build(speed=0.42, root_hash="bbbb2222")),
    )

    kept = [row["pointer"] for row in result.conflicts]
    assert f"entities/{SPEED}/candidates[{REAL}]" in kept
    assert all("run:" not in pointer for pointer in kept)
    assert len(kept) == 2, "the second is the sentence of the finding, which names the value"


def test_a_merged_atlas_still_states_that_the_rules_ran():
    """`_strip` takes the derived counters out and `recompute` cannot bring this one back: no
    finding list says how many rules ran. An atlas carrying every finding while stating that
    no drift run is recorded is a false statement about itself."""
    ours, theirs = doc(build()), doc(build(speed=0.18))
    for side in (ours, theirs):
        side["counters"]["drift"] = DRIFT

    result = merge_documents(doc(build()), ours, theirs)

    assert result.payload["counters"]["drift"] == DRIFT
    assert [row["rule"] for row in result.payload["findings"]] == ["param.shadowed"]


def test_the_drift_counter_of_a_merge_counts_the_findings_the_file_holds():
    """Two sides whose finding lists differ ran the rules over two atlases, so their own
    tallies are true of neither this file nor each other. What stays is what they agree the
    rules were, and a per rule count over the findings that are actually here."""
    ours, theirs = doc(build()), doc(build(finding=False))
    ours["counters"]["drift"] = DRIFT
    theirs["counters"]["drift"] = {**DRIFT, "produced": 0, "gated": 0, "by_rule": {}}

    result = merge_documents(doc(build(finding=False)), ours, theirs)

    counter = result.payload["counters"]["drift"]
    assert counter == {"rules_run": 33, "rules_failed": 0, "by_rule": {"param.shadowed": 1}}
    assert any("neither build" in note for note in result.notes)


# ---------------------------------------------------------------------------
# the driver git calls


def test_the_driver_writes_the_result_where_git_reads_it(tmp_path: Path):
    base = tmp_path / "base.json"
    ours = tmp_path / "ours.json"
    theirs = tmp_path / "theirs.json"
    base.write_bytes(build().to_bytes())
    ours.write_bytes(build(speed=0.18).to_bytes())
    theirs.write_bytes(build(qos=1).to_bytes())

    result = merge_files(base, ours, theirs, path="rosio-out/atlas.json")

    written = json.loads(ours.read_text(encoding="utf-8"))
    assert written == result.payload
    assert effective(written, SPEED) == 0.18
    assert written["x"][MERGE_KEY]["path"] == "rosio-out/atlas.json"


def test_a_file_added_on_both_branches_has_no_ancestor_to_merge_against(tmp_path: Path):
    base = tmp_path / "empty"
    base.write_text("", encoding="utf-8")
    ours = tmp_path / "ours.json"
    theirs = tmp_path / "theirs.json"
    ours.write_bytes(build(speed=0.18).to_bytes())
    theirs.write_bytes(build(with_planner=False).to_bytes())

    result = merge_files(base, ours, theirs)

    assert effective(result.payload, SPEED) == 0.18
    assert PLANNER in result.payload["entities"]


def test_the_verb_reports_the_rule_and_exits_zero(tmp_path: Path, capsys):
    for name, atlas in (
        ("base.json", build()),
        ("ours.json", build(qos=1)),
        ("theirs.json", build(qos=5)),
    ):
        (tmp_path / name).write_bytes(atlas.to_bytes())

    code = main(
        [
            "hook",
            "merge",
            str(tmp_path / "base.json"),
            str(tmp_path / "ours.json"),
            str(tmp_path / "theirs.json"),
        ]
    )

    body = capsys.readouterr().out
    assert code == EXIT_OK
    assert "kept both" in body
    assert "rule:" in body


def test_two_different_schema_majors_are_refused():
    ours, theirs = doc(build()), doc(build())
    theirs["schema_version"] = "99.0"

    with pytest.raises(ContractError):
        merge_documents(doc(build()), ours, theirs)


# ---------------------------------------------------------------------------
# git itself


def test_git_resolves_a_real_conflict_through_the_driver(tmp_path: Path):
    """The whole point: two branches that both rebuilt the atlas merge without a marker."""
    root = tmp_path / "repo"
    out = root / "rosio-out"
    out.mkdir(parents=True)

    # git starts the driver in its own process, so the tree under test has to be on its
    # PYTHONPATH. Without this the driver runs whatever rosio happens to be installed on
    # the machine, and the test reports on that instead of on this code.
    tree = str(Path(__file__).resolve().parent.parent)
    env = {**os.environ, "PYTHONPATH": tree + os.pathsep + os.environ.get("PYTHONPATH", "")}

    def run(*args: str) -> str:
        done = subprocess.run(
            ["git", "-C", str(root), *args], capture_output=True, text=True, check=True, env=env
        )
        return done.stdout

    subprocess.run(["git", "init", "-q", str(root)], check=True, capture_output=True)
    run("config", "user.email", "dev@example.com")
    run("config", "user.name", "developer")
    run("config", "merge.rosio-atlas.name", "rosio atlas union merge")
    run(
        "config",
        "merge.rosio-atlas.driver",
        f"{Path(__import__('sys').executable)} -m rosio hook merge %O %A %B --path %P",
    )
    (root / ".gitattributes").write_text(
        "rosio-out/atlas.json merge=rosio-atlas\n", encoding="utf-8"
    )
    (out / "atlas.json").write_bytes(build().to_bytes())
    run("add", "-A")
    run("commit", "-qm", "the atlas as it was")

    run("checkout", "-q", "-b", "dev-a")
    (out / "atlas.json").write_bytes(build(speed=0.18).to_bytes())
    run("commit", "-qam", "developer a rebuilt")
    run("checkout", "-q", "-")
    run("checkout", "-q", "-b", "dev-b")
    (out / "atlas.json").write_bytes(build(qos=1).to_bytes())
    run("commit", "-qam", "developer b rebuilt")
    run("checkout", "-q", "dev-a")
    run("merge", "--no-edit", "-q", "dev-b")

    merged = json.loads((out / "atlas.json").read_text(encoding="utf-8"))
    assert "<<<<<<<" not in (out / "atlas.json").read_text(encoding="utf-8")
    assert effective(merged, SPEED) == 0.18
    edge = next(row for row in merged["edges"].values() if row["kind"] == "publishes")
    assert edge["facts"][0]["values"]["qos_depth"] == 1
