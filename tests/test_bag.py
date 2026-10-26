"""The recorded layer: rosbag2 metadata, per topic statistics, and a killed recording."""

from __future__ import annotations

import json
import sqlite3
import tracemalloc
from pathlib import Path
from typing import Any

import pytest
import yaml

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import bag as bag_collect
from rosio.config import Config, RootSpec
from rosio.errors import CollectorError
from rosio.workspace import Workspace

QOS = (
    "- history: 1\n"
    "  depth: 10\n"
    "  reliability: 1\n"
    "  durability: 2\n"
    "  deadline:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  lifespan:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  avoid_ros_namespace_conventions: false"
)

TOPICS = (
    ("/scan", "sensor_msgs/msg/LaserScan", QOS),
    ("/imu", "sensor_msgs/msg/Imu", ""),
)

START = 1750000000000000000
STEP = 10000000


# ---------------------------------------------------------------------------
# fixtures written by hand, so no ROS and no recorder are needed


def _write_storage(path: Path, counts: dict[str, int], payload: int = 64) -> None:
    connection = sqlite3.connect(str(path))
    connection.executescript(
        """
        CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
        CREATE TABLE topics(
          id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
          serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(
          id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL,
          data BLOB NOT NULL);
        """
    )
    connection.execute("INSERT INTO schema VALUES (3, 'humble')")
    for index, (name, type_name, qos) in enumerate(TOPICS, start=1):
        connection.execute(
            "INSERT INTO topics VALUES (?, ?, ?, 'cdr', ?)", (index, name, type_name, qos)
        )
    blob = b"\x00" * payload
    for index, (name, _type, _qos) in enumerate(TOPICS, start=1):
        for step in range(counts.get(name, 0)):
            connection.execute(
                "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)",
                (index, START + step * STEP, blob),
            )
    connection.commit()
    connection.close()


def _write_metadata(directory: Path, counts: dict[str, int], relative: str) -> None:
    total = sum(counts.values())
    longest = max(counts.values()) - 1
    info: dict[str, Any] = {
        "version": 5,
        "storage_identifier": "sqlite3",
        "relative_file_paths": [relative],
        "duration": {"nanoseconds": longest * STEP},
        "starting_time": {"nanoseconds_since_epoch": START},
        "message_count": total,
        # rosbag2_py refuses a metadata document without this block, so the fixture carries
        # it and the suite reads the same way with ROS sourced and without.
        "files": [
            {
                "path": relative,
                "message_count": total,
                "duration": {"nanoseconds": longest * STEP},
                "starting_time": {"nanoseconds_since_epoch": START},
            }
        ],
        "compression_format": "",
        "compression_mode": "",
        "ros_distro": "humble",
        "topics_with_message_count": [
            {
                "message_count": counts.get(name, 0),
                "topic_metadata": {
                    "name": name,
                    "type": type_name,
                    "serialization_format": "cdr",
                    "offered_qos_profiles": qos,
                },
            }
            for name, type_name, qos in TOPICS
        ],
    }
    text = yaml.safe_dump({"rosbag2_bagfile_information": info}, sort_keys=True)
    (directory / bag_collect.METADATA_NAME).write_text(text, encoding="utf-8")


@pytest.fixture
def whole_bag(tmp_path: Path) -> Path:
    directory = tmp_path / "yard_whole"
    directory.mkdir()
    counts = {"/scan": 12, "/imu": 30}
    _write_storage(directory / "yard_whole_0.db3", counts)
    _write_metadata(directory, counts, "yard_whole_0.db3")
    return directory


@pytest.fixture
def headless_bag(tmp_path: Path) -> Path:
    """A recording with a storage file and no metadata, the killed recorder case."""
    directory = tmp_path / "yard_headless"
    directory.mkdir()
    _write_storage(directory / "yard_headless_0.db3", {"/scan": 12, "/imu": 30})
    return directory


@pytest.fixture
def truncated_bag(tmp_path: Path) -> Path:
    """A storage file whose header claims more pages than the file holds."""
    directory = tmp_path / "yard_cut"
    directory.mkdir()
    target = directory / "yard_cut_0.db3"
    _write_storage(target, {"/scan": 4000, "/imu": 4000}, payload=512)
    size = target.stat().st_size
    with target.open("r+b") as handle:
        handle.truncate(size // 2 // 4096 * 4096)
    return directory


def _atlas(directory: Path, tmp_path: Path) -> model.Atlas:
    source = tmp_path / "src"
    source.mkdir(exist_ok=True)
    ws = Workspace.discover([RootSpec("src", str(source))])
    cfg = Config(roots=(RootSpec("bag", str(directory)),), profiles=())
    builder = model.AtlasBuilder()
    bag_collect.BagCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    return builder.build()


# ---------------------------------------------------------------------------
# reading


def test_a_whole_bag_states_its_metadata_and_its_statistics(whole_bag: Path) -> None:
    read = bag_collect.read_bag(str(whole_bag))
    assert read.metadata_present is True
    assert read.storage_id == "sqlite3"
    assert read.missing == []
    assert read.statistics_path == "sqlite3"
    assert read.complete is True
    assert [topic.name for topic in read.topics] == ["/imu", "/scan"]
    scan = read.topics[1]
    assert scan.type_name == "sensor_msgs/msg/LaserScan"
    assert scan.serialization_format == "cdr"
    assert scan.count == 12
    assert scan.first_ns == START
    assert scan.span_ns == 11 * STEP
    assert scan.qos_source == "metadata"
    assert scan.qos[0]["depth"] == 10


def test_the_message_count_of_a_topic_matches_the_metadata(whole_bag: Path) -> None:
    read = bag_collect.read_bag(str(whole_bag))
    for topic in read.topics:
        assert topic.count == topic.stated_count


def test_a_bag_with_no_metadata_recovers_the_topics_and_names_the_cost(
    headless_bag: Path,
) -> None:
    read = bag_collect.read_bag(str(headless_bag))
    assert read.metadata_present is False
    assert read.metadata_path == "none"
    assert read.statistics_path == "sqlite3"
    assert read.missing == list(bag_collect.METADATA_CARRIES)
    assert "storage_identifier" in read.missing
    assert [topic.name for topic in read.topics] == ["/imu", "/scan"]
    assert read.topics[1].type_name == "sensor_msgs/msg/LaserScan"
    assert read.topics[1].qos_source == "storage_topics"
    assert read.ros_distro == "humble"
    assert read.ros_distro_source == "sqlite_schema"


def test_a_bag_with_no_metadata_never_states_the_metadata_fields(headless_bag: Path) -> None:
    values = bag_collect.read_bag(str(headless_bag)).values()
    assert "storage_id" not in values
    assert "metadata_version" not in values
    assert "compression_format" not in values
    assert values["storage_format"] == "sqlite3"
    assert values["storage_format_source"] == "file_magic"


def test_a_metadata_file_no_reader_parses_costs_the_metadata_and_not_the_recording(
    whole_bag: Path, tmp_path: Path
) -> None:
    """A recorder killed while it writes the metadata leaves a file that parses as nothing,
    and the storage next to it is still the run that happened."""
    (whole_bag / bag_collect.METADATA_NAME).write_text("[unterminated\n", encoding="utf-8")
    read = bag_collect.read_bag(str(whole_bag))
    assert read.metadata_present is False
    assert read.metadata_path == "none"
    assert read.reader_refusal[0] != ""
    assert read.missing == list(bag_collect.METADATA_CARRIES)
    assert [topic.name for topic in read.topics] == ["/imu", "/scan"]
    assert {topic.name: topic.count for topic in read.topics} == {"/imu": 30, "/scan": 12}
    atlas = _atlas(whole_bag, tmp_path)
    assert ids.build_bagtopic("yard_whole", "/scan") in atlas.entities
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["phase"] for row in run.attrs["exceptions"]] == ["metadata"]
    assert schema.validate(atlas.to_json()) == []


def test_a_truncated_storage_file_is_read_page_by_page(truncated_bag: Path) -> None:
    with pytest.raises(sqlite3.DatabaseError):
        connection = sqlite3.connect(
            f"{(truncated_bag / 'yard_cut_0.db3').as_uri()}?mode=ro", uri=True
        )
        try:
            connection.execute("select count(*) from messages").fetchall()
        finally:
            connection.close()
    read = bag_collect.read_bag(str(truncated_bag))
    assert read.statistics_path == "sqlite_pages"
    assert read.complete is False
    assert read.unreadable_pages > 0
    assert [topic.name for topic in read.topics] == ["/imu", "/scan"]
    assert read.topics[1].type_name == "sensor_msgs/msg/LaserScan"
    counted = sum(topic.count or 0 for topic in read.topics)
    assert 0 < counted < 8000


def test_a_truncated_bag_says_its_counts_are_a_floor(truncated_bag: Path) -> None:
    read = bag_collect.read_bag(str(truncated_bag))
    values = read.values()
    assert values["complete"] is False
    assert values["volatile"]["unreadable_pages"] > 0
    assert any("floor" in note for note in values["notes"])
    for topic in read.topics:
        assert topic.values()["volatile"]["complete"] is False


def test_the_page_reader_and_sqlite_agree_on_a_whole_bag(whole_bag: Path) -> None:
    target = str(whole_bag / "yard_whole_0.db3")
    through_sqlite = bag_collect._read_sqlite3(target)
    through_pages, gaps = bag_collect._read_pages(target)
    assert gaps == 0
    assert through_pages.counts == through_sqlite.counts
    assert through_pages.topics == through_sqlite.topics
    assert through_pages.ros_distro == through_sqlite.ros_distro == "humble"


def test_a_message_payload_is_never_read(tmp_path: Path) -> None:
    """A bag of large payloads costs the b tree pages and not the messages."""
    directory = tmp_path / "yard_fat"
    directory.mkdir()
    _write_storage(directory / "yard_fat_0.db3", {"/scan": 40, "/imu": 40}, payload=200_000)
    assert (directory / "yard_fat_0.db3").stat().st_size > 8_000_000
    tracemalloc.start()
    try:
        read = bag_collect.read_bag(str(directory))
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert sum(topic.count or 0 for topic in read.topics) == 80
    assert peak < 2_000_000


def test_a_directory_that_holds_no_storage_file_is_refused(tmp_path: Path) -> None:
    empty = tmp_path / "not_a_bag"
    empty.mkdir()
    with pytest.raises(CollectorError):
        bag_collect.read_bag(str(empty))


def test_a_path_that_is_not_a_directory_is_refused(tmp_path: Path) -> None:
    with pytest.raises(CollectorError):
        bag_collect.read_bag(str(tmp_path / "missing"))


def test_bags_are_found_under_a_root_and_by_their_own_path(
    whole_bag: Path, headless_bag: Path
) -> None:
    root = whole_bag.parent
    assert bag_collect.find_bags(str(root)) == sorted([str(whole_bag), str(headless_bag)])
    assert bag_collect.find_bags(str(whole_bag)) == [str(whole_bag)]
    assert bag_collect.is_bag(str(whole_bag)) is True
    assert bag_collect.is_bag(str(root)) is False


def test_an_unreadable_qos_profile_is_never_guessed() -> None:
    topic = bag_collect.TopicRead(name="/scan")
    bag_collect._set_qos(topic, "- history: [1\n  depth: ]", "storage_topics")
    assert topic.qos_readable is False
    assert topic.qos == []
    assert "offered_qos_profiles" not in topic.values()


def test_a_field_the_installed_reader_lacks_comes_out_of_the_stored_file(
    whole_bag: Path,
) -> None:
    """The humble rosbag2_py metadata object has no ros_distro, and the file has one, so
    one recording must not state two different things on two machines."""
    info: dict[str, Any] = {
        "ros_distro": "",
        "version": None,
        "compression_format": "",
        "compression_mode": "",
    }
    bag_collect._backfill(str(whole_bag), info)
    assert info["ros_distro"] == "humble"
    assert info["version"] == 5
    stated = {"ros_distro": "iron", "version": 9, "compression_format": "", "compression_mode": ""}
    bag_collect._backfill(str(whole_bag), stated)
    assert stated["ros_distro"] == "iron"
    assert stated["version"] == 9


def test_qos_the_installed_reader_cannot_read_back_comes_out_of_the_stored_file(
    whole_bag: Path,
) -> None:
    """jazzy's rosbag2_py hands the offered QoS over as setter only bindings, so the same
    recording read through it and read from the file must state the same profiles."""
    info: dict[str, Any] = {
        "ros_distro": "jazzy",
        "version": 9,
        "compression_format": "",
        "compression_mode": "",
        "topics_with_message_count": [
            {
                "message_count": 12,
                "topic_metadata": {
                    "name": "/scan",
                    "offered_qos_profiles": bag_collect._qos_text([object()]),
                },
            }
        ],
    }
    bag_collect._backfill(str(whole_bag), info)
    profiles = info["topics_with_message_count"][0]["topic_metadata"]["offered_qos_profiles"]
    assert profiles == QOS
    assert bag_collect._qos_text([{"depth": 10}]) == "- depth: 10\n"


def test_the_preferred_reader_refusing_reaches_the_run_and_not_the_recording(
    whole_bag: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A reader message names the installed version and the library path, and one recording
    must not state two different things on two boxes."""

    def refuse(path: str) -> dict[str, Any]:
        raise RuntimeError(f"{path}: metadata version 9 is newer than /opt/ros/humble/lib")

    monkeypatch.setattr(bag_collect, "rosbag2_available", lambda: True)
    monkeypatch.setattr(bag_collect, "_read_metadata_rosbag2", refuse)
    read = bag_collect.read_bag(str(whole_bag))
    assert read.metadata_path == "metadata_yaml"
    assert read.reader_refusal[0] == "RuntimeError"
    assert "/opt/ros/humble/lib" not in json.dumps(read.values())
    assert not [note for note in read.notes if "rosbag2" in note]
    atlas = _atlas(whole_bag, tmp_path)
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["phase"] for row in run.attrs["exceptions"]] == ["metadata"]
    assert str(tmp_path) not in json.dumps(atlas.to_json())


def test_a_backfill_over_a_bag_with_no_stored_file_states_nothing(headless_bag: Path) -> None:
    info: dict[str, Any] = {"ros_distro": "", "version": None}
    bag_collect._backfill(str(headless_bag), info)
    assert info == {"ros_distro": "", "version": None}


# ---------------------------------------------------------------------------
# the collector


def test_the_collector_output_satisfies_the_schema(whole_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    assert schema.validate(atlas.to_json()) == []
    assert atlas.layers[bag_collect.LAYER].present is True
    assert atlas.counters["entities_by_kind"]["bag"] == 1
    assert atlas.counters["entities_by_kind"]["bagtopic"] == 2


def test_two_runs_over_one_bag_are_byte_identical(whole_bag: Path, tmp_path: Path) -> None:
    first = _atlas(whole_bag, tmp_path)
    second = _atlas(whole_bag, tmp_path)
    assert first.canonical_bytes() == second.canonical_bytes()


def test_counts_and_spans_never_enter_the_compared_bytes(whole_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    canonical = json.loads(atlas.canonical_bytes())
    text = json.dumps(canonical)
    assert "message_count" not in text
    assert "rate_hz" not in text
    assert "span_ns" not in text
    assert "sensor_msgs/msg/LaserScan" in text


def test_every_bag_id_and_every_evidence_row_stays_machine_independent(
    whole_bag: Path, tmp_path: Path
) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    payload = json.dumps(atlas.to_json())
    assert str(tmp_path) not in payload
    for entity_id in atlas.entities:
        assert "/tmp" not in entity_id
        ids.parse(entity_id)
    for node in atlas.entities.values():
        for fact in node.facts:
            for source in fact.evidence:
                assert source.root == model.Root.BAG.value
                assert ids.parse_file(source.file).package == bag_collect.BAG_PKG


def test_a_bag_topic_is_linked_to_its_topic_and_its_type(whole_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    bagtopic = ids.build_bagtopic("yard_whole", "/scan")
    kinds = {edge.kind: edge for edge in atlas.out_edges(bagtopic)}
    assert kinds[model.EdgeKind.SAMPLES.value].dst == ids.build_topic("/scan")
    assert kinds[model.EdgeKind.USES_TYPE.value].dst == ids.build_type(
        "sensor_msgs", "msg", "LaserScan"
    )
    fact = kinds[model.EdgeKind.SAMPLES.value].fact(bag_collect.LAYER)
    assert fact is not None
    assert fact.values["type"] == "sensor_msgs/msg/LaserScan"


def test_the_bag_contains_its_files_and_its_topics(whole_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    bag_id = ids.build_bag("yard_whole")
    contained = sorted(
        edge.dst for edge in atlas.out_edges(bag_id) if edge.kind == model.EdgeKind.CONTAINS.value
    )
    assert contained == sorted(
        [
            ids.build_bagtopic("yard_whole", "/imu"),
            ids.build_bagtopic("yard_whole", "/scan"),
            ids.build_file(bag_collect.BAG_PKG, "yard_whole/metadata.yaml"),
            ids.build_file(bag_collect.BAG_PKG, "yard_whole/yard_whole_0.db3"),
        ]
    )


def test_a_missing_metadata_becomes_one_unresolved_entry(
    headless_bag: Path, tmp_path: Path
) -> None:
    atlas = _atlas(headless_bag, tmp_path)
    reasons = sorted(entry.reason for entry in atlas.unresolved)
    assert reasons == ["bag_metadata_missing"]
    entry = atlas.unresolved[0]
    assert entry.owner == ids.build_bag("yard_headless")
    assert entry.id in atlas.entities[entry.owner].unresolved


def test_a_truncated_bag_reports_both_blind_spots(truncated_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(truncated_bag, tmp_path)
    reasons = sorted(entry.reason for entry in atlas.unresolved)
    assert reasons == ["bag_metadata_missing", "bag_storage_truncated"]
    assert schema.validate(atlas.to_json()) == []


def test_a_bag_states_no_profile(whole_bag: Path, tmp_path: Path) -> None:
    atlas = _atlas(whole_bag, tmp_path)
    for node in atlas.entities.values():
        for fact in node.facts:
            assert fact.profile == model.ANY_PROFILE


def test_a_workspace_with_no_bag_root_produces_no_recorded_layer(tmp_path: Path) -> None:
    source = tmp_path / "src"
    source.mkdir()
    ws = Workspace.discover([RootSpec("src", str(source))])
    builder = model.AtlasBuilder()
    cfg = Config(roots=(RootSpec("src", str(source)),), profiles=())
    bag_collect.BagCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    atlas = builder.build()
    assert atlas.layers[bag_collect.LAYER].present is False
    assert atlas.entities == {}


def test_one_unreadable_bag_does_not_empty_the_layer(whole_bag: Path, tmp_path: Path) -> None:
    """A storage file no reader recognises costs itself: the recording is still named, and
    the loss is one blind spot instead of a bag that never appears."""
    broken = whole_bag.parent / "yard_broken"
    broken.mkdir()
    (broken / "yard_broken_0.db3").write_bytes(b"not sqlite at all")
    atlas = _atlas(whole_bag.parent, tmp_path)
    assert ids.build_bag("yard_whole") in atlas.entities
    broken_id = ids.build_bag("yard_broken")
    assert broken_id in atlas.entities
    file_id = bag_collect._file_id("yard_broken", "yard_broken_0.db3")
    entry = next(item for item in atlas.unresolved if item.owner == file_id)
    assert entry.expr == "yard_broken_0.db3 of yard_broken"
    assert "sqlite" in entry.reason
    assert entry.id in atlas.entities[file_id].unresolved
    fact = atlas.entities[broken_id].facts[0]
    assert fact.values["complete"] is False
    assert fact.values["files_unreadable"] == ["yard_broken_0.db3"]
    assert fact.values["topics"] == []


def test_a_directory_that_cannot_be_listed_does_not_cost_the_bag_beside_it(
    whole_bag: Path, tmp_path: Path
) -> None:
    locked = whole_bag.parent / "locked"
    locked.mkdir()
    (locked / "locked_0.db3").write_bytes(b"")
    locked.chmod(0o000)
    try:
        atlas = _atlas(whole_bag.parent, tmp_path)
    finally:
        locked.chmod(0o700)
    assert ids.build_bag("yard_whole") in atlas.entities
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["file"] for row in run.attrs["exceptions"]] == [ids.build_bag("locked")]
    assert str(tmp_path) not in json.dumps(atlas.to_json())


def test_the_input_digest_covers_the_reader_the_machine_has(
    whole_bag: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A box with rosbag2_py reads the metadata through it and states so, so the two boxes
    have to differ in their input and never in their answer."""
    monkeypatch.setattr(bag_collect, "rosbag2_available", lambda: False)
    without = _atlas(whole_bag, tmp_path).layers[bag_collect.LAYER].runs
    monkeypatch.setattr(bag_collect, "rosbag2_available", lambda: True)
    with_reader = _atlas(whole_bag, tmp_path).layers[bag_collect.LAYER].runs
    assert without != with_reader


def test_one_storage_path_named_twice_is_read_once(tmp_path: Path) -> None:
    directory = tmp_path / "yard_twice"
    directory.mkdir()
    counts = {"/scan": 12, "/imu": 30}
    _write_storage(directory / "yard_twice_0.db3", counts)
    _write_metadata(directory, counts, "yard_twice_0.db3")
    stored = yaml.safe_load((directory / bag_collect.METADATA_NAME).read_text())
    stored["rosbag2_bagfile_information"]["relative_file_paths"] = [
        "yard_twice_0.db3",
        "yard_twice_0.db3",
    ]
    (directory / bag_collect.METADATA_NAME).write_text(yaml.safe_dump(stored), encoding="utf-8")
    read = bag_collect.read_bag(str(directory))
    assert [item.relpath for item in read.files] == ["yard_twice_0.db3"]
    assert {topic.name: topic.count for topic in read.topics} == counts


def test_a_bag_root_that_is_gone_does_not_empty_the_layer(whole_bag: Path, tmp_path: Path) -> None:
    source = tmp_path / "src"
    source.mkdir(exist_ok=True)
    ws = Workspace.discover([RootSpec("src", str(source))])
    cfg = Config(
        roots=(
            RootSpec("bag", str(whole_bag.parent / "never_recorded")),
            RootSpec("bag", str(whole_bag)),
        ),
        profiles=(),
    )
    builder = model.AtlasBuilder()
    bag_collect.BagCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    atlas = builder.build()
    assert ids.build_bag("yard_whole") in atlas.entities
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["phase"] for row in run.attrs["exceptions"]] == ["root"]
    assert str(tmp_path) not in json.dumps(atlas.to_json())


def test_two_recordings_of_one_name_are_both_refused(whole_bag: Path, tmp_path: Path) -> None:
    """A bag id is its directory basename, so neither of two may hold it."""
    twin = whole_bag.parent / "elsewhere" / "yard_whole"
    twin.mkdir(parents=True)
    _write_storage(twin / "yard_whole_0.db3", {"/scan": 3, "/imu": 4})
    _write_metadata(twin, {"/scan": 3, "/imu": 4}, "yard_whole_0.db3")
    source = tmp_path / "src"
    source.mkdir(exist_ok=True)
    ws = Workspace.discover([RootSpec("src", str(source))])
    cfg = Config(
        roots=(RootSpec("bag", str(whole_bag)), RootSpec("bag", str(twin))),
        profiles=(),
    )
    builder = model.AtlasBuilder()
    bag_collect.BagCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    atlas = builder.build()
    assert ids.build_bag("yard_whole") not in atlas.entities
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["file"] for row in run.attrs["exceptions"]] == [ids.build_bag("yard_whole")]
    assert schema.validate(atlas.to_json()) == []


def test_a_topic_name_the_grammar_refuses_is_never_promoted(
    headless_bag: Path, tmp_path: Path
) -> None:
    connection = sqlite3.connect(str(headless_bag / "yard_headless_0.db3"))
    connection.execute("update topics set name='/scan not a name' where name='/scan'")
    connection.commit()
    connection.close()
    atlas = _atlas(headless_bag, tmp_path)
    assert ids.build_bagtopic("yard_headless", "/imu") in atlas.entities
    assert not [key for key in atlas.entities if "not a name" in key]
    entry = next(item for item in atlas.unresolved if item.reason == "topic_name_unusable")
    assert entry.owner == ids.build_bag("yard_headless")
    assert entry.expr == "/scan not a name"
    assert entry.id in atlas.entities[entry.owner].unresolved
    assert schema.validate(atlas.to_json()) == []


def test_a_storage_file_that_is_not_on_disk_is_not_called_truncated(tmp_path: Path) -> None:
    directory = tmp_path / "yard_lost"
    directory.mkdir()
    _write_metadata(directory, {"/scan": 12, "/imu": 30}, "yard_lost_0.db3")
    _write_storage(directory / "spare.db3", {"/scan": 1, "/imu": 1})
    read = bag_collect.read_bag(str(directory))
    assert read.absent is True
    assert read.truncated is False
    assert read.complete is False
    assert read.values()["files_absent"] == ["yard_lost_0.db3"]
    assert not any("past the end of the file" in note for note in read.notes)
    atlas = _atlas(directory, tmp_path)
    assert sorted(item.reason for item in atlas.unresolved) == ["bag_storage_absent"]


def test_a_row_sqlite_hands_back_with_the_wrong_type_falls_to_the_pages(tmp_path: Path) -> None:
    """sqlite opens the file and answers with a NULL timestamp, which rosbag2 cannot write."""
    directory = tmp_path / "yard_typed"
    directory.mkdir()
    target = directory / "yard_typed_0.db3"
    connection = sqlite3.connect(str(target))
    connection.executescript(
        """
        CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
        CREATE TABLE topics(
          id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
          serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(
          id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);
        """
    )
    connection.execute("INSERT INTO schema VALUES (3, 'humble')")
    connection.execute(
        "INSERT INTO topics VALUES (1, '/scan', 'sensor_msgs/msg/LaserScan', 'cdr', '')"
    )
    for _ in range(4):
        connection.execute(
            "INSERT INTO messages(topic_id, timestamp, data) VALUES (1, NULL, x'00')"
        )
    connection.commit()
    connection.close()
    read = bag_collect.read_bag(str(directory))
    assert read.statistics_path == "sqlite_pages"
    assert [topic.name for topic in read.topics] == ["/scan"]
    assert "does not hold the type it declares" in read.files[0].note


def test_a_recording_that_stated_no_qos_never_states_a_profile_list(whole_bag: Path) -> None:
    read = bag_collect.read_bag(str(whole_bag))
    imu = next(topic for topic in read.topics if topic.name == "/imu")
    assert imu.qos_raw == ""
    values = imu.values()
    assert values["qos_stated"] is False
    assert "offered_qos_profiles" not in values


# ---------------------------------------------------------------------------
# the three recordings on this machine


@pytest.fixture(params=["demo_tai", "demo_clean"])
def demo_path(request: pytest.FixtureRequest) -> Path:
    path = Path("/ssd/tmp") / str(request.param)
    if not path.is_dir():
        pytest.skip(f"recording not present at {path}")
    return path


def test_the_demo_recordings_read_whole(demo_path: Path) -> None:
    read = bag_collect.read_bag(str(demo_path))
    assert read.metadata_present is True
    assert read.complete is True
    assert read.storage_id == "sqlite3"
    assert [topic.name for topic in read.topics] == ["/gps", "/imu", "/scan"]
    assert {topic.count for topic in read.topics} == {60, 120, 600}
    for topic in read.topics:
        assert topic.count == topic.stated_count


def test_the_killed_recording_is_recovered() -> None:
    path = Path("/ssd/tmp/tmpenklwa27")
    if not path.is_dir():
        pytest.skip(f"recording not present at {path}")
    read = bag_collect.read_bag(str(path))
    assert read.metadata_present is False
    assert read.statistics_path == "sqlite_pages"
    assert read.complete is False
    assert len(read.topics) == 45
    assert read.ros_distro == "humble"
    names = {topic.name for topic in read.topics}
    assert {"/ouster/points", "/tf", "/vehicle/command"} <= names
    points = next(topic for topic in read.topics if topic.name == "/ouster/points")
    assert points.type_name == "sensor_msgs/msg/PointCloud2"
    assert points.qos[0]["depth"] == 5
    assert points.count and points.count > 0


def test_the_killed_recording_produces_a_valid_atlas(tmp_path: Path) -> None:
    path = Path("/ssd/tmp/tmpenklwa27")
    if not path.is_dir():
        pytest.skip(f"recording not present at {path}")
    atlas = _atlas(path, tmp_path)
    assert schema.validate(atlas.to_json()) == []
    assert atlas.counters["entities_by_kind"]["bagtopic"] == 45
    assert sorted(entry.reason for entry in atlas.unresolved) == [
        "bag_metadata_missing",
        "bag_storage_truncated",
    ]


def test_an_empty_storage_file_costs_itself_and_not_the_recording(
    whole_bag: Path, tmp_path: Path
) -> None:
    """A storage file of zero bytes used to end the whole recorded layer: no bag, no bag
    topics, no topics and no types, with only a run exception to say so."""
    (whole_bag / "yard_whole_0.db3").write_bytes(b"")
    atlas = _atlas(whole_bag, tmp_path)
    bag_id = ids.build_bag("yard_whole")
    assert bag_id in atlas.entities
    assert ids.build_bagtopic("yard_whole", "/scan") in atlas.entities
    assert ids.build_topic("/scan") in atlas.entities
    fact = atlas.entities[bag_id].facts[0]
    assert fact.values["complete"] is False
    assert fact.values["files_unreadable"] == ["yard_whole_0.db3"]
    assert fact.values["read"]["statistics"] == "none"
    entry = next(item for item in atlas.unresolved if item.reason.endswith("is not a sqlite file"))
    # The blind spot is on the storage file, which is the entity a reader opens after
    # seeing `loadable false`, and not on the recording around it.
    file_id = bag_collect._file_id("yard_whole", "yard_whole_0.db3")
    assert entry.owner == file_id
    assert entry.expr == "yard_whole_0.db3 of yard_whole"
    assert entry.id in atlas.entities[file_id].unresolved
    assert atlas.entities[file_id].attrs["loadable"] is False
    assert atlas.entities[file_id].attrs["refused_because"] == "bag_storage_unreadable"
    assert schema.validate(atlas.to_json()) == []


def test_a_recording_no_reader_opened_still_states_itself(whole_bag: Path, tmp_path: Path) -> None:
    """A recording read_bag refuses whole used to leave one run exception row and nothing
    else, so the atlas read exactly like a run that was never configured."""
    empty = whole_bag.parent / "yard_gone"
    empty.mkdir()
    (empty / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": {"relative_file_paths": []}}),
        encoding="utf-8",
    )
    atlas = _atlas(whole_bag.parent, tmp_path)
    assert ids.build_bag("yard_whole") in atlas.entities
    gone = atlas.entities[ids.build_bag("yard_gone")]
    assert gone.attrs["loadable"] is False
    assert gone.attrs["refused_because"] == "bag_unreadable"
    assert "holds no storage file" in gone.attrs["refused_detail"]
    entry = next(item for item in atlas.unresolved if item.owner == gone.id)
    assert entry.expr == "yard_gone"
    assert entry.id in gone.unresolved
    assert schema.validate(atlas.to_json()) == []


def test_a_storage_file_the_metadata_names_and_disk_lacks_owns_its_blind_spot(
    whole_bag: Path, tmp_path: Path
) -> None:
    """One entry per absent file, on the file, rather than one entry naming a joined list."""
    (whole_bag / "yard_whole_0.db3").unlink()
    atlas = _atlas(whole_bag, tmp_path)
    file_id = bag_collect._file_id("yard_whole", "yard_whole_0.db3")
    assert atlas.entities[file_id].attrs["loadable"] is False
    assert atlas.entities[file_id].attrs["refused_because"] == "bag_storage_absent"
    entry = next(item for item in atlas.unresolved if item.reason == "bag_storage_absent")
    assert entry.owner == file_id
    assert entry.expr == "yard_whole_0.db3 of yard_whole"
    assert entry.id in atlas.entities[file_id].unresolved


def test_metadata_that_is_a_scalar_is_a_collector_error(whole_bag: Path, tmp_path: Path) -> None:
    """The read used to raise a bare AttributeError out of the yaml document, which no
    caller catches and which hard rule 8 forbids."""
    (whole_bag / "metadata.yaml").write_text("just a scalar\n", encoding="utf-8")
    with pytest.raises(CollectorError) as raised:
        bag_collect._read_metadata_yaml(str(whole_bag))
    assert "not a mapping" in str(raised.value)
    atlas = _atlas(whole_bag, tmp_path)
    bag_id = ids.build_bag("yard_whole")
    fact = atlas.entities[bag_id].facts[0]
    assert fact.values["metadata_present"] is False
    assert sorted(fact.values["topics"]) == ["/imu", "/scan"]
    assert any(entry.reason == "bag_metadata_missing" for entry in atlas.unresolved)
    run = atlas.entities[atlas.layers[bag_collect.LAYER].runs[0]]
    assert [row["exc_type"] for row in run.attrs["exceptions"]] == ["CollectorError"]


def test_metadata_that_is_not_utf8_is_a_collector_error(whole_bag: Path) -> None:
    (whole_bag / "metadata.yaml").write_bytes(b"rosbag2_bagfile_information:\n  version: \xff\n")
    with pytest.raises(CollectorError) as raised:
        bag_collect._read_metadata_yaml(str(whole_bag))
    assert "not utf-8" in str(raised.value)
