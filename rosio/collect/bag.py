"""rosbag2 recordings: the recorded layer.

What a bag contributes is the topic set a real run carried, with the type, the
serialisation format and the offered QoS profiles the recording stored, plus the message
count and the time span per topic. Counts, spans, durations and rates are volatile and
live under `volatile`, so they never enter a conflict, an effective value or a diff.

Three rules decide the shape of this module.

* Nothing is deserialised. The metadata and the per topic statistics are the product, and
  a message payload is never read, so a 1.9 GB recording costs one pass over the b tree
  pages of the message table and a few megabytes of memory.
* Reading is layered and each layer records which path produced it. Metadata comes from
  rosbag2_py when it imports and from `metadata.yaml` when it does not. Statistics come
  from the sqlite3 module, and when sqlite refuses the file, from a stdlib reader over
  the sqlite page format.
* A recording that was killed has no `metadata.yaml` and a storage file whose header
  claims more pages than the file holds, which every sqlite client refuses to open. That
  is the case an engineer has after a bad run, so the topics, the types and the QoS come
  out of the storage tables instead, what the absent metadata would have carried is named
  in `missing`, and nothing it carried is guessed.

A bag states no profile. A recording does not know which launched composition produced it,
so every fact here holds under `*` and the join is what may bind it later.

A refusal costs its own entity and never the atlas around it. A storage file no reader
recognises is marked not loadable and carries the blind spot itself, and a recording no
reader opened at all still gets its `bag` entity with the reason, because a recording that
left nothing behind read exactly like a run that was never configured.
"""

from __future__ import annotations

import re
import sqlite3
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from rosio import ids, model
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError, ContractError

LAYER = model.Layer.RECORDED.value
NAME = "bag"

BAG_PKG = "@bag"
"""Pseudo package for files inside a recording. A bag is not an ament package and its
files are not repository files, so neither namespace can claim them."""

METADATA_NAME = "metadata.yaml"

NO_ROOT = "the configuration names no bag root, so no recording is in scope"
NO_RECORDING = "no configured bag root holds a recording"
"""What `inputs_present` says when this collector has nothing to read. Two sentences and
not one, because a run that was never given a recording and a bag root that turned out to
hold none are two different things to repair."""

SQLITE_SUFFIXES = (".db3",)
STORAGE_SUFFIXES = (".db3", ".mcap")

SQLITE_MAGIC = b"SQLite format 3\x00"

METADATA_CARRIES = (
    "compression_format",
    "compression_mode",
    "duration",
    "message_count",
    "relative_file_paths",
    "starting_time",
    "storage_identifier",
    "version",
)
"""What `metadata.yaml` states and no storage file repeats. Named one by one in `missing`
so a reader of a recovered bag sees the cost of the absent file."""

_COLUMN_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)")

_TRUNCATED = object()
"""A column the local part of a record does not reach. Never a value, never a default."""


# ---------------------------------------------------------------------------
# what one bag says


@dataclass
class TopicRead:
    """One topic of one recording."""

    name: str
    type_name: str = ""
    serialization_format: str = ""
    qos: list[Any] = field(default_factory=list)
    qos_raw: str = ""
    qos_source: str = ""
    qos_readable: bool = True
    count: int | None = None
    stated_count: int | None = None
    first_ns: int | None = None
    last_ns: int | None = None
    count_source: str = ""
    complete: bool = True

    @property
    def span_ns(self) -> int | None:
        if self.first_ns is None or self.last_ns is None:
            return None
        return self.last_ns - self.first_ns

    def values(self) -> dict[str, Any]:
        values: dict[str, Any] = {"topic": self.name}
        if self.type_name:
            values["type"] = self.type_name
        if self.serialization_format:
            values["serialization_format"] = self.serialization_format
        values["qos_stated"] = bool(self.qos_raw)
        if self.qos_source:
            values["qos_source"] = self.qos_source
        if self.qos_readable and self.qos_raw:
            values["offered_qos_profiles"] = self.qos
        volatile: dict[str, Any] = {}
        if self.count is not None:
            volatile["message_count"] = self.count
            volatile["count_source"] = self.count_source
            volatile["complete"] = self.complete
        if self.stated_count is not None and self.stated_count != self.count:
            volatile["message_count_stated"] = self.stated_count
        if self.first_ns is not None:
            volatile["first_ns"] = self.first_ns
            volatile["last_ns"] = self.last_ns
            volatile["span_ns"] = self.span_ns
            span = (self.span_ns or 0) / 1e9
            if span > 0 and self.count:
                volatile["rate_hz"] = round(self.count / span, 6)
        if volatile:
            values["volatile"] = volatile
        return values


@dataclass
class StorageRead:
    """One storage file of a recording and how it was read."""

    relpath: str
    read_path: str = "none"
    present: bool = True
    size: int = 0
    unreadable_pages: int = 0
    note: str = ""
    refusal: str = ""
    """Why the file on disk carries no statistics at all. A file that is there and holds
    nothing a reader recognises costs itself, never the recording around it."""


@dataclass
class BagRead:
    """Everything one recording could be made to say.

    A field left at None was not determined, and the reason is in `missing` or in `notes`.
    """

    name: str
    path: str
    metadata_present: bool = False
    metadata_path: str = "none"
    statistics_path: str = "none"
    storage_id: str | None = None
    storage_format: str | None = None
    metadata_version: int | None = None
    ros_distro: str | None = None
    ros_distro_source: str = ""
    compression_format: str | None = None
    compression_mode: str | None = None
    files: list[StorageRead] = field(default_factory=list)
    files_source: str = "directory"
    reader_refusal: tuple[str, str] = ("", "")
    """Why the preferred reader was left for the stored file. Which reader this machine
    holds and what it said are properties of the machine and not of the recording, so this
    reaches the run and never a compared value."""
    topics: list[TopicRead] = field(default_factory=list)
    message_count: int | None = None
    message_count_source: str = ""
    recovered_count: int | None = None
    start_ns: int | None = None
    end_ns: int | None = None
    duration_ns: int | None = None
    duration_source: str = ""
    missing: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def complete(self) -> bool:
        """False when a storage file could not be read whole, so every count is a floor."""
        return not (self.truncated or self.absent or self.unreadable)

    @property
    def unreadable(self) -> bool:
        """A storage file is on disk and no reader recognises its bytes."""
        return any(item.refusal for item in self.files)

    @property
    def truncated(self) -> bool:
        """A storage file holds pages the recorder claimed and the disk does not."""
        return any(item.unreadable_pages for item in self.files)

    @property
    def absent(self) -> bool:
        """A storage file the metadata names is not next to the metadata."""
        return any(not item.present for item in self.files)

    @property
    def unreadable_pages(self) -> int:
        return sum(item.unreadable_pages for item in self.files)

    def values(self) -> dict[str, Any]:
        values: dict[str, Any] = {
            "metadata_present": self.metadata_present,
            "read": {"metadata": self.metadata_path, "statistics": self.statistics_path},
            "files": [item.relpath for item in sorted(self.files, key=lambda row: row.relpath)],
            "files_source": self.files_source,
            "topics": sorted(topic.name for topic in self.topics),
            "complete": self.complete,
            "missing": sorted(self.missing),
        }
        if self.storage_id:
            values["storage_id"] = self.storage_id
        if self.storage_format:
            values["storage_format"] = self.storage_format
            values["storage_format_source"] = "file_magic"
        if self.metadata_version is not None:
            values["metadata_version"] = self.metadata_version
        if self.ros_distro:
            values["ros_distro"] = self.ros_distro
            values["ros_distro_source"] = self.ros_distro_source
        if self.compression_format is not None:
            values["compression_format"] = self.compression_format
        if self.compression_mode is not None:
            values["compression_mode"] = self.compression_mode
        absent = sorted(item.relpath for item in self.files if not item.present)
        if absent:
            values["files_absent"] = absent
        refused = sorted(item.relpath for item in self.files if item.refusal)
        if refused:
            values["files_unreadable"] = refused
        if self.notes:
            values["notes"] = sorted(dict.fromkeys(self.notes))
        volatile: dict[str, Any] = {"unreadable_pages": self.unreadable_pages}
        if self.message_count is not None:
            volatile["message_count"] = self.message_count
            volatile["message_count_source"] = self.message_count_source
        if self.recovered_count is not None and self.recovered_count != self.message_count:
            volatile["message_count_recovered"] = self.recovered_count
        if self.duration_ns is not None:
            volatile["duration_ns"] = self.duration_ns
            volatile["duration_source"] = self.duration_source
        if self.start_ns is not None:
            volatile["start_ns"] = self.start_ns
            volatile["end_ns"] = self.end_ns
        values["volatile"] = volatile
        return values


# ---------------------------------------------------------------------------
# discovery


def is_bag(path: str) -> bool:
    directory = Path(path)
    if not directory.is_dir():
        return False
    if (directory / METADATA_NAME).is_file():
        return True
    return any(item.suffix in STORAGE_SUFFIXES for item in directory.iterdir() if item.is_file())


def find_bags(root: str) -> list[str]:
    """The bag directories under one configured path, the path itself included."""
    return scan_bags(root)[0]


def scan_bags(root: str) -> tuple[list[str], list[tuple[str, OSError]]]:
    """The bags under one root, and the children that refused to be listed.

    A directory this process may not read is one entry in the run's exception list and
    never a reason to lose the recordings beside it.
    """
    base = Path(root)
    if not base.is_dir():
        raise CollectorError(f"bag root is not a directory: {root}", LAYER, NAME)
    if is_bag(root):
        return [str(base)], []
    found: list[str] = []
    refused: list[tuple[str, OSError]] = []
    for item in sorted(base.iterdir()):
        try:
            if is_bag(str(item)):
                found.append(str(item))
        except OSError as exc:
            refused.append((str(item), exc))
    return found, refused


# ---------------------------------------------------------------------------
# metadata


def rosbag2_available() -> bool:
    """Whether the preferred reader exists on this machine. Recorded as a run predicate."""
    from importlib.util import find_spec

    try:
        return find_spec("rosbag2_py") is not None
    except (ImportError, ValueError):
        return False


def _read_metadata_rosbag2(path: str) -> dict[str, Any]:
    """rosbag2_py, imported here so the core never depends on it."""
    try:
        import rosbag2_py
    except ImportError as exc:
        raise CollectorError(f"rosbag2_py is not importable: {exc}", LAYER, NAME) from exc
    info = rosbag2_py.Info()
    meta = info.read_metadata(path, "")
    topics = []
    for row in meta.topics_with_message_count:
        item = row.topic_metadata
        topics.append(
            {
                "message_count": int(row.message_count),
                "topic_metadata": {
                    "name": item.name,
                    "type": item.type,
                    "serialization_format": item.serialization_format,
                    "offered_qos_profiles": _qos_text(item.offered_qos_profiles),
                },
            }
        )
    return {
        "storage_identifier": meta.storage_identifier,
        "relative_file_paths": [str(item) for item in meta.relative_file_paths],
        "duration": {"nanoseconds": int(meta.duration.nanoseconds)},
        "starting_time": {
            "nanoseconds_since_epoch": int(
                meta.starting_time.nanoseconds
                if hasattr(meta.starting_time, "nanoseconds")
                else meta.starting_time.timestamp() * 1e9
            )
        },
        "message_count": int(meta.message_count),
        "compression_format": getattr(meta, "compression_format", ""),
        "compression_mode": getattr(meta, "compression_mode", ""),
        "version": getattr(meta, "version", None),
        "ros_distro": getattr(meta, "ros_distro", "") or "",
        "topics_with_message_count": topics,
    }


def _read_metadata(path: str) -> tuple[dict[str, Any], str, tuple[str, str]]:
    """The metadata, the path that produced it, and the refusal when the preferred one failed.

    rosbag2_py is preferred because it is the reader ROS itself uses. It is absent on a
    machine with no ROS and it refuses a metadata version newer than its own, and neither
    is a reason to hand back nothing when the file is right there.
    """
    if not rosbag2_available():
        # Which reader exists is a property of the machine, so it is a run predicate and
        # never a note on the recording.
        return _read_metadata_yaml(path), "metadata_yaml", ("", "")
    try:
        info = _read_metadata_rosbag2(path)
        _backfill(path, info)
        return info, "rosbag2_py", ("", "")
    except Exception as exc:
        # The message carries the installed version and the library path, so it belongs to
        # the run and not to the recording.
        refusal = (type(exc).__name__, str(exc).replace(path, Path(path).name))
    return _read_metadata_yaml(path), "metadata_yaml", refusal


READER_GATED = ("ros_distro", "version", "compression_format", "compression_mode")
"""Fields the installed rosbag2_py may not carry at all: humble's metadata object has no
`ros_distro`. Reading them out of the file the recording ships keeps the same recording
from stating two different things on two machines."""


def _backfill(path: str, info: dict[str, Any]) -> None:
    """Fill the fields the installed reader has no attribute for from the stored file.

    Only a field the reader left empty is filled, so nothing the reader stated is replaced.
    """
    unread_qos = [
        row["topic_metadata"]
        for row in info.get("topics_with_message_count", [])
        if row["topic_metadata"]["offered_qos_profiles"] is None
    ]
    if not unread_qos and not any(not info.get(key) for key in READER_GATED):
        return
    try:
        stored = _read_metadata_yaml(path)
    except (CollectorError, OSError, yaml.YAMLError):
        stored = {}
    for key in READER_GATED:
        if not info.get(key) and stored.get(key) is not None:
            info[key] = stored[key]
    stored_qos = {
        row["topic_metadata"].get("name"): row["topic_metadata"].get("offered_qos_profiles")
        for row in stored.get("topics_with_message_count", [])
        if isinstance(row, dict) and isinstance(row.get("topic_metadata"), dict)
    }
    for item in unread_qos:
        item["offered_qos_profiles"] = _qos_text(stored_qos.get(item["name"])) or ""


def _read_metadata_yaml(path: str) -> dict[str, Any]:
    try:
        text = Path(path, METADATA_NAME).read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise CollectorError(f"{METADATA_NAME} is not utf-8: {exc.reason}", LAYER, NAME) from exc
    payload = yaml.safe_load(text) or {}
    if not isinstance(payload, dict):
        raise CollectorError(
            f"the root of {METADATA_NAME} is a {type(payload).__name__} and not a mapping",
            LAYER,
            NAME,
        )
    info = payload.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise CollectorError(f"{METADATA_NAME} has no rosbag2_bagfile_information", LAYER, NAME)
    return info


def _apply_metadata(read: BagRead, info: dict[str, Any]) -> None:
    read.metadata_present = True
    read.storage_id = str(info.get("storage_identifier") or "") or None
    version = info.get("version")
    read.metadata_version = int(version) if isinstance(version, int) else None
    distro = str(info.get("ros_distro") or "")
    if distro:
        read.ros_distro = distro
        read.ros_distro_source = "metadata"
    for key in ("compression_format", "compression_mode"):
        value = info.get(key)
        if value is not None:
            setattr(read, key, str(value))
    # A storage path is relative to the bag by contract. One that is not is reduced to its
    # name, because an absolute path must never reach a file id.
    files = [
        Path(item).name if Path(item).is_absolute() else str(item)
        for item in info.get("relative_file_paths") or []
    ]
    if files:
        # One path named twice is one storage file, and reading it twice would double
        # every count the storage produces.
        read.files = [StorageRead(relpath=item) for item in sorted(dict.fromkeys(files))]
        read.files_source = "metadata"
    count = info.get("message_count")
    if isinstance(count, int):
        read.message_count = count
        read.message_count_source = "metadata"
    duration = (info.get("duration") or {}).get("nanoseconds")
    if isinstance(duration, int):
        read.duration_ns = duration
        read.duration_source = "metadata"
    start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
    if isinstance(start, int):
        read.start_ns = start
        if isinstance(duration, int):
            read.end_ns = start + duration
    for row in info.get("topics_with_message_count") or []:
        item = row.get("topic_metadata") or {}
        name = str(item.get("name") or "")
        if not name:
            continue
        topic = TopicRead(
            name=name,
            type_name=str(item.get("type") or ""),
            serialization_format=str(item.get("serialization_format") or ""),
        )
        _set_qos(topic, _qos_text(item.get("offered_qos_profiles")), "metadata")
        stated = row.get("message_count")
        if isinstance(stated, int):
            topic.count = stated
            topic.stated_count = stated
            topic.count_source = "metadata"
        read.topics.append(topic)


def _qos_text(value: Any) -> str | None:
    """The offered QoS of one topic as the yaml text humble writes, or None when the reader
    holds it in a form nothing can read back.

    humble hands the profiles over as that text and a jazzy `metadata.yaml` stores a yaml
    list. jazzy's rosbag2_py hands over `rclcpp::QoS` bindings that only have setters, so
    for those the stored file is the one place the values can come from.
    """
    if not value:
        return ""
    if isinstance(value, str):
        return value
    if all(isinstance(row, dict) for row in value):
        return yaml.safe_dump(list(value), sort_keys=False)
    return None


def _set_qos(topic: TopicRead, raw: str, source: str) -> None:
    topic.qos_raw = raw
    topic.qos_source = source
    if not raw:
        topic.qos = []
        return
    try:
        parsed = yaml.safe_load(raw)
    except yaml.YAMLError:
        topic.qos_readable = False
        return
    if isinstance(parsed, list):
        topic.qos = parsed
    elif isinstance(parsed, dict):
        topic.qos = [parsed]
    else:
        topic.qos_readable = False


# ---------------------------------------------------------------------------
# the sqlite page format, for a file no sqlite client will open


class _Pager:
    """Read only view of a sqlite file as pages.

    A killed recording leaves a header that claims more pages than the file holds, which
    is what makes every sqlite client call it malformed. Here a page past the end of the
    file is one counted gap and not the end of the read.
    """

    def __init__(self, path: str) -> None:
        # The handle outlives the call on purpose: a page is read on demand, so the file
        # is never held in memory.
        self.handle = Path(path).open("rb")  # noqa: SIM115
        head = self.handle.read(100)
        if head[:16] != SQLITE_MAGIC:
            self.handle.close()
            raise CollectorError(f"{Path(path).name} is not a sqlite file", LAYER, NAME)
        declared = struct.unpack(">H", head[16:18])[0]
        self.page_size = 65536 if declared == 1 else declared
        self.usable = self.page_size - head[20]
        self.file_pages = Path(path).stat().st_size // self.page_size
        self.header_pages = struct.unpack(">I", head[28:32])[0]
        self.gaps = 0

    def close(self) -> None:
        self.handle.close()

    def page(self, number: int) -> bytes | None:
        if number < 1 or number > self.file_pages:
            self.gaps += 1
            return None
        self.handle.seek((number - 1) * self.page_size)
        buffer = self.handle.read(self.page_size)
        return buffer if len(buffer) == self.page_size else None


def _varint(buffer: bytes, offset: int) -> tuple[int, int]:
    value = 0
    for step in range(8):
        byte = buffer[offset + step]
        value = (value << 7) | (byte & 0x7F)
        if not byte & 0x80:
            return value, offset + step + 1
    return (value << 8) | buffer[offset + 8], offset + 9


_FIXED = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 8, 7: 8, 8: 0, 9: 0}


def _record(buffer: bytes, start: int, end: int, want: int) -> list[Any]:
    """Decode the first `want` columns of a record from its local bytes.

    A column the local part does not reach comes back as `_TRUNCATED`. Nothing beyond it
    is decoded and no overflow page is followed, which is what keeps a blob unread.
    """
    header_size, cursor = _varint(buffer, start)
    header_end = start + header_size
    body = header_end
    out: list[Any] = []
    while cursor < header_end and len(out) < want:
        serial, cursor = _varint(buffer, cursor)
        if serial < 10:
            size = _FIXED[serial]
        elif serial >= 12:
            size = (serial - 12) // 2 if serial % 2 == 0 else (serial - 13) // 2
        else:
            size = 0
        if body + size > end:
            out.append(_TRUNCATED)
            break
        raw = buffer[body : body + size]
        if serial == 0:
            out.append(None)
        elif serial <= 6:
            out.append(int.from_bytes(raw, "big", signed=True))
        elif serial == 7:
            out.append(struct.unpack(">d", raw)[0])
        elif serial == 8:
            out.append(0)
        elif serial == 9:
            out.append(1)
        elif serial % 2:
            out.append(raw.decode("utf-8", "replace"))
        else:
            out.append(raw)
        body += size
    return out


def _walk_table(pager: _Pager, root: int, want: int, visit) -> None:
    """Depth first over one table b tree, calling visit(rowid, columns) per row."""
    max_local = pager.usable - 35
    min_local = ((pager.usable - 12) * 32) // 255 - 23
    seen = bytearray((pager.file_pages + 8) // 8)
    stack = [root]
    while stack:
        number = stack.pop()
        if number < 1 or number > pager.file_pages:
            pager.gaps += 1
            continue
        if seen[number >> 3] & (1 << (number & 7)):
            continue
        seen[number >> 3] |= 1 << (number & 7)
        buffer = pager.page(number)
        if buffer is None:
            continue
        base = 100 if number == 1 else 0
        kind = buffer[base]
        cells = struct.unpack(">H", buffer[base + 3 : base + 5])[0]
        if kind == 0x05:
            stack.append(struct.unpack(">I", buffer[base + 8 : base + 12])[0])
            array = base + 12
        elif kind == 0x0D:
            array = base + 8
        else:
            continue
        for index in range(cells):
            spot = array + 2 * index
            if spot + 2 > len(buffer):
                break
            cell = struct.unpack(">H", buffer[spot : spot + 2])[0]
            if cell < 1 or cell >= len(buffer):
                continue
            if kind == 0x05:
                stack.append(struct.unpack(">I", buffer[cell : cell + 4])[0])
                continue
            size, offset = _varint(buffer, cell)
            rowid, offset = _varint(buffer, offset)
            if size <= max_local:
                local = size
            else:
                spill = min_local + (size - min_local) % (pager.usable - 4)
                local = spill if spill <= max_local else min_local
            visit(rowid, _record(buffer, offset, min(offset + local, len(buffer)), want))


def _columns_of(sql: str) -> list[str]:
    body = sql[sql.index("(") + 1 : sql.rindex(")")]
    names = []
    for part in body.split(","):
        found = _COLUMN_RE.match(part)
        if found:
            names.append(found.group(1))
    return names


def _pages_tables(pager: _Pager) -> dict[str, tuple[int, list[str]]]:
    tables: dict[str, tuple[int, list[str]]] = {}

    def visit(_rowid: int, columns: list[Any]) -> None:
        if len(columns) < 5 or columns[0] != "table":
            return
        if any(item is _TRUNCATED for item in columns) or not isinstance(columns[3], int):
            return
        try:
            names = _columns_of(str(columns[4]))
        except ValueError:
            names = []
        tables[str(columns[1])] = (int(columns[3]), names)

    _walk_table(pager, 1, 5, visit)
    return tables


# ---------------------------------------------------------------------------
# statistics, one storage file at a time


@dataclass
class _Stats:
    """Per topic id counts and span, plus the topic rows the storage file holds."""

    topics: dict[int, dict[str, str]] = field(default_factory=dict)
    counts: dict[int, list[int]] = field(default_factory=dict)
    ros_distro: str = ""
    unreadable_rows: int = 0


def _read_sqlite3(path: str) -> _Stats:
    stats = _Stats()
    connection = sqlite3.connect(f"{Path(path).as_uri()}?mode=ro", uri=True)
    try:
        names = [row[1] for row in connection.execute("PRAGMA table_info(topics)")]
        wanted = [name for name in ("name", "type", "serialization_format") if name in names]
        if "offered_qos_profiles" in names:
            wanted.append("offered_qos_profiles")
        query = ", ".join(["id", *wanted])
        for row in connection.execute(f"select {query} from topics"):
            values = [str(item) for item in row[1:]]
            stats.topics[int(row[0])] = dict(zip(wanted, values, strict=True))
        for topic_id, count, first, last in connection.execute(
            "select topic_id, count(*), min(timestamp), max(timestamp) from messages "
            "group by topic_id"
        ):
            stats.counts[int(topic_id)] = [int(count), int(first), int(last)]
        try:
            for row in connection.execute("select ros_distro from schema"):
                stats.ros_distro = str(row[0])
        except sqlite3.OperationalError:
            pass
    finally:
        connection.close()
    return stats


def _read_pages(path: str) -> tuple[_Stats, int]:
    stats = _Stats()
    pager = _Pager(path)
    try:
        tables = _pages_tables(pager)
        if "topics" in tables:
            root, names = tables["topics"]
            wanted = {name: index for index, name in enumerate(names)}

            def on_topic(rowid: int, columns: list[Any]) -> None:
                row: dict[str, str] = {}
                for key in ("name", "type", "serialization_format", "offered_qos_profiles"):
                    index = wanted.get(key)
                    if index is None or index >= len(columns):
                        continue
                    value = columns[index]
                    if value is _TRUNCATED or value is None:
                        continue
                    row[key] = str(value)
                identity = columns[0] if columns and isinstance(columns[0], int) else rowid
                stats.topics[int(identity)] = row

            _walk_table(pager, root, len(names), on_topic)
        if "schema" in tables:
            root, names = tables["schema"]
            at = names.index("ros_distro") if "ros_distro" in names else -1

            def on_schema(_rowid: int, columns: list[Any]) -> None:
                if at >= 0 and at < len(columns) and isinstance(columns[at], str):
                    stats.ros_distro = columns[at]

            if at >= 0:
                _walk_table(pager, root, at + 1, on_schema)
        if "messages" in tables:
            root, names = tables["messages"]
            index_of = {name: index for index, name in enumerate(names)}
            topic_at = index_of.get("topic_id", 1)
            stamp_at = index_of.get("timestamp", 2)
            want = max(topic_at, stamp_at) + 1

            def on_message(_rowid: int, columns: list[Any]) -> None:
                if len(columns) <= stamp_at:
                    stats.unreadable_rows += 1
                    return
                topic_id = columns[topic_at]
                stamp = columns[stamp_at]
                if not isinstance(topic_id, int) or not isinstance(stamp, int):
                    stats.unreadable_rows += 1
                    return
                row = stats.counts.get(topic_id)
                if row is None:
                    stats.counts[topic_id] = [1, stamp, stamp]
                    return
                row[0] += 1
                if stamp < row[1]:
                    row[1] = stamp
                if stamp > row[2]:
                    row[2] = stamp

            _walk_table(pager, root, want, on_message)
        gaps = pager.gaps
    finally:
        pager.close()
    return stats, gaps


def _merge_stats(read: BagRead, stats: _Stats) -> None:
    """Fold one storage file into the bag. A count from the storage always replaces a
    count the metadata stated, because the metadata states the whole recording and the
    storage states what is on disk, and a split bag adds its files up."""
    if stats.ros_distro and not read.ros_distro:
        read.ros_distro = stats.ros_distro
        read.ros_distro_source = "sqlite_schema"
    by_name = {topic.name: topic for topic in read.topics}
    for topic_id in sorted(stats.topics):
        row = stats.topics[topic_id]
        name = row.get("name", "")
        if not name:
            continue
        topic = by_name.get(name)
        if topic is None:
            topic = TopicRead(name=name)
            by_name[name] = topic
            read.topics.append(topic)
        if not topic.type_name:
            topic.type_name = row.get("type", "")
        if not topic.serialization_format:
            topic.serialization_format = row.get("serialization_format", "")
        if not topic.qos_source and "offered_qos_profiles" in row:
            _set_qos(topic, row["offered_qos_profiles"], "storage_topics")
        count, first, last = stats.counts.get(topic_id, (0, None, None))
        topic.count = count if topic.count_source != "recovered" else (topic.count or 0) + count
        topic.count_source = "recovered"
        if first is not None:
            topic.first_ns = first if topic.first_ns is None else min(topic.first_ns, first)
            topic.last_ns = last if topic.last_ns is None else max(topic.last_ns, last)


# ---------------------------------------------------------------------------
# the one entry point


def read_bag(path: str) -> BagRead:
    """Everything one recording can be made to say, with the read path recorded."""
    directory = Path(path)
    if not directory.is_dir():
        raise CollectorError(f"bag is not a directory: {path}", LAYER, NAME)
    read = BagRead(name=directory.name, path=str(directory))
    if (directory / METADATA_NAME).is_file():
        try:
            info, read.metadata_path, read.reader_refusal = _read_metadata(str(directory))
        except (CollectorError, OSError, yaml.YAMLError) as exc:
            # A recorder killed while it writes the metadata leaves a file no reader
            # parses. The storage is still on disk, so the cost is named and the recording
            # is read the same way as one that carries no metadata at all.
            read.reader_refusal = (type(exc).__name__, str(exc).replace(path, directory.name))
            read.missing = list(METADATA_CARRIES)
            read.notes.append(
                "metadata.yaml is there and no reader parsed it, so the topics, the types "
                "and the QoS come from the storage tables and the rest is not stated"
            )
        else:
            _apply_metadata(read, info)
    else:
        read.missing = list(METADATA_CARRIES)
        read.notes.append(
            "no metadata.yaml, the recording was not closed, so the topics, the types and "
            "the QoS come from the storage tables and the rest is not stated"
        )
    if not read.files:
        read.files = [
            StorageRead(relpath=item.name)
            for item in sorted(directory.iterdir())
            if item.is_file() and item.suffix in STORAGE_SUFFIXES
        ]
        read.files_source = "directory"
    if not read.files:
        raise CollectorError(f"{directory.name} holds no storage file", LAYER, NAME)
    _collect_statistics(read, directory)
    _finish(read)
    return read


def _sqlite_refusal(exc: Exception) -> str:
    """Why the sqlite module was left for the page reader, in words that hold two runs.

    A sqlite message is a fixed string of the library. A TypeError or a ValueError carries
    the damaged row, so only the fact that a column had the wrong type is kept.
    """
    if isinstance(exc, sqlite3.DatabaseError):
        return str(exc)
    return "a column of the topics or the messages table does not hold the type it declares"


def _collect_statistics(read: BagRead, directory: Path) -> None:
    paths: set[str] = set()
    for item in sorted(read.files, key=lambda row: row.relpath):
        target = directory / item.relpath
        if target.is_file():
            item.size = target.stat().st_size
        if not target.is_file():
            item.present = False
            item.read_path = "none"
            item.note = "not on disk"
            read.notes.append(f"{item.relpath} is named by the metadata and is not on disk")
            continue
        if target.suffix not in SQLITE_SUFFIXES:
            item.read_path = "none"
            item.note = "storage is not sqlite3"
            read.notes.append(f"{item.relpath} is not a sqlite3 file, so it carries no statistics")
            continue
        if read.storage_format is None:
            with target.open("rb") as handle:
                if handle.read(16) == SQLITE_MAGIC:
                    read.storage_format = "sqlite3"
        try:
            stats = _read_sqlite3(str(target))
            item.read_path = "sqlite3"
        except (sqlite3.DatabaseError, TypeError, ValueError, CollectorError) as exc:
            # A TypeError or a ValueError here is sqlite opening the file and handing back
            # a row rosbag2 cannot have written, so the tables are damaged either way. The
            # reason is the class and never the message, because a message built from a
            # damaged row carries the damaged bytes into compared data.
            reason = _sqlite_refusal(exc)
            try:
                stats, gaps = _read_pages(str(target))
            except (CollectorError, OSError, ValueError) as page_exc:
                # Neither reader recognises the bytes, so this file states nothing. The
                # recording keeps every file that does, and the loss is one entry.
                item.read_path = "none"
                item.refusal = (
                    str(page_exc.args[0])
                    if isinstance(page_exc, CollectorError)
                    else type(page_exc).__name__
                )
                item.note = item.refusal
                read.notes.append(f"{item.relpath} is not readable as storage ({item.refusal})")
                continue
            item.read_path = "sqlite_pages"
            item.unreadable_pages = gaps
            item.note = reason
            read.notes.append(
                f"{item.relpath} is not readable by sqlite ({reason}), so it was read page by page"
            )
        _merge_stats(read, stats)
        paths.add(item.read_path)
        if stats.unreadable_rows:
            # The number of such rows is a count, so it stays out of a note.
            read.notes.append(f"{item.relpath} holds rows whose columns are cut off")
    read.statistics_path = "+".join(sorted(paths - {"none"})) or "none"


def _finish(read: BagRead) -> None:
    """Fill in what the recording did not state, and only from what was read."""
    counted = [
        topic.count
        for topic in read.topics
        if topic.count is not None and topic.count_source == "recovered"
    ]
    if counted:
        read.recovered_count = sum(counted)
    if read.truncated:
        read.notes.append(
            "pages of the storage are past the end of the file, so every count and every "
            "span here is a floor and not the recording"
        )
    if read.absent:
        read.notes.append(
            "a storage file the metadata names is not on disk, so every count and every "
            "span here is a floor and not the recording"
        )
    if not read.metadata_present:
        stamps = [topic.first_ns for topic in read.topics if topic.first_ns is not None]
        ends = [topic.last_ns for topic in read.topics if topic.last_ns is not None]
        if stamps and ends:
            read.start_ns = min(stamps)
            read.end_ns = max(ends)
            read.duration_ns = read.end_ns - read.start_ns
            read.duration_source = "recovered"
        if read.recovered_count is not None:
            read.message_count = read.recovered_count
            read.message_count_source = "recovered"
    for topic in read.topics:
        # A count from a file that was not read whole is a floor, and one slice of the
        # atlas has to say so on its own.
        topic.complete = read.complete
    read.topics.sort(key=lambda topic: topic.name)


# ---------------------------------------------------------------------------
# the collector


@dataclass
class BagCollector:
    """The recorded layer over every configured bag root.

    Bags are named by a root of kind `bag`, so a workspace that configures none produces
    no recorded layer and says nothing, which is the honest answer.
    """

    name: str = NAME
    layer: str = LAYER
    paths: tuple[str, ...] = ()

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        """Whether any configured root holds a recording.

        Recorded is the one layer a workspace may legitimately never have. Answering
        `completed` there claimed a pass over inputs that were never there, which reads the
        same as a pass that read every recording it was given.
        """
        if not self.paths and not any(spec.kind == model.Root.BAG.value for spec in ctx.cfg.roots):
            return NO_ROOT
        found, problems = self.recordings(ctx)
        return True if found or problems else NO_RECORDING

    def collect(self, ctx: CollectContext) -> None:
        found, problems = self.recordings(ctx)
        if not found and not problems:
            return  # nothing to read, which `inputs_present` states to the orchestrator
        reads: list[BagRead] = []
        refused: list[tuple[str, str]] = []
        for path in found:
            try:
                reads.append(read_bag(path))
            except Exception as exc:
                problems.append(_problem(path, "describe", exc))
                # A configured path that is not a recording at all is an absent input and
                # not a refusal, so it mints nothing. Only a directory that says it is a
                # recording and then refuses to be read costs an entity.
                if is_bag(path):
                    refused.append((path, str(exc).replace(path, Path(path).name)))
        reads, clashes = _one_read_per_name(reads)
        problems.extend(clashes)
        problems.extend(_refusal(item) for item in reads if item.reader_refusal[0])
        # The digest covers the probed predicate as well as the inputs, so a box whose
        # reader differs states a different input and never a different answer.
        predicates = {"rosbag2_py": rosbag2_available()}
        digest = ids.digest(
            NAME,
            *[f"{key}={predicates[key]}" for key in sorted(predicates)],
            *[f"{item.name}={_root_digest(item)}" for item in reads],
            *[f"{Path(path).name}=refused" for path, _reason in sorted(refused)],
        )
        # The run id is formed before the facts because a bag that raises while it is
        # written still has to reach the run's exception list.
        run = ids.build_run(LAYER, "", digest)
        for item in reads:
            try:
                self._emit(ctx, ctx.run or run, item)
            except Exception as exc:
                problems.append(_problem(item.path, "emit", exc))
        for path, reason in sorted(refused):
            self._refuse(ctx, path, reason)
        ctx.builder.add_run(
            LAYER,
            input_digest=digest,
            roots={item.name: _root_digest(item) for item in reads},
            predicates=predicates,
            exceptions=sorted(problems, key=_problem_order),
        )
        ctx.builder.declare_layer(
            LAYER,
            present=bool(reads),
            run=run,
            reason="" if reads else "every configured recording refused to be read",
            root=model.Root.BAG.value,
        )

    def _refuse(self, ctx: CollectContext, path: str, reason: str) -> None:
        """A recording no reader opened at all. It still costs its entity and one blind
        spot, because a recording that left nothing behind read exactly like a run that
        was never configured."""
        bag_id = ids.build_bag(Path(path).name)
        ctx.builder.node(
            bag_id,
            attrs={
                "loadable": False,
                "refused_because": "bag_unreadable",
                "refused_detail": reason,
            },
        )
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=bag_id,
                expr=Path(path).name,
                reason=reason,
                role="rosbag2_storage",
                resolvable_by=["point the bag root at a recording a reader can open"],
            )
        )

    def recordings(self, ctx: CollectContext) -> tuple[list[str], list[dict[str, Any]]]:
        if self.paths:
            return sorted(self.paths), []
        roots = [spec.path for spec in ctx.cfg.roots if spec.kind == model.Root.BAG.value]
        found: list[str] = []
        problems: list[dict[str, Any]] = []
        for root in sorted(roots):
            try:
                children, refused = scan_bags(root)
            except Exception as exc:
                problems.append(_problem(root, "root", exc))
                continue
            found.extend(children)
            problems.extend(_problem(path, "root", exc) for path, exc in refused)
        return sorted(dict.fromkeys(found)), problems

    def _emit(self, ctx: CollectContext, run: str, read: BagRead) -> None:
        builder = ctx.builder
        bag_id = ids.build_bag(read.name)
        builder.node(bag_id, attrs={"metadata_present": read.metadata_present})
        evidence = []
        for item in sorted(read.files, key=lambda row: row.relpath):
            file_id = _file_id(read.name, item.relpath)
            attrs: dict[str, Any] = {"read_path": item.read_path, "present": item.present}
            if item.refusal:
                attrs["loadable"] = False
                attrs["refused_because"] = "bag_storage_unreadable"
                attrs["refused_detail"] = item.refusal
            elif not item.present:
                attrs["loadable"] = False
                attrs["refused_because"] = "bag_storage_absent"
            builder.node(file_id, attrs=attrs)
            builder.edge(model.EdgeKind.CONTAINS.value, bag_id, file_id)
            evidence.append(model.Source(file=file_id, root=model.Root.BAG.value))
        if read.metadata_present:
            meta_id = _file_id(read.name, METADATA_NAME)
            builder.node(meta_id)
            builder.edge(model.EdgeKind.CONTAINS.value, bag_id, meta_id)
            evidence.append(
                model.Source(
                    file=meta_id,
                    locator="rosbag2_bagfile_information",
                    root=model.Root.BAG.value,
                )
            )
        builder.fact(
            bag_id,
            LAYER,
            run=run,
            values=read.values(),
            confidence=(
                model.Confidence.CERTAIN.value
                if read.metadata_present and read.complete
                else model.Confidence.RESOLVED.value
            ),
            evidence=evidence,
        )
        if not read.metadata_present:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=bag_id,
                    expr=f"{METADATA_NAME} of {read.name}",
                    reason="bag_metadata_missing",
                    role="rosbag2_metadata",
                    resolvable_by=[f"the metadata would have stated {', '.join(read.missing)}"],
                    evidence=evidence,
                )
            )
        if read.truncated:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=bag_id,
                    expr=f"storage pages of {read.name} past the end of the file",
                    reason="bag_storage_truncated",
                    role="rosbag2_storage",
                    resolvable_by=["nothing, the bytes are not on disk"],
                    evidence=evidence,
                )
            )
        for item in sorted(read.files, key=lambda row: row.relpath):
            if not item.refusal:
                continue
            # The owner is the storage file and not the recording, because the entity a
            # reader opens after seeing `loadable false` is the file that refused.
            file_id = _file_id(read.name, item.relpath)
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=file_id,
                    expr=f"{item.relpath} of {read.name}",
                    reason=item.refusal,
                    role="rosbag2_storage",
                    resolvable_by=["the file holds no storage a reader recognises"],
                    evidence=[model.Source(file=file_id, root=model.Root.BAG.value)],
                )
            )
        for item in sorted(read.files, key=lambda row: row.relpath):
            if item.present:
                continue
            file_id = _file_id(read.name, item.relpath)
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=file_id,
                    expr=f"{item.relpath} of {read.name}",
                    reason="bag_storage_absent",
                    role="rosbag2_storage",
                    resolvable_by=["put the storage file the metadata names next to it"],
                    evidence=[model.Source(file=file_id, root=model.Root.BAG.value)],
                )
            )
        for topic in read.topics:
            self._emit_topic(ctx, run, read, bag_id, topic, evidence)

    def _emit_topic(
        self,
        ctx: CollectContext,
        run: str,
        read: BagRead,
        bag_id: str,
        topic: TopicRead,
        evidence: list[model.Source],
    ) -> None:
        builder = ctx.builder
        try:
            bagtopic_id = ids.build_bagtopic(read.name, topic.name)
        except ContractError:
            # A name recovered from a damaged page is not a ros name, and a name the
            # grammar refuses mints neither a bagtopic nor a topic.
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.TOPIC_NAME.value,
                    owner=bag_id,
                    expr=topic.name[:200],
                    reason="topic_name_unusable",
                    role="rosbag2_topic",
                    resolvable_by=["nothing, the bytes of the topics table are damaged"],
                    evidence=evidence,
                )
            )
            return
        builder.fact(
            bagtopic_id,
            LAYER,
            run=run,
            values=topic.values(),
            confidence=(
                model.Confidence.CERTAIN.value
                if read.metadata_present and read.complete
                else model.Confidence.RESOLVED.value
            ),
            evidence=evidence,
        )
        builder.edge(model.EdgeKind.CONTAINS.value, bag_id, bagtopic_id)
        builder.count_resolved(model.Want.TOPIC_NAME.value)
        topic_id = ids.build_topic(ids.normalise_ros_name(topic.name))
        builder.node(topic_id)
        values: dict[str, Any] = {}
        if topic.type_name:
            values["type"] = topic.type_name
        if topic.serialization_format:
            values["serialization_format"] = topic.serialization_format
        builder.edge(
            model.EdgeKind.SAMPLES.value,
            bagtopic_id,
            topic_id,
            layer=LAYER,
            run=run,
            values=values,
            evidence=evidence,
        )
        if topic.qos_readable and topic.qos_raw:
            builder.count_resolved(model.Want.QOS.value)
        elif not topic.qos_readable:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.QOS.value,
                    owner=bagtopic_id,
                    expr=topic.qos_raw[:200],
                    reason="qos_profile_unreadable",
                    role="offered_qos_profiles",
                    evidence=evidence,
                )
            )
        self._emit_type(ctx, bagtopic_id, topic, evidence)

    def _emit_type(
        self,
        ctx: CollectContext,
        bagtopic_id: str,
        topic: TopicRead,
        evidence: list[model.Source],
    ) -> None:
        parts = topic.type_name.split("/")
        if len(parts) != 3 or not all(parts) or parts[1] not in ids.INTERFACE_KINDS:
            if topic.type_name:
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.TYPE.value,
                        owner=bagtopic_id,
                        expr=topic.type_name,
                        reason="type_name_unreadable",
                        evidence=evidence,
                    )
                )
            return
        type_id = ids.build_type(parts[0], parts[1], parts[2])
        ctx.builder.node(type_id)
        ctx.builder.edge(model.EdgeKind.USES_TYPE.value, bagtopic_id, type_id)
        ctx.builder.count_resolved(model.Want.TYPE.value)


def _problem(path: str, phase: str, exc: Exception) -> dict[str, Any]:
    """One recording that could not be read or could not be written, so the rest still is.

    The message names the directory and never the machine path, because a run record is
    compared across two boxes.
    """
    name = Path(path).name
    return {
        "file": ids.build_bag(name),
        "phase": phase,
        "exc_type": type(exc).__name__,
        "message": str(exc).replace(path, name),
    }


def _refusal(read: BagRead) -> dict[str, Any]:
    """The preferred reader refusing one recording, recorded as an input of this run."""
    return {
        "file": ids.build_bag(read.name),
        "phase": "metadata",
        "exc_type": read.reader_refusal[0],
        "message": read.reader_refusal[1],
    }


def _problem_order(row: dict[str, Any]) -> tuple[str, str, str]:
    return (row["file"], row["phase"], row["message"])


def _one_read_per_name(
    reads: list[BagRead],
) -> tuple[list[BagRead], list[dict[str, Any]]]:
    """A bag id is its directory basename, so two recordings that share one basename share
    one id. Neither is emitted: picking one would state the wrong recording under that id,
    and the clash is named in the run instead."""
    seen: dict[str, list[BagRead]] = {}
    for item in reads:
        seen.setdefault(item.name, []).append(item)
    kept = [item for item in reads if len(seen[item.name]) == 1]
    problems = [
        _problem(
            group[0].path,
            "describe",
            CollectorError(
                f"{len(group)} recordings of this run are named {name}, so neither can hold "
                "the id and none of them is stated",
                LAYER,
                NAME,
            ),
        )
        for name, group in sorted(seen.items())
        if len(group) > 1
    ]
    return kept, problems


def _root_digest(read: BagRead) -> str:
    """One hash per recording, over the storage files it holds and their sizes."""
    return ids.digest(
        *[f"{row.relpath}:{row.size}" for row in sorted(read.files, key=lambda file: file.relpath)]
    )


def _file_id(bag_name: str, relpath: str) -> str:
    return ids.build_file(BAG_PKG, f"{bag_name}/{relpath}")


register(NAME, BagCollector)


__all__ = [
    "BAG_PKG",
    "LAYER",
    "METADATA_CARRIES",
    "METADATA_NAME",
    "NAME",
    "NO_RECORDING",
    "NO_ROOT",
    "BagCollector",
    "BagRead",
    "StorageRead",
    "TopicRead",
    "find_bags",
    "is_bag",
    "read_bag",
    "scan_bags",
]
