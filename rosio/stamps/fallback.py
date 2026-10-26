"""Readers for bags that rosbags refuses to open."""

import sqlite3
import struct
from contextlib import closing, suppress
from pathlib import Path

from .model import TopicSeries

PROBE_COUNT = 5
NS_PER_S = 1000000000
_STAMP_LE = struct.Struct("<iI")
_STAMP_BE = struct.Struct(">iI")
_SEP = "=" * 80 + "\n"

# Identifiers as defined in /opt/ros/humble/include/dds/ddsi/q_protocol.h lines 208 to 221.
CDR_BE = 0x0000
CDR_LE = 0x0001
PL_CDR_BE = 0x0002
PL_CDR_LE = 0x0003
INVALID_BE = 0x0004
INVALID_LE = 0x0005
CDR2_BE = 0x0006
CDR2_LE = 0x0007
D_CDR2_BE = 0x0008
D_CDR2_LE = 0x0009
PL_CDR2_BE = 0x000A
PL_CDR2_LE = 0x000B

# Supported identifier to stamp offset. Delimited XCDR2 carries a 4 byte DHEADER
# before the first member, which pushes the stamp from offset 4 to offset 8.
ENCAPSULATIONS = {
    CDR_BE: 4,
    CDR_LE: 4,
    CDR2_BE: 4,
    CDR2_LE: 4,
    D_CDR2_BE: 8,
    D_CDR2_LE: 8,
}

UNSUPPORTED = {
    PL_CDR_BE: "parameter list CDR",
    PL_CDR_LE: "parameter list CDR",
    INVALID_BE: "invalid identifier",
    INVALID_LE: "invalid identifier",
    PL_CDR2_BE: "parameter list XCDR2",
    PL_CDR2_LE: "parameter list XCDR2",
}


class UnsupportedEncapsulation(ValueError):  # noqa: N818
    """The two byte encapsulation identifier is not one this parser can read."""

    def __init__(self, identifier):
        self.identifier = identifier
        self.name = UNSUPPORTED.get(identifier, "unknown")
        super().__init__(f"unsupported encapsulation {self.name} 0x{identifier:04x}")


def encapsulation(raw):
    """Identifier and the stamp layout behind it, as (identifier, layout, offset)."""
    identifier = (raw[0] << 8) | raw[1]
    offset = ENCAPSULATIONS.get(identifier)
    if offset is None:
        raise UnsupportedEncapsulation(identifier)
    return identifier, _STAMP_LE if identifier & 1 else _STAMP_BE, offset


def cdr_stamp_parts(raw):
    """Seconds and nanoseconds of a header stamp that follows the CDR encapsulation."""
    _, layout, offset = encapsulation(raw)
    return layout.unpack_from(raw, offset)


def tolerant_reader(path, default_typestore):
    """AnyReader subclass that survives one unparseable embedded type definition."""
    from rosbags.highlevel import AnyReader, AnyReaderError
    from rosbags.interfaces import MessageDefinitionFormat
    from rosbags.typesys import get_types_from_idl, get_types_from_msg
    from rosbags.typesys.base import TypesysError

    def connection_types(connection):
        msgdef = connection.msgdef
        if msgdef.format == MessageDefinitionFormat.NONE:
            return {}
        if msgdef.data.startswith(_SEP + "IDL: "):
            types = {}
            for part in msgdef.data.split(_SEP)[1:]:
                types.update(get_types_from_idl(part.split("\n", 1)[1]))
            return types
        return get_types_from_msg(msgdef.data, connection.msgtype)

    class TolerantAnyReader(AnyReader):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.skipped = {}

        def open(self):
            try:
                super().open()
            except (TypesysError, AssertionError, AnyReaderError):
                # An empty connection list means the storage itself never opened.
                if not self.connections:
                    self._close_readers()
                    raise
                self._register_parsable()

        def _open(self):
            # From rosbags 0.11.9 `open` closes the reader before it raises, so a definition
            # it cannot parse has to be caught one level down, while the subreaders are open.
            try:
                super()._open()
            except (TypesysError, AssertionError, AnyReaderError, ValueError):
                self.connections = [c for reader in self.readers for c in reader.connections]
                if not self.connections:
                    raise
                self._register_parsable()

        def _close_readers(self):
            for reader in self.readers:
                with suppress(Exception):
                    reader.close()

        def _register_parsable(self):
            """Register the types of every connection whose definition parses, drop the rest."""
            types = {}
            keep = []
            for connection in self.connections:
                try:
                    types.update(connection_types(connection))
                except (TypesysError, AssertionError, ValueError, KeyError):
                    self.skipped[connection.topic] = connection.msgtype
                    continue
                keep.append(connection)
            self.connections = keep
            self.typestore.register(types)
            self.isopen = True

        def notices(self):
            return [
                f"skipped topic {topic}, rosbags cannot parse type {msgtype}"
                for topic, msgtype in sorted(self.skipped.items())
            ]

    return TolerantAnyReader([Path(path)], default_typestore=default_typestore)


def read_sqlite3(path, topics=None, limit=None):
    """Read a rosbag2 sqlite3 recording directly, without message definitions.

    Topics qualify on the structure of their first messages, not on the plausibility
    of the stamps they carry, which is what the checks are there to judge.
    """
    files = _storage_files(Path(path))
    if not files:
        raise ValueError(f"no sqlite3 storage under {path}")
    wanted = None if topics is None else set(topics)
    series = {}
    dropped = set()
    for file in files:
        with closing(sqlite3.connect(f"file:{file}?mode=ro", uri=True)) as db:
            for topic_id, topic, msgtype in db.execute("SELECT id, name, type FROM topics"):
                if wanted is not None and topic not in wanted:
                    continue
                entry = series.get(topic)
                if entry is None and topic in dropped:
                    continue
                remaining = None if limit is None else limit - (len(entry.stamps) if entry else 0)
                if remaining is not None and remaining <= 0:
                    continue
                if entry is None and not _qualifies(db, topic_id):
                    dropped.add(topic)
                    continue
                if entry is None:
                    entry = series[topic] = TopicSeries(topic, msgtype, [], [])
                for receive, blob in _rows(db, topic_id, remaining):
                    sec, nanosec = cdr_stamp_parts(blob)
                    entry.stamps.append(sec + nanosec * 1e-9)
                    entry.receive_times.append(receive * 1e-9)
    notices = [
        f"sqlite3 fallback dropped topic {topic}, no header stamp at the head of its messages"
        for topic in sorted(dropped)
    ]
    return series, notices


def _storage_files(path):
    if path.is_file():
        return [path] if path.suffix == ".db3" else []
    return sorted(path.glob("*.db3"))


def _qualifies(db, topic_id):
    rows = _rows(db, topic_id, PROBE_COUNT)
    if not rows:
        return False
    for _, blob in rows:
        try:
            sec, nanosec = cdr_stamp_parts(blob)
        except (struct.error, IndexError, TypeError, UnsupportedEncapsulation):
            return False
        if sec < 0 or nanosec >= NS_PER_S:
            return False
    return True


def _rows(db, topic_id, limit):
    query = (
        "SELECT timestamp, substr(data, 1, 16) FROM messages WHERE topic_id = ? ORDER BY timestamp"
    )
    if limit is None:
        return db.execute(query, (topic_id,)).fetchall()
    return db.execute(query + " LIMIT ?", (topic_id, int(limit))).fetchall()
