"""The human readable outputs, generated from one atlas and nothing else.

Two files are produced here. `ATLAS_REPORT.md` is what a person reads first: what the
system is, which profiles exist, what runs in them, and how much of it rosio actually
read. `DRIFT.md` is the findings, ordered by severity and grouped by rule family, each one
carrying the file to edit.

Both are pure functions of a loaded atlas. Nothing here reads a file, re extracts, mutates
an entity or calls a clock, so two runs over one atlas produce identical bytes. The markdown
is plain enough to paste into a chat: no html, no reference links, and a table only where a
table carries more than a sentence would.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

REPORT_NAME = "ATLAS_REPORT.md"
DRIFT_NAME = "DRIFT.md"
HTML_NAME = "atlas.html"

MD_FORMAT = "md"
HTML_FORMAT = "html"


RENDERED_FROM = "Rendered from atlas"
"""The phrase every rendered file states its atlas's content hash with. `rosio extract`
reads it back, so a report left behind by an earlier atlas says so instead of describing a
system that is no longer the one on disk."""


def rendered_from(atlas: Any) -> str:
    return f"{RENDERED_FROM} {atlas.content_hash()}"


def stale(out_dir: str | Path, atlas: Any) -> list[str]:
    """The rendered files in `out_dir` that state another atlas than this one."""
    current = rendered_from(atlas)
    out = []
    for name in (REPORT_NAME, DRIFT_NAME, HTML_NAME):
        path = Path(out_dir) / name
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        if current not in text:
            out.append(name)
    return out


def heading(level: int, text: str) -> str:
    return f"{'#' * level} {text}"


def escape_cell(text: str) -> str:
    return str(text).replace("|", "\\|").replace("\n", " ")


def table(headers: Sequence[str], rows: Iterable[Sequence[Any]]) -> list[str]:
    """One markdown table. An empty body returns nothing, so a caller never prints a
    header over no data."""
    body = [[escape_cell(cell) for cell in row] for row in rows]
    if not body:
        return []
    out = [
        "| " + " | ".join(escape_cell(item) for item in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    out.extend("| " + " | ".join(row) + " |" for row in body)
    return out


def plural(count: int, singular: str, many: str = "") -> str:
    """A count with its noun, `1 entity` and `2 entities`. The one helper every count
    printed with a noun goes through, so no output says `1 profiles`."""
    if count == 1:
        return f"{count} {singular}"
    if not many:
        consonant_y = singular.endswith("y") and singular[-2:-1] not in ("a", "e", "i", "o", "u")
        many = f"{singular[:-1]}ies" if consonant_y else f"{singular}s"
    return f"{count} {many}"


def share(part: int, whole: int) -> str:
    """A count against its total, with the percentage a reader would otherwise compute."""
    if whole <= 0:
        return f"{part} of 0"
    return f"{part} of {whole} ({round(100 * part / whole)} percent)"


def clip(text: str, width: int = 120) -> str:
    text = str(text)
    return text if len(text) <= width else text[: width - 3] + "..."


def joined(items: Iterable[str], empty: str = "none") -> str:
    names = list(items)
    return ", ".join(names) if names else empty


def write_all(
    atlas: Any,
    *,
    out_dir: str | Path | None = None,
    formats: Sequence[str] = (MD_FORMAT,),
) -> list[str]:
    """Write the outputs the caller asked for and return what was written.

    The html renderer is a separate module. When it is not present the format is left out
    of the returned list rather than reported as written.
    """
    from rosio.config import DEFAULT_OUT_DIR
    from rosio.render.drift_md import render_drift
    from rosio.render.report import render_report

    target = Path(out_dir or DEFAULT_OUT_DIR)
    target.mkdir(parents=True, exist_ok=True)
    wanted = list(dict.fromkeys(formats or (MD_FORMAT,)))
    written: list[str] = []
    if MD_FORMAT in wanted:
        for name, text in ((REPORT_NAME, render_report(atlas)), (DRIFT_NAME, render_drift(atlas))):
            path = target / name
            path.write_text(text, encoding="utf-8")
            written.append(str(path))
    if HTML_FORMAT in wanted:
        try:
            from rosio.render import html
        except ImportError:
            return written
        writer = getattr(html, "write", None)
        if writer is None:
            return written
        written.append(str(writer(atlas, target)))
    return written


__all__ = [
    "DRIFT_NAME",
    "HTML_FORMAT",
    "HTML_NAME",
    "MD_FORMAT",
    "RENDERED_FROM",
    "REPORT_NAME",
    "clip",
    "escape_cell",
    "heading",
    "joined",
    "plural",
    "share",
    "table",
    "write_all",
]
