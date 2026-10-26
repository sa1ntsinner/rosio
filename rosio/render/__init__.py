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

MD_FORMAT = "md"
HTML_FORMAT = "html"


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


def plural(count: int, singular: str, suffix: str = "s") -> str:
    return f"{count} {singular}" if count == 1 else f"{count} {singular}{suffix}"


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
    "MD_FORMAT",
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
