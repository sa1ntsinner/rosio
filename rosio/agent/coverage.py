"""One Stop hook nudge toward packages with source the session never opened."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

SKIP = {".git", "rosio-out", "test", "tests", "build", "install"}
SRC = {".cpp", ".hpp", ".h", ".cc", ".py"}


def sources(root: Path) -> list[Path]:
    return sorted(
        path for path in root.rglob("*")
        if path.is_file() and path.suffix in SRC and path.name != "setup.py"
        and not SKIP.intersection(path.relative_to(root).parts)
    )


def packages(root: Path) -> list[Path]:
    return sorted(
        xml.parent for xml in root.rglob("package.xml")
        if not SKIP.intersection(xml.relative_to(root).parts) and sources(xml.parent)
    )


def touched(transcript: str) -> str:
    text = []
    with open(transcript, encoding="utf-8") as stream:
        for line in stream:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if message.get("type") != "assistant":
                continue
            for content in message.get("message", {}).get("content", []):
                if content.get("type") == "tool_use":
                    text.append(json.dumps(content.get("input", {})))
    return "\n".join(text)


def _run() -> None:
    data = json.load(sys.stdin)
    if data.get("stop_hook_active"):
        return
    root = Path(os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or ".")
    seen = touched(data["transcript_path"])
    rows, unread, total, read = [], [], 0, 0
    for package in packages(root):
        files = sources(package)
        # shortcut: basenames approximate opened files, use paths if collisions matter.
        got = [path for path in files if path.name in seen]
        total += len(files)
        read += len(got)
        if len(got) < len(files):
            rows.append(f"{package.relative_to(root).as_posix()} {len(got)}/{len(files)}")
            unread.extend(path for path in files if path.name not in seen)
    if not total or read == 0 or read * 10 >= total * 8:
        return
    big = sorted(unread, key=lambda path: (-path.stat().st_size, path.as_posix()))[:25]
    print(json.dumps({"decision": "block", "reason": (
        f"rosio: this session opened {read} of the {total} source files in this repository. "
        f"Per package, files opened of files held: {'; '.join(rows[:40])}. The largest files not "
        f"opened: {', '.join(path.relative_to(root).as_posix() for path in big)}. If this is a bug hunt "
        "or a review, read the node, plugin and driver files among them the way "
        ".claude/skills/rosio/review.md asks and "
        "add what you find to the report before finishing. Otherwise finish now."
    )}))


def main() -> int:
    try:
        _run()
    except Exception as exc:
        print(f"rosio stop hook skipped: {' '.join(str(exc).splitlines())}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    main()
