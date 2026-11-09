"""Stop hook: a session that changed code reviews its diff once against coding.md, and one that
only read is nudged toward the packages it never opened."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

SKIP = {".git", "rosio-out", "test", "tests", "build", "install"}
SRC = {".cpp", ".hpp", ".h", ".cc", ".py"}
MARK = "rosio: this session opened"
NUDGES = 3
REVIEW = "rosio: this session changed"
EDITS = {"Edit", "Write", "MultiEdit", "NotebookEdit"}


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


def edited(transcript: str, root: Path) -> list[str]:
    paths = set()
    with open(transcript, encoding="utf-8") as stream:
        for line in stream:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if message.get("type") != "assistant":
                continue
            for content in message.get("message", {}).get("content", []):
                if content.get("type") == "tool_use" and content.get("name") in EDITS:
                    path = Path(content.get("input", {}).get("file_path", ""))
                    try:
                        rel = path.resolve().relative_to(root.resolve())
                    except ValueError:
                        continue
                    if rel.parts and rel.parts[0] not in (".claude", "rosio-out") and rel.suffix != ".md":
                        paths.add(rel.as_posix())
    # agents also edit through the shell, so a changed file the session named counts too
    seen = touched(transcript)
    git = ["git", "-C", str(root)]
    for cmd in (["diff", "--name-only", "--relative", "HEAD"], ["ls-files", "--others", "--exclude-standard"]):
        try:
            out = subprocess.run(git + cmd, capture_output=True, text=True, timeout=20).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        for rel in out.splitlines():
            parts = Path(rel).parts
            if (parts and parts[0] not in (".claude", "rosio-out") and Path(rel).suffix != ".md"
                    and Path(rel).name not in ("CLAUDE.md", ".mcp.json", "rosio.yaml")
                    and Path(rel).name in seen):
                paths.add(Path(rel).as_posix())
    return sorted(paths)


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
    # a transcript records one hook message several times, so count by its number
    text = Path(data["transcript_path"]).read_text(encoding="utf-8")
    said = re.findall(r"\(reminder (\d+) of", text)
    nudges = max(map(int, said), default=0)
    if nudges >= NUDGES or (data.get("stop_hook_active") and not nudges):
        return
    root = Path(os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or ".")
    changed = edited(data["transcript_path"], root)
    if changed:
        if REVIEW not in text:
            print(json.dumps({"decision": "block", "reason": (
                f"{REVIEW} {', '.join(changed[:10])}. Before you finish, read "
                ".claude/skills/rosio/coding.md and check the diff against it: every path that "
                "hits this same defect is fixed, and another defect you found is named in the "
                "report rather than fixed here; a small deterministic test, no sleeps or timing, "
                "that runs the fixed line and fails without the change; comments of one line "
                "that say why and none that restate the code; nothing unrelated touched; and the package built or the test run when "
                "a toolchain is here. Fix what falls short, then finish. In the report state "
                "what you built or ran and what you could not, and nothing the diff does not "
                "show; do not recite this list."
            )}))
        return
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
    big = []
    for package in sorted({path.parent for path in unread}):
        mine = [path for path in unread if path.parent == package]
        big += sorted(mine, key=lambda path: (-path.stat().st_size, path.as_posix()))[:3]
    big = sorted(big, key=lambda path: (-path.stat().st_size, path.as_posix()))[:30]
    print(json.dumps({"decision": "block", "reason": (
        f"{MARK} {read} of the {total} source files in this repository "
        f"(reminder {nudges + 1} of {NUDGES}). Per package, files opened of files held: "
        f"{'; '.join(rows[:40])}. Large files not opened: "
        f"{', '.join(path.relative_to(root).as_posix() for path in big)}. If this is a bug hunt "
        "or a review, it is not finished: read these files end to end the way "
        ".claude/skills/rosio/review.md asks, handing groups of packages to subagents when you "
        "can start them, and add each bug you confirm to the report. Otherwise finish now."
    )}))


def main() -> int:
    try:
        _run()
    except Exception as exc:
        print(f"rosio stop hook skipped: {' '.join(str(exc).splitlines())}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    main()
