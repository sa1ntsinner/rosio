"""Stop hook: a session that changed code is told the findings its edits added and reviews its
diff once against coding.md, and one that only read is nudged toward the packages it never
opened."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

from rosio import verify
from rosio.agent import push
from rosio.model import dumps_line

SKIP = {".git", "rosio-out", "test", "tests", "build", "install", "log", "node_modules"}
SRC = {".cpp", ".hpp", ".h", ".cc", ".py"}
MARK = "rosio: this session opened"
NUDGES = 3
REVIEW = "rosio: this session changed"
EDITS = {"Edit", "Write", "MultiEdit", "NotebookEdit"}


def sources(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file()
        and path.suffix in SRC
        and path.name != "setup.py"
        and not SKIP.intersection(path.relative_to(root).parts)
    )


def packages(root: Path) -> list[Path]:
    return sorted(
        xml.parent
        for xml in root.rglob("package.xml")
        if not SKIP.intersection(xml.relative_to(root).parts) and sources(xml.parent)
    )


OWN = (".claude", ".codex", ".agents", "rosio-out")


def calls(transcript: str | None):
    """Each tool call of a session as (name, input), from a Claude Code transcript or a Codex
    rollout, whose `arguments` or `input` is the call's text."""
    if not transcript or not Path(transcript).is_file():
        return
    with Path(transcript).open(encoding="utf-8") as stream:
        for line in stream:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if message.get("type") == "assistant":
                for content in message.get("message", {}).get("content", []):
                    if content.get("type") == "tool_use":
                        yield content.get("name"), content.get("input", {})
            elif message.get("type") == "response_item":
                item = message.get("payload") or {}
                if item.get("type") in ("function_call", "custom_tool_call"):
                    yield item.get("name"), item.get("arguments") or item.get("input") or ""


def edited(transcript: str | None, root: Path) -> list[str]:
    paths = set()
    for name, args in calls(transcript):
        if name in EDITS and isinstance(args, dict):
            path = Path(args.get("file_path", ""))
            try:
                rel = path.resolve().relative_to(root.resolve())
            except ValueError:
                continue
            if rel.parts and rel.parts[0] not in OWN and rel.suffix != ".md":
                paths.add(rel.as_posix())
    # agents also edit through the shell, so a changed file the session named counts too, and
    # every changed file when there is no transcript to name it
    seen = touched(transcript)
    git = ["git", "-C", str(root)]
    changed = ["diff", "--name-only", "--relative", "HEAD"]
    for cmd in (changed, ["ls-files", "--others", "--exclude-standard"]):
        try:
            out = subprocess.run(git + cmd, capture_output=True, text=True, timeout=20).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        for rel in out.splitlines():
            parts = Path(rel).parts
            if (
                parts
                and parts[0] not in OWN
                and Path(rel).suffix != ".md"
                and Path(rel).name not in ("CLAUDE.md", ".mcp.json", "rosio.yaml")
                and (seen is None or Path(rel).name in seen)
            ):
                paths.add(Path(rel).as_posix())
    return sorted(paths)


def touched(transcript: str | None) -> str | None:
    """The text of every tool call, or None when there is no transcript to read."""
    if not transcript or not Path(transcript).is_file():
        return None
    text = [args if isinstance(args, str) else dumps_line(args) for _, args in calls(transcript)]
    return "\n".join(text)


def _run() -> None:
    data = json.load(sys.stdin)
    transcript = data.get("transcript_path")
    # a transcript records one hook message several times, so count by its number
    held = transcript and Path(transcript).is_file()
    text = Path(transcript).read_text(encoding="utf-8") if held else ""
    said = re.findall(r"\(reminder (\d+) of", text)
    nudges = max(map(int, said), default=0)
    if nudges >= NUDGES or (data.get("stop_hook_active") and not nudges):
        return
    # only Codex hands a turn id, and it reads the skill files from the portable directory
    codex = "turn_id" in data
    skill = ".agents/skills/rosio" if codex else ".claude/skills/rosio"
    root = Path((not codex and os.environ.get("CLAUDE_PROJECT_DIR")) or data.get("cwd") or ".")
    changed = edited(transcript, root)
    if changed:
        try:
            measured = (
                ""
                if data.get("stop_hook_active")
                else push.stop_check(root, changed, text, push.stale_at_start(data))
            )
        except Exception as exc:
            measured = ""
            print(f"rosio stop check skipped: {' '.join(str(exc).splitlines())}", file=sys.stderr)
        parts = [measured] if measured else []
        if REVIEW not in text:
            parts.append(
                f"{REVIEW} {', '.join(changed[:10])}. Before you finish, read "
                f"{skill}/coding.md and check the diff against it: every path that "
                "hits this same defect is fixed, and another defect you found is named in the "
                "report rather than fixed here; a small deterministic test, no sleeps or timing, "
                "that runs the fixed line and fails without the change, every name it calls "
                "checked with rg and its build lines copied from a neighbouring test (no test "
                "only when it would need a public header or signature changed, or the bug is a "
                "race it cannot reach); comments of one line that say why and none that restate "
                "the code; nothing unrelated touched; and the package built and the test run "
                "when a toolchain is here. Fix what falls short, then finish. In the report "
                "write only what a command printed in this session, and not run for every step "
                "you did not run: a false claim is worse than not run. Do not recite this list."
            )
            built = verify.touched_packages(root, [root / path for path in changed])
            if built:
                parts.append(
                    f"rosio: packages changed {', '.join(built)}. Run rosio verify to build and "
                    "test them before saying the change works."
                )
        if parts:
            print(dumps_line({"decision": "block", "reason": "\n\n".join(parts)}))
        return
    seen = touched(transcript)
    if seen is None:
        return
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
    names = ", ".join(path.relative_to(root).as_posix() for path in big)
    reason = (
        f"{MARK} {read} of the {total} source files in this repository "
        f"(reminder {nudges + 1} of {NUDGES}). Per package, files opened of files held: "
        f"{'; '.join(rows[:40])}. Large files not opened: {names}. If this is a bug hunt "
        "or a review, it is not finished: read these files end to end the way "
        f"{skill}/review.md asks, handing groups of packages to subagents when you "
        "can start them, and add each bug you confirm to the report. Otherwise finish now."
    )
    print(dumps_line({"decision": "block", "reason": reason}))


def main() -> int:
    try:
        _run()
    except Exception as exc:
        print(f"rosio stop hook skipped: {' '.join(str(exc).splitlines())}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    main()
