"""The table of assistants rosio can install itself into, one row per assistant.

The mechanisms cover files the tool reads, MCP registration and a Stop hook:

    skill         a self contained file the tool loads on demand, ours end to end
    reference     a file beside the skill that the skill tells the reader to open
    rules         a self contained rules file, ours end to end
    instructions  a managed section inside a file the user also writes in
    mcp           a server entry merged into a json or toml config the user also writes in
    hook          a Stop command merged into the tool's json settings

Proof is per scope, because a target carries a project path and a user path and those are two
claims. `proven` lists the scopes where somebody started that tool and watched it read the
file at that path, or wrote the file with that tool's own command and read it back. A path
taken from documentation, however good the documentation, is absent from `proven` with the
reason in `note`. It is still installable, because writing one file into a directory the tool
may read costs nothing, but the caller is told rather than sold a guess. `verified` is derived
and stays true only while every scope a target offers is proven, so a half proven row prints
as unverified with the note naming the half that holds. `proof` states the same thing in one
phrase and is carried in the table, because a bool over two paths cannot say that one of them
was watched and the other was read in a manual.

Four rows carry proof today, all against a tool installed on the machine that wrote this
table: two Claude Code rows and two Codex CLI rows, and on the user scope in every case. Every
other row is somebody reading a manual.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from rosio.errors import UsageError

MECHANISMS = ("hook", "instructions", "mcp", "reference", "rules", "skill")
SCOPES = ("project", "user")

SKILL_NAME = "rosio"
SERVE_ARGS = ("-m", "rosio", "serve")
"""stdio is the default transport of `rosio serve`, so the registration names no transport
flag. At user scope the interpreter that runs install starts the server, by its absolute path:
an assistant started outside the virtual environment has no `rosio` on its PATH."""


def serve_command(atlas: Path | None = None) -> tuple[str, ...]:
    """The command an assistant runs to start this rosio, on `atlas` when one is named."""
    return (sys.executable, *SERVE_ARGS, *(("--atlas", str(atlas)) if atlas else ()))


def project_command() -> tuple[str, ...]:
    """The command a committed project file names: `rosio` from the PATH, on the project's
    atlas by its path inside the project, which is the directory an MCP client starts its
    servers in. This machine's interpreter or an absolute path fails in every other clone."""
    from rosio.config import DEFAULT_OUT_DIR
    from rosio.extract import ATLAS_NAME

    return ("rosio", "serve", "--atlas", f"{DEFAULT_OUT_DIR}/{ATLAS_NAME}")


@dataclass(frozen=True)
class Target:
    """One file one platform reads, and how rosio writes into it."""

    mechanism: str
    project: str = ""
    user: str = ""
    template: str = ""
    fmt: str = "md"
    container: str = ""
    entry_style: str = "command_args"
    proven: tuple[str, ...] = ()
    note: str = ""

    def path_for(self, scope: str) -> str:
        return self.project if scope == "project" else self.user

    def proven_at(self, scope: str) -> bool:
        return scope in self.proven

    @property
    def offered(self) -> tuple[str, ...]:
        return tuple(scope for scope in SCOPES if self.path_for(scope))

    @property
    def verified(self) -> bool:
        """True only when every scope this target offers was watched."""
        paths = self.offered
        return bool(paths) and all(scope in self.proven for scope in paths)

    @property
    def proof(self) -> str:
        """Which scopes were demonstrated, in one phrase, because a bool over two paths
        cannot say that one of them was watched and the other was read in a manual."""
        watched = [scope for scope in self.offered if self.proven_at(scope)]
        rest = [scope for scope in self.offered if not self.proven_at(scope)]
        if not watched:
            return "from the tool's documentation"
        seen = f"seen read at {' and '.join(watched)} scope"
        if not rest:
            return seen
        return f"{seen}, {' and '.join(rest)} scope from the tool's documentation"

    @property
    def writer(self) -> str:
        if self.mechanism in ("skill", "rules", "reference"):
            return "whole_file"
        if self.mechanism == "instructions":
            return "section"
        return "json_merge" if self.fmt == "json" else "toml_section"


@dataclass(frozen=True)
class Platform:
    """One assistant, and every file installing into it writes."""

    key: str
    title: str
    targets: tuple[Target, ...] = field(default_factory=tuple)
    note: str = ""

    @property
    def verified(self) -> bool:
        return all(target.verified for target in self.targets)

    def targets_for(self, scope: str) -> tuple[Target, ...]:
        return tuple(target for target in self.targets if target.path_for(scope))


_SKILL = f".claude/skills/{SKILL_NAME}/SKILL.md"
_PORTABLE_SKILL = f".agents/skills/{SKILL_NAME}/SKILL.md"
_REVIEW = f".claude/skills/{SKILL_NAME}/review.md"
_PORTABLE_REVIEW = f".agents/skills/{SKILL_NAME}/review.md"


PLATFORMS: tuple[Platform, ...] = (
    Platform(
        key="agent-skills",
        title="portable agent skills directory",
        targets=(
            Target(
                mechanism="skill",
                project=_PORTABLE_SKILL,
                user=_PORTABLE_SKILL,
                template="skill.md",
                note="the cross framework skills directory several tools now read, same "
                "SKILL.md layout as the Claude Code skill",
            ),
            Target(
                mechanism="reference",
                project=_PORTABLE_REVIEW,
                user=_PORTABLE_REVIEW,
                template="review.md",
                note="the bug classes of ROS 2 code, read beside the skill when hunting bugs",
            ),
        ),
    ),
    Platform(
        key="aider",
        title="Aider",
        targets=(
            Target(
                mechanism="instructions",
                project="CONVENTIONS.md",
                template="instructions.md",
                note="aider reads this file only when it is named in .aider.conf.yml as "
                "'read: CONVENTIONS.md' or passed as --read CONVENTIONS.md, and install "
                "does not edit that config",
            ),
        ),
    ),
    Platform(
        key="amp",
        title="Amp",
        targets=(
            Target(
                mechanism="instructions",
                project="AGENTS.md",
                template="instructions.md",
                note="AGENTS.md is the documented convention, and no Amp install has read "
                "this file back",
            ),
        ),
        note="the MCP registration lives in the editor extension settings, which install "
        "does not write",
    ),
    Platform(
        key="claude-code",
        title="Claude Code",
        targets=(
            Target(
                mechanism="hook",
                project=".claude/settings.json",
                user=".claude/settings.json",
                fmt="json",
                container="hooks",
                note="the Stop hook has not been proven in a running Claude Code session",
            ),
            Target(
                mechanism="skill",
                project=_SKILL,
                user=_SKILL,
                template="skill.md",
                proven=("user",),
                note="the user scope path was watched being read by a running install, the "
                "project scope path carries the same layout and was not",
            ),
            Target(
                mechanism="reference",
                project=_REVIEW,
                user=_REVIEW,
                template="review.md",
                note="the bug classes of ROS 2 code, which the skill names by this path",
            ),
            Target(
                mechanism="instructions",
                project="CLAUDE.md",
                user=".claude/CLAUDE.md",
                template="instructions.md",
                proven=("user",),
                note="the user scope file was watched being read, no project scope file has "
                "been read back",
            ),
            Target(
                mechanism="mcp",
                project=".mcp.json",
                fmt="json",
                container="mcpServers",
                note="the mcpServers key is confirmed from a real user config, the project "
                "scoped .mcp.json path is not, and no project file has been read back",
            ),
        ),
        note="user scope MCP lives in ~/.claude.json beside machine state, so install "
        "writes the project .mcp.json only",
    ),
    Platform(
        key="codex",
        title="Codex CLI",
        targets=(
            Target(
                mechanism="reference",
                project=_PORTABLE_REVIEW,
                user=_PORTABLE_REVIEW,
                template="review.md",
                note="the bug classes of ROS 2 code, which the AGENTS.md section names by this "
                "path",
            ),
            Target(
                mechanism="instructions",
                project="AGENTS.md",
                user=".codex/AGENTS.md",
                template="instructions.md",
                proven=("user",),
                note="the user scope file was watched being read, no project scope file has "
                "been read back",
            ),
            Target(
                mechanism="mcp",
                user=".codex/config.toml",
                fmt="toml",
                container="mcp_servers",
                proven=("user",),
            ),
        ),
    ),
    Platform(
        key="copilot",
        title="GitHub Copilot",
        targets=(
            Target(
                mechanism="instructions",
                project=".github/copilot-instructions.md",
                template="instructions.md",
                note="path taken from the Copilot documentation, and no Copilot install "
                "has read this file back",
            ),
            Target(
                mechanism="mcp",
                project=".vscode/mcp.json",
                fmt="json",
                container="servers",
                note="workspace MCP file of the editor, not of the CLI",
            ),
        ),
    ),
    Platform(
        key="cursor",
        title="Cursor",
        targets=(
            Target(
                mechanism="rules",
                project=f".cursor/rules/{SKILL_NAME}.mdc",
                template="rules.mdc",
                fmt="mdc",
                note="rules directory and the mdc frontmatter come from the Cursor "
                "documentation, and no Cursor install has loaded this rule",
            ),
            Target(
                mechanism="mcp",
                project=".cursor/mcp.json",
                user=".cursor/mcp.json",
                fmt="json",
                container="mcpServers",
                note="path and container taken from the Cursor documentation, and no "
                "Cursor install has read this entry back",
            ),
        ),
        note="user scope rules live in the application settings and not in a file, so "
        "install writes the project rule only",
    ),
    Platform(
        key="gemini-cli",
        title="Gemini CLI",
        targets=(
            Target(
                mechanism="instructions",
                project="GEMINI.md",
                user=".gemini/GEMINI.md",
                template="instructions.md",
                note="paths taken from the Gemini CLI documentation, and no Gemini CLI "
                "install has read this file back",
            ),
            Target(
                mechanism="mcp",
                project=".gemini/settings.json",
                user=".gemini/settings.json",
                fmt="json",
                container="mcpServers",
                note="settings file and container taken from the Gemini CLI "
                "documentation, and no Gemini CLI install has read this entry back",
            ),
        ),
    ),
    Platform(
        key="kilo",
        title="Kilo Code",
        targets=(
            Target(
                mechanism="rules",
                project=f".kilocode/rules/{SKILL_NAME}.md",
                user=f".kilocode/rules/{SKILL_NAME}.md",
                template="instructions.md",
                note="rules directory read per workspace, layout taken from the project "
                "documentation and not verified against a running install",
            ),
            Target(
                mechanism="mcp",
                project=".kilocode/mcp.json",
                fmt="json",
                container="mcpServers",
                note="workspace MCP file taken from the project documentation and not "
                "verified against a running install",
            ),
        ),
    ),
    Platform(
        key="opencode",
        title="OpenCode",
        targets=(
            Target(
                mechanism="instructions",
                project="AGENTS.md",
                user=".config/opencode/AGENTS.md",
                template="instructions.md",
                note="paths taken from the OpenCode documentation, and no OpenCode "
                "install has read this file back",
            ),
            Target(
                mechanism="mcp",
                project="opencode.json",
                user=".config/opencode/opencode.json",
                fmt="json",
                container="mcp",
                entry_style="local_command",
                note="server entry shape taken from the project documentation and not "
                "verified against a running install",
            ),
        ),
    ),
)


def platforms() -> tuple[Platform, ...]:
    return tuple(sorted(PLATFORMS, key=lambda p: p.key))


def keys() -> tuple[str, ...]:
    return tuple(platform.key for platform in platforms())


def get(key: str) -> Platform:
    for platform in platforms():
        if platform.key == key:
            return platform
    raise UsageError(f"unknown platform {key!r}, one of: {', '.join(keys())}")


def check_scope(scope: str) -> str:
    if scope not in SCOPES:
        raise UsageError(f"unknown scope {scope!r}, one of: {', '.join(SCOPES)}")
    return scope


def server_entry(target: Target, command: Sequence[str]) -> dict:
    """The MCP server object this target's config format expects, for one command line."""
    program, *args = command
    if target.entry_style == "local_command":
        return {"type": "local", "command": [program, *args], "enabled": True}
    if target.fmt == "toml":
        return {"command": program, "args": args}
    return {"type": "stdio", "command": program, "args": args}


def table() -> list[dict]:
    """The platform table as plain rows, for printing and for tests."""
    rows = []
    for platform in platforms():
        for target in platform.targets:
            rows.append(
                {
                    "platform": platform.key,
                    "mechanism": target.mechanism,
                    "project": target.project,
                    "user": target.user,
                    "verified": target.verified,
                    "proven": list(target.proven),
                    "proof": target.proof,
                    "note": target.note or platform.note,
                }
            )
    return rows


__all__ = [
    "MECHANISMS",
    "PLATFORMS",
    "SCOPES",
    "SERVE_ARGS",
    "SKILL_NAME",
    "Platform",
    "Target",
    "check_scope",
    "get",
    "keys",
    "platforms",
    "project_command",
    "serve_command",
    "server_entry",
    "table",
]
