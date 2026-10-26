"""Portable evidence for a problem somebody else needs to inspect.

The atlas stays byte identical unless removing environment or secret values is necessary.
Source redaction keeps line counts, so the atlas still points at the line to edit.
"""

from __future__ import annotations

import ast
import json
import re
import shlex
import stat
import subprocess
import zipfile
from collections import Counter
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any

import yaml

from rosio import __version__, ids
from rosio.config import CONFIG_NAME, DEFAULT_OUT_DIR, load_config
from rosio.errors import ContractError, UsageError
from rosio.model import Atlas, Finding, dumps
from rosio.query import site
from rosio.render.drift_md import render_drift
from rosio.render.report import render_report
from rosio.workspace import Workspace

MAX_FILES = 2000
MAX_BYTES = 50 * 1024 * 1024
MAX_DIFF_BYTES = 5 * 1024 * 1024
OPEN_MAX_MEMBERS = MAX_FILES + 16
OPEN_MAX_BYTES = 4 * MAX_BYTES
SECRET = re.compile(r"password|passwd|secret|token|api[_-]?key|private[_-]?key", re.I)
ENV_KEYS = {"env", "environment", "environ", "additional_env"}
ENV_PREDICATE = re.compile(r"env:(?P<name>[^=]+)=(?P<value>.*)", re.S)
ASSIGNMENT = re.compile(r"(?P<name>[\w.-]+):=(?P<value>\"[^\"\n]*\"|'[^'\n]*'|[^\s,\]\)]+)")
LAUNCH_DEFAULT = re.compile(
    r"DeclareLaunchArgument\(\s*['\"](?P<name>[^'\"]+)['\"][^)]*?"
    r"default_value\s*=\s*(?P<value>'[^'\n]*'|\"[^\"\n]*\")",
    re.S,
)


def _safe_name(name: str) -> bool:
    path = PurePosixPath(name)
    return bool(name) and not (
        path.as_posix().rstrip("/") != name.rstrip("/")
        or name == "."
        or path.is_absolute()
        or PureWindowsPath(name).drive
        or "\\" in name
        or ".." in path.parts
        or ":" in name
        or "\x00" in name
        or any(part != part.rstrip(". ") for part in path.parts)
    )


def _sources(value: Any) -> set[str]:
    if isinstance(value, dict):
        found = {value["file"]} if str(value.get("file", "")).startswith("file:") else set()
        return found.union(*(_sources(child) for child in value.values()))
    if isinstance(value, list):
        return set().union(*(_sources(child) for child in value))
    return set()


def _remember(value: Any, secrets: set[str] | None) -> None:
    if secrets is None:
        return
    if isinstance(value, str) and value and value != "<redacted>":
        secrets.add(value)
    elif isinstance(value, dict):
        for child in value.values():
            _remember(child, secrets)
    elif isinstance(value, list):
        for child in value:
            _remember(child, secrets)


def _mask(value: Any, secrets: set[str], counts: Counter, field: str = "") -> Any:
    if isinstance(value, dict):
        return {key: _mask(child, secrets, counts, key) for key, child in value.items()}
    if isinstance(value, list):
        return [_mask(child, secrets, counts, field) for child in value]
    if isinstance(value, str):
        if value in secrets:
            counts["values"] += 1
            return "<redacted>"
        if field in {"title", "detail", "fix", "say", "expr", "raw"}:
            for secret in sorted(secrets, key=lambda item: (-len(item), item)):
                counts["values"] += value.count(secret)
                value = value.replace(secret, "<redacted>")
    return value


def _clean(
    value: Any, counts: Counter, sensitive: bool = False, secrets: set[str] | None = None
) -> Any:
    if isinstance(value, dict):
        result = {}
        sensitive = sensitive or bool(SECRET.search(str(value.get("name", ""))))
        if str(value.get("id", "")).startswith("param:"):
            sensitive = bool(SECRET.search(str(value.get("label", ""))))
        for key, child in value.items():
            if key in ENV_KEYS and child:
                counts["environment_blocks"] += 1
                continue
            read = ENV_PREDICATE.fullmatch(key)
            if read:
                # The launch walk keys an environment read by its value, and an environment
                # block leaves the bundle whole, so the value of this one goes too.
                if SECRET.search(read["name"]):
                    _remember(read["value"], secrets)
                result.setdefault(f"env:{read['name']}=<redacted>", child)
                counts["values"] += 1
                continue
            if (":" not in key and SECRET.search(key) and not isinstance(child, (dict, list))) or (
                sensitive and key in {"value", "default"}
            ):
                _remember(child, secrets)
                result[key] = "<redacted>"
                counts["values"] += 1
            else:
                result[key] = _clean(child, counts, sensitive, secrets)
        return result
    if isinstance(value, list):
        return [_clean(child, counts, sensitive, secrets) for child in value]
    if isinstance(value, str):

        def replace(match: re.Match) -> str:
            if not SECRET.search(match["name"]) or match["value"].strip("'\"") == "<redacted>":
                return match[0]
            counts["values"] += 1
            _remember(match["value"].strip("'\""), secrets)
            return match["name"] + ":=<redacted>"

        return ASSIGNMENT.sub(replace, value)
    return value


def redact(text: str, counts: Counter, secrets: set[str] | None = None) -> str:
    """Redact matching YAML keys, literal launch defaults and name:=value arguments."""
    spans = []
    try:
        document = yaml.compose(text)
    except yaml.YAMLError:
        document = None

    def walk(node: Any, seen: set[int]) -> None:
        if node is None or id(node) in seen:
            return
        seen.add(id(node))
        if isinstance(node, yaml.MappingNode):
            sensitive = any(
                isinstance(key, yaml.ScalarNode)
                and key.value == "name"
                and isinstance(value, yaml.ScalarNode)
                and SECRET.search(value.value)
                for key, value in node.value
            )
            for key, value in node.value:
                if isinstance(key, yaml.ScalarNode) and (
                    SECRET.search(key.value)
                    or key.value in ENV_KEYS
                    or (sensitive and key.value in {"default", "default_value", "value"})
                ):
                    spans.append((value.start_mark.index, value.end_mark.index))
                    if isinstance(value, yaml.ScalarNode):
                        _remember(value.value, secrets)
                else:
                    walk(value, seen)
        elif isinstance(node, yaml.SequenceNode):
            for child in node.value:
                walk(child, seen)

    walk(document, set())
    try:
        tree = ast.parse(text)
    except (SyntaxError, ValueError):
        tree = None
    lines = text.splitlines(keepends=True)
    offsets = [0]
    for line in lines:
        offsets.append(offsets[-1] + len(line))

    def index(line: int, column: int) -> int:
        return offsets[line - 1] + len(lines[line - 1].encode()[:column].decode())

    for node in ast.walk(tree) if tree else ():
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        name = function.id if isinstance(function, ast.Name) else getattr(function, "attr", "")
        for keyword in node.keywords:
            if keyword.arg in ENV_KEYS:
                value = keyword.value
                spans.append(
                    (
                        index(value.lineno, value.col_offset),
                        index(value.end_lineno, value.end_col_offset),
                    )
                )
        if name == "SetEnvironmentVariable":
            value = (
                node.args[1]
                if len(node.args) > 1
                else next((kw.value for kw in node.keywords if kw.arg == "value"), None)
            )
            if value is not None:
                spans.append(
                    (
                        index(value.lineno, value.col_offset),
                        index(value.end_lineno, value.end_col_offset),
                    )
                )
        if name != "DeclareLaunchArgument":
            continue
        argument = (
            node.args[0]
            if node.args
            else next((kw.value for kw in node.keywords if kw.arg == "name"), None)
        )
        if not isinstance(argument, ast.Constant) or not SECRET.search(str(argument.value)):
            continue
        default = next((kw.value for kw in node.keywords if kw.arg == "default_value"), None)
        if default is not None:
            for literal in ast.walk(default):
                if isinstance(literal, ast.Constant):
                    _remember(literal.value, secrets)
            spans.append(
                (
                    index(default.lineno, default.col_offset),
                    index(default.end_lineno, default.end_col_offset),
                )
            )
    merged: list[tuple[int, int]] = []
    for start, end in sorted(set(spans)):
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
        else:
            merged.append((start, end))
    for start, end in reversed(merged):
        text = text[:start] + '"<redacted>"' + "\n" * text[start:end].count("\n") + text[end:]
        counts["values"] += 1

    def launch_default(match: re.Match) -> str:
        if not SECRET.search(match["name"]) or match["value"].strip("'\"") == "<redacted>":
            return match[0]
        counts["values"] += 1
        _remember(match["value"].strip("'\""), secrets)
        return match[0][: match.start("value") - match.start()] + '"<redacted>"'

    text = LAUNCH_DEFAULT.sub(launch_default, text)
    for match in reversed(list(re.finditer(r"<(?:arg|argument)\b[^>]*>", text, re.S))):
        tag = match[0]
        name = re.search(r"\bname\s*=\s*['\"]([^'\"]+)['\"]", tag)
        if name and SECRET.search(name[1]):
            tag, changed = re.subn(
                r"\b(default|value)\s*=\s*(['\"])[^'\"]*['\"]",
                lambda value: value[1] + '="&lt;redacted&gt;"' + "\n" * value[0].count("\n"),
                tag,
            )
            counts["values"] += changed
            text = text[: match.start()] + tag + text[match.end() :]
    return _clean(text, counts, secrets=secrets)


def _portable(text: str, workspace: Workspace) -> str:
    paths = {workspace.repo_root, *(root.path for root in workspace.roots)}
    for path in sorted(paths, key=lambda item: (-len(item), item)):
        for spelling in sorted({path, Path(path).as_posix(), str(path).replace("\\", "\\\\")}):
            text = text.replace(spelling, ".")
    return text


def _redact_diff(text: str, counts: Counter, secrets: set[str] | None = None) -> str:
    lines = text.splitlines(keepends=True)
    start = 0
    for end in range(len(lines) + 1):
        if (
            end < len(lines)
            and lines[end][:1] in {"+", "-", " "}
            and not lines[end].startswith(("+++", "---"))
        ):
            continue
        if start < end:
            prefixes = [line[0] for line in lines[start:end]]
            content = "".join(line[1:] for line in lines[start:end])
            redacted = redact(content, counts, secrets).splitlines(keepends=True)
            lines[start:end] = [
                prefix + line for prefix, line in zip(prefixes, redacted, strict=True)
            ]
        start = end + 1
    return "".join(lines)


def _git(workspace: Path, counts: Counter, secrets: set[str]) -> tuple[dict, str]:
    def run(*args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["git", "-C", str(workspace), *args], capture_output=True, check=False
        )

    try:
        commit = run("rev-parse", "HEAD")
        if commit.returncode:
            return {}, ""
        branch = run("symbolic-ref", "--quiet", "--short", "HEAD")
        paths = (
            "--",
            ":(top,glob,exclude)**/*.rosio.zip",
            ":(top,exclude)*.rosio.zip",
            f":(top,exclude){DEFAULT_OUT_DIR}",
        )
        dirty = run("status", "--porcelain", "--untracked-files=all", *paths)
        diff = run("diff", "HEAD", "--no-ext-diff", "--no-textconv", "--no-color", *paths)
    except OSError:
        return {}, ""
    text = diff.stdout.decode("utf-8", errors="replace")
    blocks = re.split(r"(?=^diff --git )", text, flags=re.M)
    kept = []
    size = 0
    omitted = []
    for block in blocks:
        if not block:
            continue
        reason = "binary" if "Binary files " in block or "GIT binary patch" in block else ""
        if not reason:
            block = _redact_diff(block, counts, secrets)
        length = len(block.encode())
        if not reason and size + length > MAX_DIFF_BYTES:
            reason = "over_cap"
        if reason:
            omitted.append({"file": block.splitlines()[0], "reason": reason})
        else:
            kept.append(block)
            size += length
    return {
        "commit": commit.stdout.decode().strip(),
        "branch": branch.stdout.decode().strip(),
        "dirty": bool(dirty.stdout),
        "diff_omitted": omitted,
    }, "".join(kept)


def _member(file_id: str) -> str:
    package, relative = ids.parse_file(file_id)
    return f"files/{relative}" if package == ids.REPO_PKG else f"files/{package}/{relative}"


def _wanted(original: Any, loaded: Atlas, include: str) -> set[str]:
    """The file ids to carry: what the findings cite, or every cited file, with manifests."""
    wanted = (
        _sources(original)
        if include == "cited"
        else {row.file for finding in loaded.findings for row in finding.where}
    )
    for file_id in list(wanted):
        package = ids.parse_file(file_id).package
        if package != ids.REPO_PKG:
            wanted.add(ids.build_file(package, "package.xml"))
    return wanted


def _local(ws: Workspace, file_id: str) -> tuple[Path | None, str]:
    """Where a cited file sits on this machine, or why it cannot be carried."""
    if not _safe_name(_member(file_id)):
        return None, "outside_every_root"
    try:
        local = Path(ws.abspath(file_id)).resolve()
    except ContractError:
        return None, "missing"
    if not any(local.is_relative_to(Path(root.path).resolve()) for root in ws.roots):
        return None, "outside_every_root"
    if not local.is_file():
        return None, "missing"
    return local, ""


def _files(
    ws: Workspace, wanted: set[str], counts: Counter, secrets: set[str]
) -> tuple[dict[str, bytes], list[dict], list[dict]]:
    """The cited files, redacted and made portable, with every file left out and why."""
    members: dict[str, bytes] = {}
    included: list[dict] = []
    omitted: list[dict] = []
    size = 0
    for file_id in sorted(wanted):
        local, reason = _local(ws, file_id)
        if local is not None and (
            len(included) >= MAX_FILES or size + local.stat().st_size > MAX_BYTES
        ):
            reason = "over_cap"
        if local is None or reason:
            omitted.append({"file": file_id, "reason": reason})
            continue
        try:
            content = local.read_bytes()
        except OSError:
            omitted.append({"file": file_id, "reason": "missing"})
            continue
        try:
            content = _portable(redact(content.decode("utf-8"), counts, secrets), ws).encode()
        except UnicodeDecodeError:
            if local.suffix.lower() in {".yaml", ".yml", ".py", ".xml", ".launch"}:
                omitted.append({"file": file_id, "reason": "unreadable_text"})
                continue
            for prefix in sorted({ws.repo_root, *(root.path for root in ws.roots)}):
                content = content.replace(prefix.encode(), b".")
        if size + len(content) > MAX_BYTES:
            omitted.append({"file": file_id, "reason": "over_cap"})
            continue
        member = _member(file_id)
        members[member] = content
        size += len(content)
        included.append({"file": file_id, "path": member})
    return members, included, omitted


def _config(base: Path, ws: Workspace, counts: Counter, secrets: set[str]) -> bytes | None:
    """The sender's configuration with `files` as its only root and no setup scripts."""
    path = next((p / CONFIG_NAME for p in (base, base.parent) if (p / CONFIG_NAME).is_file()), None)
    if path is None:
        return None
    config = _clean(yaml.safe_load(path.read_text(encoding="utf-8")), counts, secrets=secrets)
    config["roots"] = [{"kind": "src", "path": "files"}]
    config.pop("ros_setup", None)
    return _portable(yaml.safe_dump(config, sort_keys=True), ws).encode()


def _reports(raw: bytes) -> tuple[Atlas, dict[str, bytes]]:
    loaded = Atlas.loads(raw.decode())
    return loaded, {
        "atlas.json": raw,
        "DRIFT.md": render_drift(loaded).encode(),
        "ATLAS_REPORT.md": render_report(loaded).encode(),
    }


def _write(target: Path, members: dict[str, bytes]) -> None:
    """Sorted members, a fixed timestamp and fixed modes, so one state is one archive."""
    target.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, content in sorted(members.items()):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            archive.writestr(info, content, compress_type=zipfile.ZIP_DEFLATED)


def create(
    path: str | Path = ".",
    *,
    atlas: str | Path | None = None,
    out: str | Path | None = None,
    include: str = "findings",
    note: str = "",
) -> Path:
    """Collect cited files without executing or extracting the workspace again."""
    if include not in {"findings", "cited"}:
        raise UsageError("bundle include must be findings or cited")
    base = Path(path).expanduser().resolve()
    target = Path(out) if out else base / DEFAULT_OUT_DIR / "bundle.rosio.zip"
    source = Path(atlas) if atlas else base / DEFAULT_OUT_DIR / "atlas.json"
    if source.is_dir():
        source /= "atlas.json"
    try:
        raw = source.read_bytes()
        Atlas.loads(raw.decode("utf-8"))
        cfg = load_config(None, workspace=str(base))
        ws = Workspace.discover(cfg.roots, cfg.ignore)
        counts: Counter = Counter()
        secrets: set[str] = set()
        original = json.loads(raw)
        cleaned = _clean(original, counts, secrets=secrets)
        if cleaned != original:
            raw = dumps(cleaned).encode()
        loaded, members = _reports(_portable(raw.decode(), ws).encode())

        files, included, omitted = _files(ws, _wanted(original, loaded, include), counts, secrets)
        members.update(files)
        config = _config(base, ws, counts, secrets)
        if config is not None:
            members[CONFIG_NAME] = config
        revision, diff = _git(base, counts, secrets)
        if revision:
            members["changes.diff"] = _portable(diff, ws).encode()

        # A secret first met in a cited file can still sit in a finding's prose.
        payload = json.loads(members["atlas.json"])
        masked = _mask(payload, secrets, counts)
        if masked != payload:
            loaded, reports = _reports(dumps(masked).encode())
            members.update(reports)

        manifest = {
            "rosio_version": __version__,
            "schema_version": loaded.schema_version,
            "finding_counts": dict(sorted(Counter(f.severity for f in loaded.findings).items())),
            "profiles": [p.id for p in loaded.by_kind("profile")],
            "layers": sorted(k for k, v in loaded.layers.items() if v.present),
            "note": _clean(note, counts),
            "files": included,
            "omitted": omitted,
            "revision": revision,
            "limits": {
                "files": MAX_FILES,
                "bytes": MAX_BYTES,
                "diff_bytes": MAX_DIFF_BYTES,
                "hit": any(
                    row["reason"] == "over_cap"
                    for row in [*omitted, *revision.get("diff_omitted", [])]
                ),
            },
            "redactions": dict(sorted(counts.items())),
        }
        members["manifest.json"] = _portable(dumps(manifest), ws).encode()
        _write(target, members)
        return target
    except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
        raise UsageError(f"cannot create bundle: {exc}") from exc


def open_bundle(file: str | Path, *, into: str | Path | None = None, force: bool = False) -> str:
    """Validate all members before writing any of them, including existing symlinks."""
    source = Path(file).expanduser()
    default = source.name.removesuffix(".rosio.zip")
    if default == source.name:
        default = source.stem + ".rosio"
    target = Path(into) if into else source.with_name(default)
    try:
        with zipfile.ZipFile(source) as archive:
            names = archive.namelist()
            # Windows folds case and drops trailing dots and spaces, so two names that differ
            # only there would land on one file after the first was validated.
            if len({name.casefold() for name in names}) != len(names):
                raise UsageError("bundle contains duplicate members")
            if len(names) > OPEN_MAX_MEMBERS:
                raise UsageError(f"bundle holds more than {OPEN_MAX_MEMBERS} members")
            # zipfile stops each member at its declared size, so the header sum is a bound.
            if sum(info.file_size for info in archive.infolist()) > OPEN_MAX_BYTES:
                raise UsageError(f"bundle expands past {OPEN_MAX_BYTES} bytes")
            for info in archive.infolist():
                destination = target / info.filename
                if (
                    not _safe_name(info.filename)
                    or stat.S_ISLNK(info.external_attr >> 16)
                    or not destination.resolve().is_relative_to(target.resolve())
                ):
                    raise UsageError(f"unsafe bundle member: {info.filename}")
            if not {"manifest.json", "atlas.json"}.issubset(names):
                raise UsageError("bundle needs manifest.json and atlas.json")
            manifest = json.loads(archive.read("manifest.json"))
            if not isinstance(manifest, dict):
                raise UsageError("bundle manifest must be a mapping")
            if not isinstance(manifest.get("finding_counts", {}), dict):
                raise UsageError("bundle finding_counts must be a mapping")
            if not isinstance(manifest.get("revision", {}), dict):
                raise UsageError("bundle revision must be a mapping")
            atlas = Atlas.loads(archive.read("atlas.json").decode())
            if target.exists() and (not target.is_dir() or (any(target.iterdir()) and not force)):
                raise UsageError("bundle destination is not empty, use --force to overwrite")
            target.mkdir(parents=True, exist_ok=True)
            archive.extractall(target)
        if not (target / CONFIG_NAME).exists():
            (target / CONFIG_NAME).write_text(
                "roots:\n  - kind: src\n    path: files\n", encoding="utf-8"
            )
        lines = [f"Bundle: {manifest.get('note') or '(no note)'}"]
        revision = manifest.get("revision") or {}
        if revision:
            lines.append(
                f"Revision: {revision['commit']} ({revision['branch']}), dirty: {revision['dirty']}"
            )
        counts = manifest.get("finding_counts") or {}
        lines.append(
            "Findings: " + (", ".join(f"{v} {k}" for k, v in sorted(counts.items())) or "none")
        )
        for finding in sorted(atlas.findings, key=Finding.sort_key)[:3]:
            lines.append(f"{finding.severity}: {finding.spoken}")
            lines.extend(f"  edit files/{site(row)}" for row in finding.where)
        named = shlex.quote(str(target / "atlas.json"))
        lines.append(f"rosio drift --atlas {named}")
        selector = shlex.quote(atlas.findings[0].id) if atlas.findings else "SELECTOR"
        lines.append(f"rosio explain {selector} --atlas {named}")
        return "\n".join(lines)
    except (OSError, ValueError, KeyError, UnicodeError, zipfile.BadZipFile, RuntimeError) as exc:
        raise UsageError(f"cannot open bundle: {exc}") from exc
