"""A handover keeps cited lines readable without exposing the sender's machine."""

from __future__ import annotations

import json
import shutil
import subprocess
import zipfile
from collections import Counter
from pathlib import Path

import pytest

from rosio import bundle, cli, extract, ids, model
from rosio.config import Config, RootSpec, load_config
from rosio.errors import UsageError
from rosio.workspace import Workspace

FIXTURE = Path(__file__).parent / "fixtures" / "ws_static"


@pytest.fixture
def problem(tmp_path):
    root = tmp_path / "robot"
    shutil.copytree(FIXTURE, root)
    planted = root / "src/fx_desc/config/credentials.yaml"
    planted.parent.mkdir()
    planted.write_text("api_key: abc123\nnormal: readable\n", encoding="utf-8")
    ws = Workspace.discover([RootSpec("src", str(root / "src"))])
    cfg = Config(roots=ws.roots, profiles=(), out_dir=str(root / "rosio-out"), deterministic=True)
    atlas = extract.run(cfg, ws)
    file_id = ws.file_id(str(planted))
    atlas.findings.append(
        model.Finding(
            rule="param.orphan_block",
            severity="error",
            subject=file_id,
            title="Inspect this configuration",
            say="Edit fx_desc/config/credentials.yaml.",
            where=[model.Source(file=file_id, locator="api_key", line_hint=1)],
        )
    )
    source = root / "rosio-out/atlas.json"
    source.write_bytes(atlas.to_bytes())
    return root, source, atlas


def test_create_is_portable_deterministic_and_redacted(problem, tmp_path):
    root, source, atlas = problem
    first = bundle.create(root, out=tmp_path / "first.rosio.zip", note="Stops on startup")
    second = bundle.create(root, out=tmp_path / "second.rosio.zip", note="Stops on startup")
    assert first.read_bytes() == second.read_bytes()
    with zipfile.ZipFile(first) as archive:
        names = archive.namelist()
        assert names == sorted(names)
        assert {"atlas.json", "manifest.json", "DRIFT.md", "ATLAS_REPORT.md"} <= set(names)
        assert "files/fx_desc/config/credentials.yaml" in names
        assert archive.read("atlas.json") == source.read_bytes()
        assert b"abc123" not in archive.read("files/fx_desc/config/credentials.yaml")
        assert b"<redacted>" in archive.read("files/fx_desc/config/credentials.yaml")
        manifest = json.loads(archive.read("manifest.json"))
        assert manifest["note"] == "Stops on startup"
        assert manifest["finding_counts"]["error"] >= 1
        assert manifest["redactions"]["values"] >= 1
        assert manifest["schema_version"] == atlas.schema_version
        for info in archive.infolist():
            assert info.date_time == (1980, 1, 1, 0, 0, 0)
            content = archive.read(info)
            assert str(tmp_path).encode() not in content
            assert str(tmp_path).replace("\\", "\\\\").encode() not in content


def test_create_cli_honors_atlas_output_and_note(problem, tmp_path, capsys):
    root, source, _atlas = problem
    output = tmp_path / "cli.rosio.zip"
    assert (
        cli.main(
            [
                "bundle",
                "create",
                str(root),
                "--atlas",
                str(source),
                "--out",
                str(output),
                "--note",
                "Inspect this robot",
                "--include",
                "cited",
                "--json",
            ]
        )
        == 0
    )
    assert json.loads(capsys.readouterr().out)["data"]["written"] == str(output)
    with zipfile.ZipFile(output) as archive:
        assert json.loads(archive.read("manifest.json"))["note"] == "Inspect this robot"


def test_the_global_out_names_the_directory_that_holds_the_atlas(problem, tmp_path, capsys):
    """`rosio --out rosio-shim bundle create` failed with Is a directory: the bundle's own
    --out FILE took the value the global --out DIR was given."""
    root, source, _atlas = problem
    elsewhere = root / "rosio-shim"
    elsewhere.mkdir()
    (elsewhere / "atlas.json").write_bytes(source.read_bytes())
    source.unlink()
    assert cli.main(["--out", str(elsewhere), "bundle", "create", str(root), "--json"]) == 0
    written = json.loads(capsys.readouterr().out)["data"]["written"]
    assert written == str(elsewhere / "bundle.rosio.zip")
    target = tmp_path / "named.rosio.zip"
    assert (
        cli.main(["--out", str(elsewhere), "bundle", "create", str(root), "--out", str(target)])
        == 0
    )
    assert target.is_file()


def test_open_and_query_with_readable_citations(problem, tmp_path, capsys):
    root, _source, original = problem
    archive = bundle.create(root)
    into = tmp_path / "received"
    assert cli.main(["bundle", "open", str(archive), "--into", str(into)]) == 0
    summary = capsys.readouterr().out
    assert "edit files/fx_desc/config/credentials.yaml:1" in summary
    assert len(original.findings) > 3
    assert f"and {len(original.findings) - 3} more, every one of them:" in summary
    assert "rosio drift --atlas" in summary
    assert "rosio explain" in summary
    loaded = model.Atlas.loads((into / "atlas.json").read_text())
    config = load_config(None, workspace=str(into))
    workspace = Workspace.discover(config.roots)
    sender = Workspace.discover([RootSpec("src", str(root / "src"))])
    for finding in loaded.findings:
        for row in finding.where:
            path = Path(workspace.abspath(row.file))
            if not Path(sender.abspath(row.file)).is_file():
                continue
            assert path.is_file(), row.file
            if row.line_hint:
                assert len(path.read_text().splitlines()) >= row.line_hint
    assert cli.main(["drift", "--atlas", str(into / "atlas.json")]) == 0
    assert cli.main(["explain", original.findings[-1].id, "--atlas", str(into / "atlas.json")]) == 0
    with pytest.MonkeyPatch.context() as patch:
        patch.chdir(into)
        capsys.readouterr()
        assert cli.main(["drift"]) == 0
        assert f"{len(loaded.findings)} findings" in capsys.readouterr().out
    with pytest.raises(UsageError, match="not empty"):
        bundle.open_bundle(archive, into=into)
    bundle.open_bundle(archive, into=into, force=True)


@pytest.mark.parametrize(
    "name", ["../escaped", "/absolute", "C:/absolute", "a/../../b", r"..\escaped"]
)
def test_zip_slip_is_refused_before_writing(tmp_path, name):
    archive = tmp_path / "bad.rosio.zip"
    with zipfile.ZipFile(archive, "w") as handle:
        handle.writestr("innocent", "first")
        handle.writestr(name, "bad")
    into = tmp_path / "out"
    assert cli.main(["bundle", "open", str(archive), "--into", str(into)]) == 2
    assert not into.exists()


def test_include_cited_and_caps_report_omissions(problem, tmp_path, monkeypatch):
    root, _source, _atlas = problem
    cited = bundle.create(root, include="cited", out=tmp_path / "all.rosio.zip")
    with zipfile.ZipFile(cited) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        assert len(manifest["files"]) > 2
    monkeypatch.setattr(bundle, "MAX_FILES", 1)
    capped = bundle.create(root, include="cited", out=tmp_path / "capped.rosio.zip")
    with zipfile.ZipFile(capped) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        assert manifest["limits"]["hit"]
        assert len(manifest["files"]) == 1
        assert any(row["reason"] == "over_cap" for row in manifest["omitted"])


def test_missing_and_outside_files_are_recorded(problem, tmp_path):
    root, source, atlas = problem
    for path in ("absent.yaml", "escape.yaml"):
        atlas.findings.append(
            model.Finding(
                rule="param.orphan_block",
                severity="warn",
                subject=ids.build_file("fx_desc", path),
                title="Missing input",
                where=[model.Source(file=ids.build_file("fx_desc", path))],
            )
        )
    outside = tmp_path / "private.yaml"
    outside.write_text("hidden: true\n")
    link = root / "src/fx_desc/escape.yaml"
    try:
        link.symlink_to(outside)
    except OSError:
        pytest.skip("symlinks unavailable")
    source.write_bytes(atlas.to_bytes())
    archive = bundle.create(root)
    with zipfile.ZipFile(archive) as handle:
        omitted = json.loads(handle.read("manifest.json"))["omitted"]
        assert {row["reason"] for row in omitted} >= {"missing", "outside_every_root"}


def test_redaction_keeps_lines_and_non_secret_values():
    text = """api_key: abc123
password: |
  first
  second
normal: still_here
"""
    counts = Counter()
    result = bundle.redact(text, counts)
    assert "abc123" not in result and "first" not in result and "second" not in result
    assert len(result.splitlines()) == len(text.splitlines())
    launch = "DeclareLaunchArgument('api_key', default_value='abc123')\n"
    assert "abc123" not in bundle.redact(launch, counts)
    assert "abc123" not in bundle.redact("api_key:=abc123 normal:=readable", counts)
    assert "still_here" in result
    xml = '<arg name="api_key" default="abc123"/>\n'
    assert "abc123" not in bundle.redact(xml, counts)
    nested = "Node(additional_env={'KEY': 'environment_value'})\n"
    assert "environment_value" not in bundle.redact(nested, counts)


def test_zip_symlinks_and_existing_escape_are_refused(problem, tmp_path):
    root, _source, _atlas = problem
    archive = bundle.create(root)
    into = tmp_path / "symlink"
    outside = tmp_path / "outside"
    into.mkdir()
    outside.mkdir()
    try:
        (into / "files").symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("symlinks unavailable")
    with pytest.raises(UsageError, match="unsafe bundle member"):
        bundle.open_bundle(archive, into=into, force=True)
    assert not list(outside.iterdir())
    bad = tmp_path / "link.rosio.zip"
    with zipfile.ZipFile(bad, "w") as handle:
        link = zipfile.ZipInfo("files/link")
        link.create_system = 3
        link.external_attr = 0o120777 << 16
        handle.writestr(link, "../../outside")
    with pytest.raises(UsageError, match="unsafe bundle member"):
        bundle.open_bundle(bad, into=tmp_path / "out")


@pytest.mark.parametrize(
    ("members", "message"),
    [
        (["atlas.json", "atlas.json."], "unsafe bundle member"),
        (["atlas.json", "files/a /b.yaml"], "unsafe bundle member"),
        (["atlas.json", "ATLAS.json"], "duplicate members"),
    ],
)
def test_names_windows_folds_together_are_refused(tmp_path, members, message):
    bad = tmp_path / "alias.rosio.zip"
    with zipfile.ZipFile(bad, "w") as handle:
        handle.writestr("manifest.json", "{}")
        for name in members:
            handle.writestr(name, "{}")
    with pytest.raises(UsageError, match=message):
        bundle.open_bundle(bad, into=tmp_path / "out")


def test_an_archive_that_expands_past_the_budget_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(bundle, "OPEN_MAX_BYTES", 1000)
    bad = tmp_path / "large.rosio.zip"
    with zipfile.ZipFile(bad, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        handle.writestr("manifest.json", "{}")
        handle.writestr("atlas.json", " " * 2000)
    with pytest.raises(UsageError, match="expands past 1000 bytes"):
        bundle.open_bundle(bad, into=tmp_path / "out")
    assert not (tmp_path / "out").exists()


def test_git_revision_and_text_changes(problem, tmp_path):
    root, _source, _atlas = problem

    def git(*args):
        subprocess.run(["git", "-C", str(root), *args], check=True, capture_output=True)

    git("init")
    git("-c", "user.name=Test", "-c", "user.email=test@example.org", "add", ".")
    git("-c", "user.name=Test", "-c", "user.email=test@example.org", "commit", "-m", "Fixture")
    (root / "src/fx_desc/package.xml").write_text("<package><name>fx_desc</name></package>\n")
    (root / "src/fx_desc/config/credentials.yaml").write_text("api_key: changed_secret\n")
    archive = bundle.create(root, out=tmp_path / "revision.rosio.zip")
    with zipfile.ZipFile(archive) as handle:
        manifest = json.loads(handle.read("manifest.json"))
        assert manifest["revision"]["dirty"]
        assert len(manifest["revision"]["commit"]) == 40
        assert b"package.xml" in handle.read("changes.diff")
        assert b"abc123" not in handle.read("changes.diff")
        assert b"changed_secret" not in handle.read("changes.diff")
    first = bundle.create(root).read_bytes()
    assert first == bundle.create(root).read_bytes()


def test_config_and_atlas_environment_are_removed(problem, tmp_path):
    root, source, atlas = problem
    (root / "rosio.yaml").write_text(
        "roots:\n  - kind: src\n    path: src\n"
        "profiles:\n  - slug: demo\n    env:\n      PASSWORD: environment_secret\n"
        "    entrypoints: []\n"
    )
    atlas.generated["env"] = {"NORMAL": "atlas_environment_value"}
    source.write_bytes(atlas.to_bytes())
    archive = bundle.create(root)
    with zipfile.ZipFile(archive) as handle:
        for name in handle.namelist():
            assert b"environment_secret" not in handle.read(name)
            assert b"atlas_environment_value" not in handle.read(name)
        config = handle.read("rosio.yaml")
        assert b"path: files" in config
    into = tmp_path / "with_config"
    bundle.open_bundle(archive, into=into)
    assert load_config(None, workspace=str(into)).roots[0].path == str(into / "files")


def test_redacted_values_cannot_reappear_in_atlas_prose(problem):
    root, source, atlas = problem
    atlas.findings[-1].say = "api_key abc123 needs changing in credentials.yaml."
    atlas.generated["packages"]["token_detector"] = {"src": "token_detector"}
    source.write_bytes(atlas.to_bytes())
    archive = bundle.create(root)
    with zipfile.ZipFile(archive) as handle:
        for name in ("atlas.json", "DRIFT.md", "ATLAS_REPORT.md"):
            assert b"abc123" not in handle.read(name)
        loaded = model.Atlas.loads(handle.read("atlas.json"))
        assert loaded.generated["packages"]["token_detector"] == {"src": "token_detector"}


def test_environment_reads_recorded_as_predicates_lose_their_values():
    counts = Counter()
    secrets: set[str] = set()
    cleaned = bundle._clean(
        {"predicates": {"env:API_KEY=TOPSECRET": True, "env:ROBOT=rover_two": True, "f": 1}},
        counts,
        secrets=secrets,
    )
    assert cleaned == {
        "predicates": {"env:API_KEY=<redacted>": True, "env:ROBOT=<redacted>": True, "f": 1}
    }
    assert secrets == {"TOPSECRET"}
    assert "TOPSECRET" not in json.dumps(bundle._mask(cleaned, secrets, counts))


def test_binary_citations_are_included(problem):
    root, source, atlas = problem
    payload = b"\xff\x00\xfe\x01"
    (root / "src/fx_desc/geometry.dat").write_bytes(payload)
    file_id = ids.build_file("fx_desc", "geometry.dat")
    atlas.findings.append(
        model.Finding(
            rule="param.orphan_block",
            severity="warn",
            subject=file_id,
            title="Geometry input",
            where=[model.Source(file=file_id)],
        )
    )
    source.write_bytes(atlas.to_bytes())
    with zipfile.ZipFile(bundle.create(root)) as archive:
        assert archive.read("files/fx_desc/geometry.dat") == payload


def test_root_relative_citation(problem, tmp_path):
    root, source, atlas = problem
    file_id = ids.build_file(ids.REPO_PKG, "src/notes.yaml")
    (root / "src/notes.yaml").write_text("normal: value\n")
    atlas.findings.append(
        model.Finding(
            rule="param.orphan_block",
            severity="warn",
            subject=file_id,
            title="Root input",
            where=[model.Source(file=file_id, line_hint=1)],
        )
    )
    source.write_bytes(atlas.to_bytes())
    archive = bundle.create(root)
    into = tmp_path / "root_relative"
    bundle.open_bundle(archive, into=into)
    ws = Workspace.discover([RootSpec("src", str(into / "files"))])
    assert Path(ws.abspath(file_id)).read_text() == "normal: value\n"
