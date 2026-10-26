"""The corpus runner must never turn an unread entry into a pass."""

import importlib.util
import sys
from pathlib import Path

import pytest

from rosio.stamps.model import Finding, TopicSeries

CORPUS = Path(__file__).resolve().parents[2] / "corpus" / "stamps"
MANIFEST = """entries:
  - name: absent-one
    tier: control
    format: rosbag2
    fetch: manual
"""


@pytest.fixture(scope="module")
def runner():
    if not (CORPUS / "run.py").exists():
        pytest.skip("corpus is not part of this install")
    sys.path.insert(0, str(CORPUS))
    spec = importlib.util.spec_from_file_location("corpus_run", CORPUS / "run.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def manifest(tmp_path):
    path = tmp_path / "manifest.yaml"
    path.write_text(MANIFEST)
    return path


def series(topic="/scan", stamps=(0.0,)):
    return {
        topic: TopicSeries(
            topic=topic, msgtype="x", stamps=list(stamps), receive_times=list(stamps)
        )
    }


def test_entry_without_series_fails(runner, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(runner, "find_data", lambda root: root)
    monkeypatch.setattr(runner, "analyse", lambda path, limit: ({}, [], []))
    assert runner.run_entry({"name": "empty"}, tmp_path, 10) is False
    out = capsys.readouterr().out
    assert "no topics with headers were found, nothing was analyzed" in out
    assert "FAIL" in out


def test_entry_with_series_passes(runner, monkeypatch, tmp_path):
    finding = Finding("monotonic", "/scan", "error", "/scan stamps go backwards", {})
    monkeypatch.setattr(runner, "find_data", lambda root: root)
    monkeypatch.setattr(runner, "analyse", lambda path, limit: (series(), [finding], []))
    entry = {"name": "full", "expect": [{"check": "monotonic"}]}
    assert runner.run_entry(entry, tmp_path, 10) is True


def test_entry_with_empty_series_fails(runner, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(runner, "find_data", lambda root: root)
    monkeypatch.setattr(runner, "analyse", lambda path, limit: (series(stamps=()), [], []))
    entry = {"name": "silent", "forbid": [{"check": "monotonic"}]}
    assert runner.run_entry(entry, tmp_path, 10) is False
    out = capsys.readouterr().out
    assert "no samples were analyzed" in out
    assert "FAIL" in out


def test_limit_zero_is_refused(runner, manifest, tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        runner.main(["--manifest", str(manifest), "--dest", str(tmp_path), "--limit", "0"])
    assert exc.value.code == 2
    assert "positive" in capsys.readouterr().err


def test_limit_negative_is_refused(runner, manifest, tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        runner.main(["--manifest", str(manifest), "--dest", str(tmp_path), "--limit", "-5"])
    assert exc.value.code == 2
    assert "positive" in capsys.readouterr().err


def test_all_skipped_exits_three(runner, manifest, tmp_path, capsys):
    code = runner.main(["--manifest", str(manifest), "--dest", str(tmp_path / "empty")])
    assert code == 3
    out = capsys.readouterr().out
    assert "1 skipped" in out
    assert "nothing was validated" in out


def test_one_pass_exits_zero(runner, manifest, monkeypatch, tmp_path):
    monkeypatch.setattr(runner, "run_entry", lambda entry, root, limit: True)
    assert runner.main(["--manifest", str(manifest), "--dest", str(tmp_path)]) == 0
