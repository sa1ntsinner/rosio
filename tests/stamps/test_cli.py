import json

import pytest

from rosio import __version__
from rosio.stamps.cli import main


def test_version(capsys):
    with pytest.raises(SystemExit) as exit_info:
        main(["--version"])
    assert exit_info.value.code == 0
    assert capsys.readouterr().out.strip() == __version__


def test_without_bag(capsys):
    assert main([]) == 2
    assert "usage" in capsys.readouterr().out.lower()


def test_missing_bag(capsys, tmp_path):
    assert main([str(tmp_path / "absent")]) == 2
    err = capsys.readouterr().err
    assert f"rosio stamps: no recording at {tmp_path / 'absent'}" in err
    assert "PosixPath" not in err and "WindowsPath" not in err


def test_malformed_bag(capsys, tmp_path):
    bag = tmp_path / "malformed"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("garbage: 1\n")
    assert main([str(bag)]) == 2
    assert "rosio stamps:" in capsys.readouterr().err


def test_truncated_bag(capsys, clean_bag):
    db = next(clean_bag.glob("*.db3"))
    with db.open("r+b") as handle:
        handle.truncate(db.stat().st_size // 3)
    assert main([str(clean_bag)]) == 2
    err = capsys.readouterr().err
    assert err.startswith("rosio stamps:")
    assert err.count("\n") == 1
    assert "Traceback" not in err


def test_bad_limit(capsys, tmp_path):
    assert main([str(tmp_path / "absent"), "--limit", "0"]) == 2
    assert "--limit" in capsys.readouterr().err


def test_clean_bag(capsys, clean_bag):
    assert main([str(clean_bag)]) == 0
    assert capsys.readouterr().out == "\nverdict:\n  No timestamp problems found across 1 topic.\n"


def test_bag_without_headers_is_not_clean(capsys, headless_bag):
    assert main([str(headless_bag)]) == 2
    out = capsys.readouterr().out
    assert "no topics with headers were found, nothing was analyzed" in out
    assert "No timestamp problems" not in out


def test_bag_without_headers_json_verdict(capsys, headless_bag):
    assert main([str(headless_bag), "--json-verdict"]) == 2
    payload = json.loads(capsys.readouterr().out)
    assert payload["findings"] == []
    assert payload["verdict"]["level"] == "unknown"


def test_backward_bag(capsys, backward_bag):
    assert main([str(backward_bag)]) == 1
    assert "ERROR monotonic /scan:" in capsys.readouterr().out


def test_json_output(capsys, backward_bag):
    assert main([str(backward_bag), "--json"]) == 1
    findings = json.loads(capsys.readouterr().out)
    assert [f["check"] for f in findings] == ["monotonic"]
    assert findings[0]["evidence"]["largest_backward_jump"] == pytest.approx(0.5)


def test_stats_output(capsys, clean_bag):
    assert main([str(clean_bag), "--stats", "--topics", "/scan"]) == 0
    line = capsys.readouterr().out.strip()
    assert line.startswith("/scan msgs=4 ")
    assert "rate=10.00Hz" in line
    assert "offset=0.050000s" in line
