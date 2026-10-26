"""The git hooks that keep an atlas current, and the two revision mode of diff.

Every repository here is created under the test's own temporary directory. Nothing in this
file touches a repository it did not create.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from rosio import history, model
from rosio.cli import main
from rosio.errors import EXIT_OK, ContractError, UsageError

PACKAGE = """<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>0.1.0</version>
  <description>a package with one parameter file</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>
</package>
"""

PARAMS = """demo_node:
  ros__parameters:
    rate: 20.0
    frame: base_link
"""


def git(root: Path, *args: str) -> str:
    done = subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    )
    return done.stdout.strip()


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    """A workspace in a git repository, with one package and one parameter file."""
    root = tmp_path / "repo"
    package = root / "src" / "demo_pkg"
    (package / "config").mkdir(parents=True)
    (package / "package.xml").write_text(PACKAGE, encoding="utf-8")
    (package / "config" / "params.yaml").write_text(PARAMS, encoding="utf-8")
    git(root.parent, "init", "-q", str(root))
    git(root, "config", "user.email", "dev@example.com")
    git(root, "config", "user.name", "developer")
    git(root, "add", "-A")
    git(root, "commit", "-qm", "one package")
    return root


# ---------------------------------------------------------------------------
# the script itself


def test_the_hook_embeds_the_interpreter_and_never_looks_it_up_on_path():
    """A hook runs from a graphical client and from CI, where neither the shell profile nor
    the PATH of the installer applies."""
    script = history.hook_script("post-commit", "/opt/venv/bin/python")

    assert "/opt/venv/bin/python" in script
    assert history.MARKER in script
    assert "rosio extract" not in script
    assert script.startswith("#!/bin/sh")


def test_the_hook_exits_zero_whatever_happens():
    for event in history.HOOK_EVENTS:
        script = history.hook_script(event, "/usr/bin/python3")
        assert script.rstrip().endswith("exit 0")
        assert "||" in script


def test_a_file_checkout_does_not_rebuild_but_a_branch_checkout_does():
    checkout = history.hook_script("post-checkout", "/usr/bin/python3")
    commit = history.hook_script("post-commit", "/usr/bin/python3")

    assert '[ "$3" = "1" ] || exit 0' in checkout
    assert '"$3"' not in commit


def test_an_interpreter_path_with_a_quote_in_it_stays_one_word():
    script = history.hook_script("post-commit", "/opt/it's here/python")

    assert "'/opt/it'\\''s here/python'" in script


def test_only_the_events_rosio_installs_are_written():
    with pytest.raises(UsageError):
        history.hook_script("pre-commit", "/usr/bin/python3")


# ---------------------------------------------------------------------------
# install, status, uninstall


def test_install_writes_the_hooks_the_state_and_the_merge_driver(repo: Path):
    report = history.install(repo, measure=False)

    hooks = repo / ".git" / "hooks"
    for event in history.HOOK_EVENTS:
        target = hooks / event
        assert target.is_file(), event
        assert target.stat().st_mode & 0o111, event
    state = json.loads((repo / ".git" / history.STATE_NAME).read_text(encoding="utf-8"))
    assert state["argv"][:2] == ["extract", "src"]
    assert state["python"]
    driver = git(repo, "config", "--local", "--get", f"merge.{history.DRIVER_NAME}.driver")
    assert "rosio hook merge %O %A %B" in driver
    attributes = (repo / ".gitattributes").read_text(encoding="utf-8")
    assert "rosio-out/atlas.json merge=rosio-atlas" in attributes
    assert report.ok


def test_install_is_idempotent(repo: Path):
    first = history.install(repo, measure=False)
    second = history.install(repo, measure=False)

    assert second.data["attribute_added"] is False
    assert (repo / ".gitattributes").read_text(encoding="utf-8").count("merge=rosio-atlas") == 1
    assert first.data["hooks"] == second.data["hooks"]


def test_install_refuses_to_replace_a_hook_it_did_not_write(repo: Path):
    hooks = repo / ".git" / "hooks"
    hooks.mkdir(parents=True, exist_ok=True)
    mine = hooks / "post-commit"
    mine.write_text("#!/bin/sh\necho somebody else was here\n", encoding="utf-8")

    with pytest.raises(UsageError) as caught:
        history.install(repo, measure=False)
    assert "--force" in str(caught.value)
    assert "somebody else" in mine.read_text(encoding="utf-8")

    history.install(repo, measure=False, force=True)
    assert history.MARKER in mine.read_text(encoding="utf-8")


def test_a_dry_run_writes_nothing(repo: Path):
    report = history.install(repo, measure=False, dry_run=True)

    assert not (repo / ".git" / history.STATE_NAME).exists()
    assert not (repo / ".gitattributes").exists()
    assert not (repo / ".git" / "hooks" / "post-commit").exists()
    assert "would install" in report.text()


def test_status_before_and_after_an_install(repo: Path):
    before = history.status(repo)
    assert before.data["installed"] is False
    assert "no rosio hook" in before.text()

    history.install(repo, measure=False)
    after = history.status(repo)

    assert after.data["installed"] is True
    assert set(after.data["hooks"].values()) == {"installed"}
    assert after.data["merge_driver"]["configured"] is True
    assert after.data["merge_driver"]["attribute"] is True
    assert after.ok


def test_the_interpreter_is_embedded_as_given_and_its_symlink_is_not_followed(repo: Path):
    """A virtual environment is a symlink to the interpreter it was built from, and
    following it embeds the base interpreter, which holds neither rosio nor the
    environment the workspace is read in."""
    venv = repo / "venv" / "bin"
    venv.mkdir(parents=True)
    link = venv / "python"
    link.symlink_to(sys.executable)

    report = history.install(repo, python=str(link), measure=False)

    assert report.data["python"] == str(link)
    assert f"ROSIO_PYTHON='{link}'" in (repo / ".git" / "hooks" / "post-commit").read_text()


def test_status_says_so_when_the_interpreter_the_hook_embeds_is_gone(repo: Path):
    history.install(repo, python="/nowhere/python", measure=False)
    report = history.status(repo)

    assert report.data["python"]["exists"] is False
    assert report.ok is False
    assert "MISSING" in report.text()


def test_status_reports_a_foreign_hook_rather_than_claiming_the_repository(repo: Path):
    history.install(repo, measure=False)
    (repo / ".git" / "hooks" / "post-merge").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    report = history.status(repo)

    assert report.data["hooks"]["post-merge"] == "foreign"
    assert report.ok is False


def test_uninstall_removes_what_install_wrote_and_nothing_else(repo: Path):
    history.install(repo, measure=False)
    (repo / ".gitattributes").write_text(
        (repo / ".gitattributes").read_text(encoding="utf-8") + "*.md text\n", encoding="utf-8"
    )

    report = history.uninstall(repo)

    assert sorted(report.data["removed"]) == sorted(history.HOOK_EVENTS)
    assert not (repo / ".git" / history.STATE_NAME).exists()
    kept = (repo / ".gitattributes").read_text(encoding="utf-8")
    assert "*.md text" in kept
    assert "merge=rosio-atlas" not in kept
    assert history.DRIVER_NAME not in git(repo, "config", "--local", "--list")


def test_uninstall_leaves_an_attribute_line_the_repository_already_carried(repo: Path):
    """The line belongs in the repository, so every clone gets it from the history rather
    than from an install. Deleting it on one machine takes the driver away from the team
    through the next commit."""
    line = history.attribute_line(history.DEFAULT_OUT)
    (repo / ".gitattributes").write_text(line + "\n", encoding="utf-8")
    git(repo, "add", ".gitattributes")
    git(repo, "commit", "-qm", "the atlas is merged by the driver")

    report = history.install(repo, measure=False)
    assert report.data["attribute_added"] is False
    history.uninstall(repo)

    assert (repo / ".gitattributes").read_text(encoding="utf-8") == line + "\n"
    assert git(repo, "status", "--porcelain") == ""


def test_uninstall_leaves_a_hook_rosio_did_not_write(repo: Path):
    history.install(repo, measure=False)
    (repo / ".git" / "hooks" / "post-commit").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    report = history.uninstall(repo)

    assert report.data["kept"] == ["post-commit"]
    assert (repo / ".git" / "hooks" / "post-commit").is_file()


# ---------------------------------------------------------------------------
# the rebuild and what it costs


def test_the_rebuild_is_a_library_operation_and_never_the_command_line():
    """The hook calls an operation over a repository, not the entry point that orchestrates
    it. Importing the module and running one rebuild through it must not pull the parser,
    the exit codes or the verb table in behind them."""
    done = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys, rosio.history;assert 'rosio.cli' not in sys.modules, sorted(sys.modules)",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    source = Path(history.__file__).read_text(encoding="utf-8")
    assert "from rosio.cli" not in source
    assert "import rosio.cli" not in source


def test_a_rebuild_leaves_the_working_directory_where_it_found_it(repo: Path, tmp_path: Path):
    """Every path a rebuild reads is resolved against the repository, so the operation is the
    same one wherever the caller stands and two of them could run side by side."""
    history.install(repo, measure=False)
    elsewhere = tmp_path / "somewhere_else"
    elsewhere.mkdir()
    here = Path.cwd()
    try:
        os.chdir(elsewhere)
        record = history.run_rebuild(repo, event="post-commit")
        assert Path.cwd() == elsewhere.resolve()
    finally:
        os.chdir(here)

    assert record["ok"] is True, record["error"]
    assert (repo / "rosio-out" / "atlas.json").is_file()
    assert not (elsewhere / "rosio-out").exists()


def test_the_operation_writes_the_atlas_the_stored_command_names(repo: Path):
    history.install(repo, measure=False)
    state = history.load_state(repo)

    written = history.build_atlas(repo, state.argv)

    assert written == repo / "rosio-out" / "atlas.json"
    assert written.is_file()


def test_the_hook_script_still_fires_from_an_interpreter_that_is_not_on_the_path(repo: Path):
    """The hook embeds an absolute interpreter and runs `python -m rosio hook run`, so an
    empty PATH has to be enough. This is the property the inversion could have broken."""
    history.install(repo, measure=False)
    hook = repo / ".git" / "hooks" / "post-commit"

    done = subprocess.run(
        [str(hook)],
        cwd=str(repo),
        env={"PATH": "", "HOME": str(repo), "TMPDIR": os.environ.get("TMPDIR", "/tmp")},
        capture_output=True,
        text=True,
        check=False,
    )

    assert done.returncode == 0, done.stderr
    assert (repo / "rosio-out" / "atlas.json").is_file()
    assert history.read_log(repo)[-1]["ok"] is True


def test_a_rebuild_writes_an_atlas_and_records_what_it_cost(repo: Path):
    history.install(repo, measure=False)

    record = history.run_rebuild(repo, event="post-commit")

    assert record["ok"] is True
    assert (repo / "rosio-out" / "atlas.json").is_file()
    assert record["volatile"]["seconds"] >= 0
    assert history.read_log(repo)[-1]["event"] == "post-commit"


def test_the_measured_install_states_the_cost_of_a_commit(repo: Path):
    report = history.install(repo, measure=True)

    assert report.data["measured"]["ok"] is True
    assert "a rebuild takes" in report.text()


def test_a_rebuild_that_fails_is_recorded_and_never_raises(repo: Path):
    history.install(repo, workspace="not_here", measure=False)

    record = history.run_rebuild(repo, event="post-commit")

    assert record["ok"] is False
    assert record["error"] or record["exit"] != 0
    assert history.read_log(repo)[-1]["ok"] is False


def test_the_hook_verb_exits_zero_even_when_the_rebuild_failed(repo: Path, capsys):
    history.install(repo, workspace="not_here", measure=False)

    assert main(["hook", "run", "--repo", str(repo)]) == EXIT_OK
    assert "unaffected" in capsys.readouterr().err


def test_the_hook_verb_exits_zero_when_no_hook_is_installed(repo: Path, capsys):
    assert main(["hook", "run", "--repo", str(repo)]) == EXIT_OK
    assert "hook install" in capsys.readouterr().err


def test_the_log_keeps_the_last_records_and_not_the_whole_history(repo: Path):
    history.install(repo, measure=False)
    for index in range(history.LOG_KEEP + 3):
        history.append_log(repo, {"event": f"run{index}", "ok": True, "volatile": {"seconds": 1.0}})

    rows = history.read_log(repo)

    assert len(rows) == history.LOG_KEEP
    assert rows[-1]["event"] == f"run{history.LOG_KEEP + 2}"


def test_status_reports_the_median_and_the_slowest_rebuild(repo: Path):
    history.install(repo, measure=False)
    for seconds in (1.0, 5.0, 3.0):
        history.append_log(
            repo, {"event": "post-commit", "ok": True, "volatile": {"seconds": seconds}}
        )

    timing = history.status(repo).data["rebuilds"]

    assert timing["count"] == 3
    assert timing["volatile"]["median_seconds"] == 3.0
    assert timing["volatile"]["slowest_seconds"] == 5.0


# ---------------------------------------------------------------------------
# the rebuild of the working tree, and what it is allowed to skip


def test_the_second_rebuild_of_one_tree_is_a_read_and_the_same_answer(repo: Path):
    """The documented workflow pays for two rebuilds per change, one for the source commit
    and one for the commit that carries the atlas rebuilt from it. The second reads the same
    bytes out of the same files as the first."""
    history.install(repo, measure=False)
    atlas = repo / "rosio-out" / "atlas.json"

    first = history.run_rebuild(repo, event="post-commit")
    built = atlas.read_text(encoding="utf-8")
    second = history.run_rebuild(repo, event="post-commit")

    assert first["cached"] is False
    assert second["cached"] is True
    assert second["key"] == first["key"] != ""
    assert atlas.read_text(encoding="utf-8") == built


def test_a_cache_read_is_the_same_answer_as_extracting_again(repo: Path):
    history.install(repo, measure=False)
    history.run_rebuild(repo, event="first")
    atlas = repo / "rosio-out" / "atlas.json"
    cached = atlas.read_text(encoding="utf-8")

    fresh = history.run_rebuild(repo, event="again", reuse=False)

    assert fresh["cached"] is False
    assert atlas.read_text(encoding="utf-8") == cached


def test_an_edit_inside_a_root_misses_and_a_file_outside_every_root_does_not(repo: Path):
    """The key has to be about what the extraction reads. The output directory is not inside
    a root, which is what makes the commit that carries the rebuilt atlas a hit."""
    history.install(repo, measure=False)
    argv = history.load_state(repo).argv
    python = history.load_state(repo).python
    base, _facts = history.worktree_key(repo, argv, python)

    (repo / "NOTE.md").write_text("a note beside the sources\n", encoding="utf-8")
    assert history.worktree_key(repo, argv, python)[0] == base

    (repo / "src" / "demo_pkg" / "config" / "params.yaml").write_text(
        PARAMS.replace("20.0", "15.0"), encoding="utf-8"
    )
    assert history.worktree_key(repo, argv, python)[0] != base


def test_the_cache_refuses_an_answer_built_under_a_different_capability_set(
    repo: Path, monkeypatch: pytest.MonkeyPatch
):
    """One key can otherwise serve two different atlases: an optional extra importable on one
    run and shadowed on the next produces two answers, and the interpreter path is the same
    string both times."""
    history.install(repo, measure=False)
    argv = history.load_state(repo).argv
    python = history.load_state(repo).python
    with_extra, _facts = history.worktree_key(repo, argv, python)

    real = history.capability_set
    # flip whichever way the extra actually is, so the key really changes on a box without it
    present = real().get("tree_sitter") != "absent"
    flipped = "absent" if present else "/nowhere/tree_sitter/__init__.py"
    monkeypatch.setattr(
        history, "capability_set", lambda *_a, **_k: {**real(), "tree_sitter": flipped}
    )
    without_extra, _facts = history.worktree_key(repo, argv, python)

    assert without_extra != with_extra
    assert history.read_cache(repo, without_extra) is None


def test_the_capability_set_names_the_file_and_not_only_the_version():
    """Shadowing an installed package with a broken one at the front of the path leaves the
    version where it was and moves the file."""
    row = history.capability_set(("json",))["json"]

    assert row != "absent"
    assert row.endswith(".py") or row.endswith("json")


def test_a_package_that_is_not_there_is_stated_as_absent():
    assert history.capability_set(("no_such_module_anywhere",)) == {
        "no_such_module_anywhere": "absent"
    }


def _wrapper(tmp_path: Path, log: Path) -> Path:
    """An executable that records that it ran and then is the interpreter."""
    root = Path(history.__file__).resolve().parent.parent
    script = tmp_path / "embedded-python"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$*" >> {log}\n'
        f'PYTHONPATH="{root}${{PYTHONPATH:+:$PYTHONPATH}}" exec {sys.executable} "$@"\n',
        encoding="utf-8",
    )
    script.chmod(0o755)
    return script


def test_the_rebuild_runs_the_interpreter_the_hook_embeds(repo: Path, tmp_path: Path):
    """`--measure` timed the installing interpreter, which is the same as the embedded one
    only when `--python` is absent, and that is the case `--python` exists for."""
    log = tmp_path / "calls.log"
    report = history.install(repo, python=str(_wrapper(tmp_path, log)), measure=True)

    assert report.data["measured"]["ok"] is True
    assert "-m rosio extract" in log.read_text(encoding="utf-8")


def test_the_key_belongs_to_the_interpreter_that_builds_and_not_to_this_one(
    repo: Path, tmp_path: Path
):
    """The entry the timed install writes has to be the one the hook reads, and the hook runs
    the embedded interpreter."""
    log = tmp_path / "calls.log"
    embedded = _wrapper(tmp_path, log)
    history.install(repo, python=str(embedded), measure=False)
    argv = history.load_state(repo).argv

    mine, _one = history.worktree_key(repo, argv, sys.executable)
    theirs, _two = history.worktree_key(repo, argv, str(embedded))

    assert theirs != mine


def test_the_measured_install_primes_the_cache_it_measured(repo: Path):
    report = history.install(repo, measure=True)

    assert report.data["measured"]["cached"] is False
    assert report.data["measured_unchanged"]["cached"] is True
    assert "read back from the build cache" in report.text()


def test_a_non_deterministic_rebuild_is_never_cached(repo: Path):
    """A run that writes a clock into the file has no answer worth storing."""
    history.install(repo, measure=False)
    state = history.load_state(repo)
    argv = [*state.argv, "--no-deterministic"]

    assert history.worktree_key(repo, argv, state.python) == ("", {})


def test_an_interpreter_that_cannot_answer_turns_the_cache_off_rather_than_guessing(
    repo: Path, tmp_path: Path
):
    dead = tmp_path / "not-an-interpreter"
    dead.write_text("#!/bin/sh\nexit 3\n", encoding="utf-8")
    dead.chmod(0o755)
    history.install(repo, measure=False)

    assert history.worktree_key(repo, history.load_state(repo).argv, str(dead)) == ("", {})


def test_no_cache_extracts_again(repo: Path):
    history.install(repo, measure=False)
    history.run_rebuild(repo, event="first")

    assert main(["hook", "run", "--repo", str(repo), "--no-cache", "--quiet"]) == EXIT_OK
    assert history.read_log(repo)[-1]["cached"] is False


def test_a_cache_read_is_counted_and_kept_out_of_the_median(repo: Path):
    """Averaging a read and a build states a cost nothing ever paid."""
    history.install(repo, measure=False)
    for seconds in (10.0, 12.0, 14.0):
        history.append_log(
            repo, {"event": "post-commit", "ok": True, "volatile": {"seconds": seconds}}
        )
    history.append_log(
        repo, {"event": "post-commit", "ok": True, "cached": True, "volatile": {"seconds": 0.1}}
    )

    timing = history.status(repo).data["rebuilds"]

    assert timing["count"] == 4
    assert timing["built"] == 3
    assert timing["cached"] == 1
    assert timing["volatile"]["median_seconds"] == 12.0


# ---------------------------------------------------------------------------
# finding the repository


def test_a_directory_outside_a_repository_is_refused(tmp_path: Path):
    with pytest.raises(UsageError) as caught:
        history.find_repo(tmp_path)
    assert "not inside a git repository" in str(caught.value)


def test_the_repository_is_found_from_a_directory_below_it(repo: Path):
    assert history.find_repo(repo / "src" / "demo_pkg") == repo


# ---------------------------------------------------------------------------
# two revisions


def test_a_revision_is_exported_without_touching_the_repository(repo: Path, tmp_path: Path):
    (repo / "src" / "demo_pkg" / "config" / "params.yaml").write_text(
        PARAMS.replace("20.0", "15.0"), encoding="utf-8"
    )
    git(repo, "commit", "-qam", "slower")

    old = history.export_revision(repo, "HEAD~1", tmp_path / "old")
    new = history.export_revision(repo, "HEAD", tmp_path / "new")

    assert "20.0" in (old / "src/demo_pkg/config/params.yaml").read_text(encoding="utf-8")
    assert "15.0" in (new / "src/demo_pkg/config/params.yaml").read_text(encoding="utf-8")
    assert git(repo, "status", "--porcelain") == ""


def test_an_unknown_revision_says_so(repo: Path, tmp_path: Path):
    with pytest.raises(UsageError):
        history.export_revision(repo, "no-such-revision", tmp_path / "out")


def test_a_revision_is_built_against_its_own_install_tree(tmp_path: Path):
    """The ament index of the working tree answers for whatever is checked out now, which is
    not the revision being read."""
    tree = tmp_path / "tree"
    (tree / "install" / "demo_pkg" / "share" / "ament_index").mkdir(parents=True)

    env = history.revision_env(tree)

    assert env["AMENT_PREFIX_PATH"].split(":")[0] == str(tree / "install" / "demo_pkg")


def test_two_revisions_are_built_and_compared(repo: Path):
    (repo / "src" / "demo_pkg" / "config" / "second.yaml").write_text(PARAMS, encoding="utf-8")
    git(repo, "add", "-A")
    git(repo, "commit", "-qm", "a second parameter file")

    answer = history.compare_revisions("HEAD~1", "HEAD", repo=repo, workspace="src")

    assert answer.data["revisions"] == {"before": "HEAD~1", "after": "HEAD", "workspace": "src"}
    assert "revision HEAD~1 against HEAD" in answer.text()
    assert answer.data["identical"] is False


def test_the_revision_mode_and_the_profile_mode_are_two_questions(repo: Path, capsys):
    code = main(["diff", "--rev", "HEAD", "--rev", "HEAD", "--repo", str(repo), "--profile", "one"])

    assert code != EXIT_OK
    assert "at a time" in capsys.readouterr().err


def test_the_revision_mode_needs_two_revisions(repo: Path, capsys):
    assert main(["diff", "--rev", "HEAD", "--repo", str(repo)]) != EXIT_OK
    assert "exactly two" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# the build cache


def test_the_second_build_of_one_revision_is_a_read(repo: Path, tmp_path: Path):
    first_text, first = history.revision_atlas(repo, "HEAD", tmp_path / "one", "src")
    second_text, second = history.revision_atlas(repo, "HEAD", tmp_path / "two", "src")

    assert first["cached"] is False
    assert second["cached"] is True
    assert second_text == first_text
    assert second["volatile"]["seconds"] < first["volatile"]["seconds"]
    assert history.cache_state(repo)["entries"] == 1


def test_a_cached_atlas_carries_no_directory_of_the_build_that_wrote_it(repo: Path, tmp_path: Path):
    """Two builds of one revision export into two directories, and an atlas carrying either
    of them would compare as changed against itself and cache an answer for one directory."""
    first, _one = history.revision_atlas(repo, "HEAD", tmp_path / "one", "src", cache=False)
    second, _two = history.revision_atlas(repo, "HEAD", tmp_path / "two", "src", cache=False)

    assert first == second
    assert str(tmp_path) not in first


def test_an_edit_to_the_tree_is_a_different_key(repo: Path):
    before = history.revision_key(repo, "HEAD", "src")
    (repo / "src" / "demo_pkg" / "config" / "params.yaml").write_text(
        PARAMS.replace("20.0", "15.0"), encoding="utf-8"
    )
    git(repo, "commit", "-qam", "slower")

    assert history.revision_key(repo, "HEAD", "src") != before
    assert history.revision_key(repo, "HEAD~1", "src") == before
    assert history.revision_key(repo, "HEAD", ".") != history.revision_key(repo, "HEAD", "src")


def test_two_revisions_holding_one_tree_are_one_entry(repo: Path):
    """A rebase that only reordered commits produces one tree, and the key is over the tree
    and never over the revision name."""
    git(repo, "commit", "-q", "--allow-empty", "-m", "an empty commit")

    assert history.revision_key(repo, "HEAD", "src") == history.revision_key(repo, "HEAD~1", "src")


def test_an_unknown_revision_has_no_key_and_is_never_cached(repo: Path):
    assert history.revision_key(repo, "no-such-revision", "src") == ""
    assert history.read_cache(repo, "") is None


def test_a_half_written_entry_and_an_entry_of_another_version_are_both_a_miss(repo: Path):
    key = history.revision_key(repo, "HEAD", "src")
    history.write_cache(repo, key, "{}", revision="HEAD", workspace="src")
    assert history.read_cache(repo, key) == "{}"

    meta = history.cache_dir(repo) / f"{key}.json"
    row = json.loads(meta.read_text(encoding="utf-8"))
    meta.write_text(json.dumps({**row, "version": "0.0.0"}), encoding="utf-8")
    assert history.read_cache(repo, key) is None

    meta.unlink()
    assert history.read_cache(repo, key) is None


def test_the_cache_keeps_the_newest_entries_only(repo: Path):
    for index in range(history.CACHE_KEEP + 3):
        history.write_cache(repo, f"{index:032d}", "{}", revision=str(index), workspace="src")

    assert history.cache_state(repo)["entries"] == history.CACHE_KEEP
    assert history.read_cache(repo, f"{0:032d}") is None


def test_a_revision_comparison_states_what_the_cache_did_and_what_it_cannot_do(repo: Path):
    answer = history.compare_revisions("HEAD", "HEAD", repo=repo, workspace="src")

    body = answer.text()
    assert [row["cached"] for row in answer.data["builds"]] == [False, True]
    assert "read from the build cache" in body
    for rule in history.CACHE_RULES:
        assert rule in body


def test_the_cache_can_be_switched_off(repo: Path):
    answer = history.compare_revisions("HEAD", "HEAD", repo=repo, workspace="src", cache=False)

    assert [row["cached"] for row in answer.data["builds"]] == [False, False]
    assert history.cache_state(repo)["entries"] == 0


def test_uninstall_takes_the_build_cache_with_it(repo: Path):
    history.install(repo, measure=False)
    history.write_cache(repo, "a" * 32, "{}", revision="HEAD", workspace="src")

    report = history.uninstall(repo)

    assert report.data["cache_cleared"] == 1
    assert not history.cache_dir(repo).exists()


def test_the_cache_is_bounded_by_size_as_well_as_by_count(repo: Path):
    """The atlas of a large stack can be tens of megabytes, so a count alone bounds nothing a
    reader can predict."""
    for index in range(4):
        history.write_cache(repo, f"{index:032d}", "x" * 400, revision=str(index), workspace="src")

    history.trim_cache(repo, budget=1000)

    assert history.cache_state(repo)["entries"] == 2
    assert history.read_cache(repo, f"{3:032d}") is not None
    assert history.read_cache(repo, f"{0:032d}") is None


# ---------------------------------------------------------------------------
# one encoder


def test_the_three_sidecar_files_are_written_by_the_one_encoder(repo: Path):
    """AGENTS section 9 point 1 says no module reaches the json encoder, and this module
    reached it three times: for the hook state file, for the hook log and for the cache
    metadata. `tests/test_model.py` walks the package for a fourth."""
    history.install(repo, measure=False)
    history.append_log(repo, {"event": "post-commit", "ok": True, "exit": 0})
    history.write_cache(repo, "b" * 32, "{}", revision="HEAD", workspace="src")

    state = history.state_path(repo).read_text(encoding="utf-8")
    assert state == model.dumps(json.loads(state))

    line = history.log_path(repo).read_text(encoding="utf-8").splitlines()[-1]
    assert line == model.dumps_line(json.loads(line))

    meta = (history.cache_dir(repo) / f"{'b' * 32}.json").read_text(encoding="utf-8")
    assert meta == model.dumps(json.loads(meta))


def test_a_log_row_that_cannot_be_encoded_is_refused_and_not_written(repo: Path):
    history.append_log(repo, {"event": "post-commit", "ok": True})
    with pytest.raises(ContractError):
        history.append_log(repo, {"event": "post-merge", "volatile": {"seconds": float("nan")}})

    assert [row["event"] for row in history.read_log(repo)] == ["post-commit"]
