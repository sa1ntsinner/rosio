"""Tests for the agent install surface: the platform table, the writers, the round trip."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

from rosio import extra_command
from rosio.agent import install as ins
from rosio.agent import platforms as plat
from rosio.errors import UsageError


def snapshot(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): path.read_text(encoding="utf-8")
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def statuses(report) -> dict[str, str]:
    return {str(action.path): action.status for action in report.actions}


# the table


def test_the_table_covers_every_platform_the_readme_promises():
    assert plat.keys() == (
        "agent-skills",
        "aider",
        "amp",
        "claude-code",
        "codex",
        "copilot",
        "cursor",
        "gemini-cli",
        "kilo",
        "opencode",
    )


def test_every_target_has_a_known_mechanism_a_path_and_a_writer():
    for platform in plat.platforms():
        assert platform.targets, platform.key
        for target in platform.targets:
            assert target.mechanism in plat.MECHANISMS
            assert target.project or target.user
            assert target.writer in ("whole_file", "section", "json_merge", "toml_section")
            if target.mechanism == "mcp":
                assert target.container
            else:
                assert (ins.TEMPLATES / target.template).is_file()


def test_an_unverified_target_says_why():
    for row in plat.table():
        if not row["verified"]:
            assert row["note"]


def test_only_the_scopes_a_running_tool_confirmed_are_marked_proven():
    """proven means somebody started that tool and watched it read the file at that scope.
    The list is pinned here so a new row cannot inherit the claim by copying a neighbour,
    and a target with a project path and a user path makes two claims, not one."""
    proven = {
        (row["platform"], row["mechanism"]): tuple(row["proven"])
        for row in plat.table()
        if row["proven"]
    }
    assert proven == {
        ("claude-code", "skill"): ("user",),
        ("claude-code", "instructions"): ("user",),
        ("codex", "instructions"): ("user",),
        ("codex", "mcp"): ("user",),
    }


def test_a_target_proven_on_one_scope_only_is_not_marked_verified():
    """The four proven rows were all watched at user scope, and three of them also carry a
    project path nothing has read back. verified covers every scope a target offers, so
    those three stay unverified and say which half holds."""
    rows = {(row["platform"], row["mechanism"]): row for row in plat.table()}
    half = rows[("claude-code", "skill")]
    assert half["project"] and half["user"]
    assert half["proven"] == ["user"]
    assert half["verified"] is False
    assert "user scope" in half["note"]
    whole = rows[("codex", "mcp")]
    assert not whole["project"]
    assert whole["verified"] is True
    assert {row["platform"] for row in plat.table() if row["verified"]} == {"codex"}


def test_the_table_states_which_scope_was_demonstrated():
    """A bool over two paths cannot say that one of them was watched, so every row carries
    the phrase as well, and the phrase and the flag never disagree."""
    rows = {(row["platform"], row["mechanism"]): row for row in plat.table()}
    assert rows[("claude-code", "skill")]["proof"] == (
        "seen read at user scope, project scope from the tool's documentation"
    )
    assert rows[("codex", "mcp")]["proof"] == "seen read at user scope"
    assert rows[("copilot", "instructions")]["proof"] == "from the tool's documentation"
    for row in plat.table():
        demonstrated = row["proof"].startswith("seen read") and "documentation" not in row["proof"]
        assert demonstrated is row["verified"], row
        for scope in plat.SCOPES:
            if scope in row["proven"]:
                assert scope in row["proof"]


def test_an_unverified_platform_says_so_in_the_install_report(tmp_path):
    report = ins.install("gemini-cli", "project", root=tmp_path, home=tmp_path / "home")
    assert any("not yet seen read by gemini-cli" in note for note in report.notes)


def test_every_mechanism_is_used_by_at_least_one_platform():
    used = {row["mechanism"] for row in plat.table()}
    assert used == set(plat.MECHANISMS)


def test_an_unknown_platform_or_scope_raises_a_usage_error_naming_the_choices():
    with pytest.raises(UsageError) as unknown:
        plat.get("emacs")
    assert "claude-code" in str(unknown.value)
    with pytest.raises(UsageError):
        plat.check_scope("global")


# the content


def test_the_rendered_skill_carries_the_rule_the_commands_and_a_worked_example():
    text = ins.render("skill.md")
    assert text.startswith("---\nname: rosio\n")
    for needle in (
        "rosio param <node> <name>",
        "rosio query <text>",
        "rosio explain <id or finding>",
        "rosio drift --min-severity warn",
        "rosio graph --focus <node>",
        "a yaml value is usually not the value that reached the node",
        "$ rosio param /controller_server use_sim_time",
        "state effective",
        "rank 0 wins",
    ):
        assert needle in text, needle


NEEDLES_ENV = "ROSIO_PRIVATE_NEEDLES"

# Always refused, wherever the needle file points: an absolute path off this machine is a
# leak on any workspace and needs no list to name it.
ALWAYS = ("/home/", "/root/", "/Users/")


def private_needles() -> tuple[str, ...]:
    """The names this repository must never carry, read from a file outside it.

    The needles identify the workspace rosio was developed against, so writing them down
    here would be the leak the test exists to catch. Point ROSIO_PRIVATE_NEEDLES at a file
    holding one needle per line, blank lines and # comments ignored.

    With the variable unset the test skips, because a stranger cloning this repository has no
    private workspace and nothing to guard against. With it set, an unreadable or empty file
    fails rather than skipping: a guard that quietly stops guarding is worse than no guard,
    because the tree keeps reporting green.
    """
    configured = os.environ.get(NEEDLES_ENV)
    if not configured:
        pytest.skip(f"{NEEDLES_ENV} is not set, so there is no private workspace to guard against")
    path = Path(configured)
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as error:
        pytest.fail(f"{NEEDLES_ENV} names {path}, which cannot be read: {error}")
    needles = tuple(
        line.strip()
        for line in raw.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )
    if not needles:
        pytest.fail(f"{NEEDLES_ENV} names {path}, which holds no needles")
    return needles + ALWAYS


def test_no_template_names_anything_from_a_private_workspace():
    """These four files are copied into a stranger's repository. A package name, a node
    name or a path from the workspace this tool was written against has no business
    travelling with them, and a worked example has to come from something the reader can
    open: nav2_bringup ships with ROS 2 humble."""
    needles = private_needles()
    for name in ("core.md", "skill.md", "rules.mdc", "instructions.md"):
        text = (ins.TEMPLATES / name).read_text(encoding="utf-8")
        for needle in needles:
            assert needle.lower() not in text.lower(), (name, needle)


def test_no_installed_file_names_anything_from_a_private_workspace(tmp_path):
    """The rendered files, not just the templates, for every platform and both scopes."""
    needles = private_needles()
    for platform in plat.platforms():
        for scope in plat.SCOPES:
            root = tmp_path / platform.key / scope
            root.mkdir(parents=True)
            ins.install(platform.key, scope, root=root, home=root)
            for path in sorted(root.rglob("*")):
                if not path.is_file():
                    continue
                text = path.read_text(encoding="utf-8")
                for needle in needles:
                    assert needle.lower() not in text.lower(), (path, needle)


# ---------------------------------------------------------------------------
# what the templates have to say


def test_every_agent_facing_file_documents_the_waiver_flag():
    """The one thing that stops a team turning the whole gate off was in the code and in the
    contract and in no document a reader or an agent ever opens. An agent that meets
    `--fail-on` and never meets `--waivers` proposes lowering the gate."""
    for name in ("skill.md", "rules.mdc", "instructions.md"):
        text = ins.render(name)
        assert "rosio drift --waivers" in text, name
        assert "rosio-waivers.yaml" in text, name


def test_the_agent_facing_files_state_what_a_waiver_file_holds():
    text = ins.render("skill.md")
    assert "`waivers:` list" in text
    assert "finding id" in text and "sha256:" in text
    assert "mandatory `reason`" in text


def test_the_agent_facing_files_say_a_stale_key_reports_itself():
    """A waiver whose finding was fixed is how a waiver file quietly widens."""
    text = ins.render("skill.md")
    assert "matched nothing" in text
    assert "stale waiver reports itself" in text


def test_the_readme_documents_the_waiver_file_beside_the_verb():
    """The human document and the agent facing one carry the same flag, or the two readers of
    this tool get two different answers about the same gate."""
    readme = (Path(ins.__file__).resolve().parents[2] / "README.md").read_text(encoding="utf-8")
    assert "--waivers FILE" in readme
    assert "rosio drift --waivers rosio-waivers.yaml" in readme
    assert "waiver matched nothing, delete it:" in readme
    assert "reason: " in readme


def test_the_content_stays_under_a_page():
    body = (ins.TEMPLATES / "core.md").read_text(encoding="utf-8")
    assert len(body.splitlines()) < 60
    assert len(body) < 3000


def test_the_content_carries_no_dash_punctuation():
    for name in ("core.md", "skill.md", "rules.mdc", "instructions.md"):
        text = (ins.TEMPLATES / name).read_text(encoding="utf-8")
        for dash in ("\u2014", "\u2013"):
            assert dash not in text
        for line in text.splitlines():
            assert " - " not in line, (name, line)


def test_every_template_renders_with_no_placeholder_left():
    for name in ("skill.md", "rules.mdc", "instructions.md"):
        assert "{{" not in ins.render(name)


# install, project scope


def test_install_writes_the_three_files_claude_code_reads(tmp_path):
    report = ins.install("claude-code", "project", root=tmp_path, home=tmp_path / "home")
    assert statuses(report) == {
        str(tmp_path / ".claude/skills/rosio/SKILL.md"): ins.CREATED,
        str(tmp_path / "CLAUDE.md"): ins.CREATED,
        str(tmp_path / ".mcp.json"): ins.CREATED,
    }
    skill = (tmp_path / ".claude/skills/rosio/SKILL.md").read_text(encoding="utf-8")
    assert skill.endswith(ins.MANAGED_NOTE + "\n")
    assert ins.MD_BEGIN in (tmp_path / "CLAUDE.md").read_text(encoding="utf-8")
    doc = json.loads((tmp_path / ".mcp.json").read_text(encoding="utf-8"))
    assert doc["mcpServers"]["rosio"] == {
        "type": "stdio",
        "command": "rosio",
        "args": ["serve", "--atlas", "rosio-out/atlas.json"],
    }


def test_a_project_entry_holds_nothing_of_this_machine(tmp_path):
    """`.mcp.json` is committed, and it named this machine's venv and checkout, so a
    teammate's clone got an entry that failed to start."""
    ins.install("claude-code", "project", root=tmp_path, home=tmp_path / "home")
    text = (tmp_path / ".mcp.json").read_text(encoding="utf-8")
    entry = json.loads(text)["mcpServers"]["rosio"]
    assert entry["command"] == "rosio"
    assert entry["args"] == ["serve", "--atlas", "rosio-out/atlas.json"]
    assert str(tmp_path) not in text and sys.executable not in text


def test_a_user_entry_starts_this_interpreter_and_names_no_atlas(tmp_path):
    """An assistant started outside the virtual environment had no rosio on its PATH."""
    ins.install("codex", "user", root=tmp_path, home=tmp_path / "home")
    text = (tmp_path / "home/.codex/config.toml").read_text(encoding="utf-8")
    assert f"command = {json.dumps(sys.executable)}" in text
    assert "--atlas" not in text


def test_a_project_entry_says_when_rosio_is_not_on_a_minimal_path(tmp_path, monkeypatch):
    monkeypatch.setattr(ins, "_on_minimal_path", lambda name: False)
    report = ins.install("cursor", "project", root=tmp_path, home=tmp_path / "home")
    assert "--scope user" in report.text()
    assert "uv tool install '" in report.text()
    monkeypatch.setattr(ins, "_on_minimal_path", lambda name: True)
    report = ins.install("cursor", "project", root=tmp_path / "other", home=tmp_path / "h")
    assert "minimal PATH" not in report.text()


def test_a_platform_whose_user_scope_writes_no_server_is_never_offered_it(tmp_path, monkeypatch):
    """claude-code user scope writes no MCP entry, so --scope user left no server at all."""
    monkeypatch.setattr(ins, "_on_minimal_path", lambda name: False)
    report = ins.install("claude-code", "project", root=tmp_path, home=tmp_path / "home")
    assert "--scope user" not in report.text()
    assert "uv tool install '" in report.text()
    assert "Claude Code's own command" in report.text()


def test_install_prints_one_line_for_paths_nobody_watched_being_read(tmp_path):
    report = ins.install("copilot", "project", root=tmp_path, home=tmp_path / "home")
    assert [note for note in report.notes if "seen read" in note] == [
        "2 of these paths not yet seen read by copilot, rosio install --list says which"
    ]


def test_no_server_entry_is_written_when_the_server_could_not_start(tmp_path, monkeypatch):
    from importlib.metadata import PackageNotFoundError

    def absent(name):
        raise PackageNotFoundError(name)

    monkeypatch.setattr("importlib.metadata.version", absent)
    report = ins.install("claude-code", "project", root=tmp_path, home=tmp_path / "home")
    assert statuses(report)[str(tmp_path / ".mcp.json")] == ins.SKIPPED
    assert not (tmp_path / ".mcp.json").exists()
    assert (tmp_path / "CLAUDE.md").is_file()
    assert extra_command("mcp") in report.text()


def test_an_unchanged_install_prints_no_notes(tmp_path):
    first = ins.install("claude-code", "project", root=tmp_path)
    assert "note:" in first.text()
    assert "note:" not in ins.install("claude-code", "project", root=tmp_path).text()


def test_install_is_idempotent(tmp_path):
    ins.install("claude-code", "project", root=tmp_path)
    before = snapshot(tmp_path)
    again = ins.install("claude-code", "project", root=tmp_path)
    assert set(statuses(again).values()) == {ins.UNCHANGED}
    assert snapshot(tmp_path) == before


def test_a_dry_run_writes_nothing_and_still_reports_every_file(tmp_path):
    report = ins.install("claude-code", "project", root=tmp_path, dry_run=True)
    assert len(report.actions) == 3
    assert snapshot(tmp_path) == {}
    assert "would create " in report.text()
    assert "would created" not in report.text()


def test_two_roots_get_byte_identical_files(tmp_path):
    one, two = tmp_path / "one", tmp_path / "two"
    ins.install("cursor", "project", root=one)
    ins.install("cursor", "project", root=two)
    # the server entry names each project's own atlas, and nothing else differs
    assert {k: v.replace(str(one), "@") for k, v in snapshot(one).items()} == {
        k: v.replace(str(two), "@") for k, v in snapshot(two).items()
    }


def test_the_report_names_the_undo_for_every_file_it_wrote(tmp_path):
    report = ins.install("claude-code", "project", root=tmp_path)
    text = report.text()
    for action in report.actions:
        assert str(action.path) in text
        assert action.undo
    assert "undo: delete" in text


# the user's own content


def test_an_instruction_file_keeps_the_content_the_user_wrote(tmp_path):
    claude = tmp_path / "CLAUDE.md"
    claude.write_text("# house rules\n\nrun the tests before pushing.\n", encoding="utf-8")
    ins.install("claude-code", "project", root=tmp_path)
    text = claude.read_text(encoding="utf-8")
    assert text.startswith("# house rules\n\nrun the tests before pushing.\n")
    assert ins.MD_BEGIN in text
    ins.uninstall("claude-code", "project", root=tmp_path)
    assert claude.read_text(encoding="utf-8") == "# house rules\n\nrun the tests before pushing.\n"


def test_a_skill_file_rosio_did_not_write_is_kept_and_reported(tmp_path):
    skill = tmp_path / ".claude/skills/rosio/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("mine\n", encoding="utf-8")
    report = ins.install("claude-code", "project", root=tmp_path)
    assert statuses(report)[str(skill)] == ins.KEPT
    assert skill.read_text(encoding="utf-8") == "mine\n"
    assert "pass force to replace" in report.text()


def test_force_replaces_a_file_rosio_did_not_write_and_says_so(tmp_path):
    skill = tmp_path / ".claude/skills/rosio/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("mine\n", encoding="utf-8")
    report = ins.install("claude-code", "project", root=tmp_path, force=True)
    assert statuses(report)[str(skill)] == ins.UPDATED
    assert "replaces content rosio did not write" in report.text()
    assert ins.MANAGED_MARK in skill.read_text(encoding="utf-8")


def test_an_existing_mcp_entry_is_kept_unless_forced(tmp_path):
    config = tmp_path / ".mcp.json"
    config.write_text('{"mcpServers": {"rosio": {"command": "mine"}}}', encoding="utf-8")
    report = ins.install("claude-code", "project", root=tmp_path)
    assert statuses(report)[str(config)] == ins.KEPT
    assert json.loads(config.read_text(encoding="utf-8"))["mcpServers"]["rosio"] == {
        "command": "mine"
    }


def test_other_servers_in_the_config_survive_install_and_uninstall(tmp_path):
    config = tmp_path / ".mcp.json"
    config.write_text('{"mcpServers": {"other": {"command": "x"}}}', encoding="utf-8")
    ins.install("claude-code", "project", root=tmp_path)
    doc = json.loads(config.read_text(encoding="utf-8"))
    assert set(doc["mcpServers"]) == {"other", "rosio"}
    ins.uninstall("claude-code", "project", root=tmp_path)
    assert json.loads(config.read_text(encoding="utf-8")) == {
        "mcpServers": {"other": {"command": "x"}}
    }


def test_a_broken_json_config_is_reported_and_not_rewritten(tmp_path):
    config = tmp_path / ".mcp.json"
    config.write_text("{not json", encoding="utf-8")
    with pytest.raises(UsageError) as bad:
        ins.install("claude-code", "project", root=tmp_path)
    assert "fix it by hand" in str(bad.value)
    assert config.read_text(encoding="utf-8") == "{not json"


def test_a_section_that_opens_and_never_closes_is_reported(tmp_path):
    (tmp_path / "CLAUDE.md").write_text(ins.MD_BEGIN + "\nhalf\n", encoding="utf-8")
    with pytest.raises(UsageError):
        ins.install("claude-code", "project", root=tmp_path)


# uninstall


@pytest.mark.parametrize("platform", ["claude-code", "cursor", "gemini-cli", "codex", "opencode"])
def test_install_then_uninstall_returns_the_tree_to_its_starting_state(tmp_path, platform):
    root, home = tmp_path / "proj", tmp_path / "home"
    root.mkdir()
    home.mkdir()
    (root / "README.md").write_text("hello\n", encoding="utf-8")
    before = snapshot(tmp_path)
    for scope in plat.SCOPES:
        ins.install(platform, scope, root=root, home=home)
    for scope in plat.SCOPES:
        ins.uninstall(platform, scope, root=root, home=home)
    assert snapshot(tmp_path) == before
    assert sorted(p.name for p in tmp_path.rglob("*") if p.is_dir()) == ["home", "proj"]


def test_uninstall_twice_is_quiet(tmp_path):
    ins.install("claude-code", "project", root=tmp_path)
    ins.uninstall("claude-code", "project", root=tmp_path)
    again = ins.uninstall("claude-code", "project", root=tmp_path)
    assert set(statuses(again).values()) == {ins.ABSENT}


def test_uninstall_leaves_a_skill_file_rosio_did_not_write(tmp_path):
    skill = tmp_path / ".claude/skills/rosio/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("mine\n", encoding="utf-8")
    report = ins.uninstall("claude-code", "project", root=tmp_path)
    assert statuses(report)[str(skill)] == ins.KEPT
    assert skill.is_file()


# scopes and the other mechanisms


def test_user_scope_writes_under_the_home_directory_only(tmp_path):
    root, home = tmp_path / "proj", tmp_path / "home"
    root.mkdir()
    report = ins.install("claude-code", "user", root=root, home=home)
    assert snapshot(root) == {}
    assert sorted(snapshot(home)) == [".claude/CLAUDE.md", ".claude/skills/rosio/SKILL.md"]
    assert "no file at project scope" not in report.text()


def test_a_platform_with_no_file_at_this_scope_says_so_instead_of_writing(tmp_path):
    report = ins.install("amp", "user", root=tmp_path / "p", home=tmp_path / "h")
    assert report.actions == []
    assert "no file at user scope" in report.text()
    assert not (tmp_path / "h").exists()


def test_the_codex_toml_block_is_valid_and_names_the_serve_command(tmp_path):
    home = tmp_path / "home"
    config = home / ".codex/config.toml"
    config.parent.mkdir(parents=True)
    config.write_text('model = "gpt-5"\n', encoding="utf-8")
    ins.install("codex", "user", root=tmp_path, home=home)
    text = config.read_text(encoding="utf-8")
    assert text.startswith('model = "gpt-5"\n')
    assert "[mcp_servers.rosio]" in text
    assert 'args = ["-m", "rosio", "serve"]' in text
    assert f"command = {json.dumps(sys.executable)}" in text
    assert "type" not in text.split("[mcp_servers.rosio]")[1]


def test_the_opencode_entry_uses_the_shape_that_config_expects(tmp_path):
    ins.install("opencode", "project", root=tmp_path)
    doc = json.loads((tmp_path / "opencode.json").read_text(encoding="utf-8"))
    assert doc["mcp"]["rosio"] == {
        "type": "local",
        "command": ["rosio", "serve", "--atlas", "rosio-out/atlas.json"],
        "enabled": True,
    }


def test_the_cursor_rule_carries_frontmatter_and_the_body(tmp_path):
    ins.install("cursor", "project", root=tmp_path)
    text = (tmp_path / ".cursor/rules/rosio.mdc").read_text(encoding="utf-8")
    assert text.startswith("---\ndescription: ")
    assert "rosio param <node> <name>" in text


def test_the_portable_skills_directory_gets_the_same_skill_as_claude_code(tmp_path):
    one, two = tmp_path / "one", tmp_path / "two"
    ins.install("claude-code", "project", root=one)
    ins.install("agent-skills", "project", root=two)
    assert (one / ".claude/skills/rosio/SKILL.md").read_text(encoding="utf-8") == (
        two / ".agents/skills/rosio/SKILL.md"
    ).read_text(encoding="utf-8")


def test_two_platforms_sharing_one_instruction_file_do_not_fight(tmp_path):
    ins.install("codex", "project", root=tmp_path)
    before = snapshot(tmp_path)
    report = ins.install("opencode", "project", root=tmp_path)
    agents = tmp_path / "AGENTS.md"
    assert statuses(report)[str(agents)] == ins.UNCHANGED
    assert agents.read_text(encoding="utf-8") == before["AGENTS.md"]
    assert agents.read_text(encoding="utf-8").count(ins.MD_BEGIN) == 1


# the command line


def test_the_module_entry_point_installs_and_prints_what_it_wrote(tmp_path, capsys):
    code = ins.main(["claude-code", "--root", str(tmp_path), "--home", str(tmp_path / "home")])
    out = capsys.readouterr().out
    assert code == 0
    assert str(tmp_path / "CLAUDE.md") in out
    assert (tmp_path / ".mcp.json").is_file()


def test_the_command_line_lists_the_table(capsys):
    assert ins.main(["--list"]) == 0
    out = capsys.readouterr().out
    assert "claude-code" in out
    assert "from the tool's documentation" in out


def test_the_command_line_reports_an_unknown_platform_with_the_usage_exit_code(tmp_path, capsys):
    code = ins.main(["emacs", "--root", str(tmp_path)])
    assert code == 2
    assert "unknown platform" in capsys.readouterr().err


def test_serve_is_not_imported_by_install():
    import rosio.agent

    assert "serve" in rosio.agent.__all__
    name = "nothing_here"
    with pytest.raises(AttributeError):
        getattr(rosio.agent, name)


def test_the_listed_table_prints_what_was_proven_and_not_one_flag_over_two_paths(capsys):
    """A row demonstrated at user scope and a row nobody ever demonstrated printed alike,
    so the table claimed less than one of them had and more than the other."""
    assert ins.main(["--list"]) == 0
    out = capsys.readouterr().out

    assert "checked: seen read at user scope, project scope from the tool's documentation" in out
    assert "checked: seen read at user scope\n" in out
    assert "checked: from the tool's documentation" in out


def test_every_row_of_the_listed_table_carries_its_own_proof(capsys):
    ins.main(["--list"])
    lines = capsys.readouterr().out.splitlines()
    rows = [line for line in lines if line and not line.startswith((" ", "platform", "Seen read"))]
    proofs = [line.strip() for line in lines if line.strip().startswith("checked:")]

    assert len(rows) == len(plat.table())
    assert proofs == [f"checked: {row['proof']}" for row in plat.table()]


def test_the_listed_table_says_what_demonstrated_means(capsys):
    ins.main(["--list"])
    out = capsys.readouterr().out

    assert "watched it read the file at that path" in out
    assert "installs anyway" in out
