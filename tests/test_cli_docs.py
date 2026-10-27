"""The checked facts the documents state, held equal to the source they come from.

AGENTS section 11a states the rule: a tracked document carries only facts a test here
recomputes from the package or from a file in this repository, and never a measurement.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import pytest

from rosio import cli, config, model, scaffold
from rosio.agent import install as agent_install
from rosio.drift import build, rules
from rosio.serve import tools

ROOT = Path(__file__).resolve().parents[1]

WORD_NUMBERS = {
    "twelve": 12,
    "twenty": 20,
    "forty seven": 47,
}

SERVED_TOOLS = (
    "overview",
    "param_chain",
    "drift",
    "explain",
    "query",
    "path",
    "neighbours",
    "graph",
    "unresolved",
    "stamps",
    "transform",
    "migrate",
)
"""The twelve the server exposes, in the order `serve/server.py` returns them."""


def doc(name: str) -> str:
    path = ROOT / name
    if not path.is_file():
        pytest.skip(f"{name} is not in this checkout")
    return path.read_text(encoding="utf-8")


def flat(text: str) -> str:
    return " ".join(text.split())


def block(text: str, after: str) -> list[str]:
    """The lines of the first fenced block following a heading or a sentence."""
    start = text.index(after)
    fence = text.index("```", start)
    end = text.index("```", fence + 3)
    return [line for line in text[fence + 3 : end].splitlines() if line.strip()]


# ---------------------------------------------------------------------------
# the verb set


def test_the_readme_command_reference_lists_every_verb():
    lines = block(doc("README.md"), "## Command reference")
    published = {line.split()[1] for line in lines if line.startswith("rosio ")}
    assert published == set(cli.VERBS)


def test_the_readme_init_line_names_every_init_flag():
    lines = block(doc("README.md"), "## Command reference")
    (line,) = [line for line in lines if line.startswith("rosio init ")]
    parser = cli.build_parser()
    init = next(
        action.choices["init"]
        for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    flags = {
        option
        for action in init._actions
        for option in action.option_strings
        if option.startswith("--") and option != "--help"
    }
    published = set(re.findall(r"--[a-z][a-z-]*", line.split("#", 1)[0]))
    shared = {option for action in parser._actions for option in action.option_strings}
    assert flags - shared == published


def test_the_contract_verb_bullet_matches_the_cli():
    text = doc("AGENTS.md")
    bullet = text[text.index("**The verb set is committed here") :]
    bullet = bullet[: bullet.index("Twenty.")]
    named = re.findall(r"`([a-z]+)`", bullet)
    assert set(named) == set(cli.VERBS)
    assert WORD_NUMBERS["twenty"] == len(cli.VERBS)


def test_the_cli_quotes_a_sentence_the_contract_holds():
    """The docstring under `VERBS` once cited a sentence AGENTS.md no longer contained."""
    assert flat(cli.CONTRACT_VERB_RULE) in flat(doc("AGENTS.md"))


# ---------------------------------------------------------------------------
# the rule vocabulary


def test_the_contract_rule_block_is_the_committed_vocabulary():
    text = doc("AGENTS.md")
    published = set(" ".join(block(text, "Committed rule ids for M3.")).split())
    assert published == set(rules.COMMITTED)


def test_the_published_rule_counts_match_the_registry():
    assert WORD_NUMBERS["forty seven"] == len(rules.COMMITTED)
    assert "Forty seven rules with a committed vocabulary" in doc("README.md")


# ---------------------------------------------------------------------------
# the agent surface


def test_the_published_tool_names_match_the_server():
    readme = doc("README.md")
    published = re.search(r"Twelve tools: (.+?)\.\s", flat(readme)).group(1)
    named = re.findall(r"`([a-z_]+)`", published)
    assert named == list(SERVED_TOOLS)
    assert WORD_NUMBERS["twelve"] == len(SERVED_TOOLS)
    for name in SERVED_TOOLS:
        assert callable(getattr(tools, name))


def test_the_published_skill_size_matches_the_template():
    readme = doc("README.md")
    published = float(re.search(r"the skill file is ([\d.]+) KB", readme).group(1))
    written = len(agent_install.render("skill.md").encode("utf-8")) / 1024
    assert round(written, 1) == published


def test_the_published_schema_version_matches_the_model():
    text = doc("AGENTS.md")
    published = re.search(r"The current version is \*\*`([\d.]+)`\*\*", text).group(1)
    assert published == model.SCHEMA_VERSION


def _options(parser: argparse.ArgumentParser) -> set[str]:
    found = set()
    for action in parser._actions:
        found |= set(action.option_strings)
        if isinstance(action, argparse._SubParsersAction):
            for child in action.choices.values():
                found |= _options(child)
    return found


def test_every_flag_the_readme_names_exists_on_its_verb():
    parser = cli.build_parser()
    verbs = next(
        action.choices
        for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    shared = _options(parser) - {o for verb in verbs.values() for o in _options(verb)}
    unknown = []
    for match in re.finditer(r"rosio ([a-z]+)\b([^\n`]*)", doc("README.md")):
        verb, rest = match.group(1), match.group(2).split("#", 1)[0]
        if verb not in verbs:
            continue
        named = set(re.findall(r"(?<![\w-])--[a-z][a-z-]*", rest))
        unknown += [f"{verb} {flag}" for flag in named - _options(verbs[verb]) - shared]
    assert unknown == []


# ---------------------------------------------------------------------------
# the language server


def test_the_server_readme_documents_every_custom_method():
    source = "".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "crates" / "rosio-lsp" / "src").rglob("*.rs")
    )
    methods = set(re.findall(r'"(rosio/[A-Za-z]+|rosio-lsp\.[A-Za-z]+)"', source))
    readme = doc("crates/rosio-lsp/README.md")
    assert sorted(m for m in methods if f"`{m}`" not in readme) == []


# ---------------------------------------------------------------------------
# the configuration reference


def _table_keys(text: str, heading: str) -> set[str]:
    (section,) = [part for part in text.split("\n## ") if part.startswith(f"{heading}\n")]
    return set(re.findall(r"^\| `([a-z_]+)` \|", section, flags=re.MULTILINE))


@pytest.mark.parametrize(
    ("heading", "keys"),
    [
        ("Top level", "_TOP_KEYS"),
        ("A root", "_ROOT_KEYS"),
        ("A profile", "_PROFILE_KEYS"),
        ("An entry point", "_ENTRY_KEYS"),
        ("A waiver file", "_WAIVER_KEYS"),
    ],
)
def test_the_configuration_page_lists_exactly_the_keys_the_loader_accepts(heading, keys):
    assert _table_keys(doc("docs/configuration.md"), heading) == set(getattr(config, keys))


def test_the_configuration_page_states_the_defaults_the_code_uses():
    text = flat(doc("docs/configuration.md"))
    assert f"`{int(config.DEFAULT_LAUNCH_TIMEOUT)}`" in text
    assert f"`{config.DEFAULT_OUT_DIR}`" in text
    assert f"`{build.DEFAULT_RESOLUTION_BUDGET}` overall" in text
    for pattern in config.INSTALL_PREFIX_IGNORE:
        assert f"`{pattern}`" in text
    assert "docs/configuration.md" in scaffold.HEADER
    assert "docs/configuration.md" in doc("README.md")


# ---------------------------------------------------------------------------
# the schema version, wherever a tracked document states it

SCHEMA_STATED = re.compile(r"\b(?:atlas )?schema,? (?:version )?\**`?(\d+\.\d+)\b")
"""`schema 1.18` as overview prints it, `atlas schema 1.18` as version prints it."""


def _stating_documents() -> list[Path]:
    named = [ROOT / name for name in ("README.md", "AGENTS.md", "ARCHITECTURE.md", "CONTEXT.md")]
    named += sorted((ROOT / "docs").rglob("*.md"))
    named += sorted((ROOT / "site" / "output").glob("*.txt"))
    named += [ROOT / "site" / "index.html", ROOT / "site" / "copy.md"]
    named += sorted((ROOT / "rosio" / "agent" / "templates").glob("*"))
    return [path for path in named if path.is_file()]


def test_every_document_that_states_the_schema_version_states_the_current_one():
    """AGENTS section 9a said 1.16 while the tool wrote 1.18, and the site showed 1.13."""
    stale = [
        f"{path.relative_to(ROOT)}: {found}"
        for path in _stating_documents()
        for found in SCHEMA_STATED.findall(path.read_text(encoding="utf-8"))
        if found != model.SCHEMA_VERSION
    ]
    assert stale == []
