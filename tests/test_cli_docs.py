"""The checked facts the documents state, held equal to the source they come from.

AGENTS section 11a states the rule: a tracked document carries only facts a test here
recomputes from the package or from a file in this repository, and never a measurement.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import pytest

from rosio import cli, model
from rosio.agent import install as agent_install
from rosio.drift import rules
from rosio.serve import tools

ROOT = Path(__file__).resolve().parents[1]

WORD_NUMBERS = {
    "eleven": 11,
    "twenty": 20,
    "forty six": 46,
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
    "migrate",
)
"""The eleven the server exposes, in the order `serve/server.py` returns them."""


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
    assert WORD_NUMBERS["forty six"] == len(rules.COMMITTED)
    assert "Forty six rules with a committed vocabulary" in doc("README.md")


# ---------------------------------------------------------------------------
# the agent surface


def test_the_published_tool_names_match_the_server():
    readme = doc("README.md")
    published = re.search(r"Eleven tools: (.+?)\.\s", flat(readme)).group(1)
    named = re.findall(r"`([a-z_]+)`", published)
    assert named == list(SERVED_TOOLS)
    assert WORD_NUMBERS["eleven"] == len(SERVED_TOOLS)
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
