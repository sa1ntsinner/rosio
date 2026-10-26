"""The hand written rules in `data/rules.yaml`, loaded and checked once.

A rule that names an unknown release, an unknown matcher or no source is a defect in the
table, and it fails loudly at load time instead of matching nothing at run time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cache
from pathlib import Path
from typing import Any

import yaml

from rosio.errors import ContractError
from rosio.migrate import distros

DATA = Path(__file__).with_name("data") / "rules.yaml"

MATCHERS = frozenset(
    {
        "cmake_call",
        "cpp_identifier",
        "cpp_regex",
        "launch_node",
        "python_import",
        "params_yaml",
        "bt_xml",
    }
)


@dataclass(frozen=True)
class Rule:
    id: str
    title: str
    say: str
    source: str
    find: dict[str, Any]
    deprecated: str | None = None
    removed: str | None = None
    fix: str | dict[str, str] | None = None
    works_from: str | None = None

    @property
    def matcher(self) -> str:
        return next(key for key in self.find if key in MATCHERS)

    def effect(self, start: str, target: str) -> tuple[str, str] | None:
        """What a move from `start` to `target` does to code this rule matches: the effect
        and the release where it lands. None when the move crosses neither release.

        Only a crossed release counts. A deprecation the code already lived with before the
        move is not news of the move, and it would bury the changes that are.
        """
        if self.removed and distros.crossed(self.removed, start, target):
            return "removed", self.removed
        if self.deprecated and distros.crossed(self.deprecated, start, target):
            return "deprecated", self.deprecated
        return None


@dataclass(frozen=True)
class Successor:
    say: str
    source: str
    use: str | None = None


@dataclass(frozen=True)
class Catalog:
    rules: tuple[Rule, ...]
    headers: dict[str, Successor] = field(default_factory=dict)
    packages: dict[str, Successor] = field(default_factory=dict)
    removed_modules: dict[tuple[int, ...], frozenset[str]] = field(default_factory=dict)
    header_prefixes: dict[str, Successor] = field(default_factory=dict)

    def rule(self, rule_id: str) -> Rule:
        for rule in self.rules:
            if rule.id == rule_id:
                return rule
        raise ContractError(f"no migration rule {rule_id}")


@cache
def catalog() -> Catalog:
    data = yaml.safe_load(DATA.read_text(encoding="utf-8"))
    rules = tuple(_rule(entry) for entry in data["rules"])
    ids = [rule.id for rule in rules]
    if len(ids) != len(set(ids)):
        raise ContractError("migration rule ids repeat")
    headers = {name: _successor(name, entry) for name, entry in data["headers"].items()}
    packages = {name: _successor(name, entry) for name, entry in data["packages"].items()}
    modules = {
        tuple(int(part) for part in version.split(".")): frozenset(names)
        for version, names in data["removed_modules"].items()
    }
    prefixes = {
        name: _successor(name, entry) for name, entry in data.get("header_prefixes", {}).items()
    }
    for name, entry in prefixes.items():
        if not name.endswith("/") or not entry.use or not entry.use.endswith("/"):
            raise ContractError(f"migration header prefix {name} needs directory prefixes")
    return Catalog(rules, headers, packages, modules, prefixes)


def _rule(entry: dict[str, Any]) -> Rule:
    rule_id = entry.get("id", "?")
    for key in ("id", "title", "say", "source", "find"):
        if not entry.get(key):
            raise ContractError(f"migration rule {rule_id} has no {key}")
    unknown = set(entry["find"]) & MATCHERS
    if len(unknown) != 1:
        raise ContractError(f"migration rule {rule_id} names no known matcher")
    for key in ("deprecated", "removed", "works_from"):
        if entry.get(key):
            distros.index(entry[key])
    return Rule(
        id=entry["id"],
        title=entry["title"],
        say=" ".join(entry["say"].split()),
        source=entry["source"],
        find=dict(entry["find"]),
        deprecated=entry.get("deprecated"),
        removed=entry.get("removed"),
        fix=entry.get("fix"),
        works_from=entry.get("works_from"),
    )


def _successor(name: str, entry: dict[str, Any]) -> Successor:
    if not entry.get("source") or not (entry.get("say") or entry.get("use")):
        raise ContractError(f"migration entry {name} needs a source and a use or say")
    say = entry.get("say") or f"use {entry['use']}"
    return Successor(" ".join(say.split()), entry["source"], entry.get("use"))
