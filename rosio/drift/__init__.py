"""The drift engine: pure functions from a joined atlas to findings.

A rule never re extracts, never reads a file and never mutates an entity. `rules` owns the
committed vocabulary, the alias table, the severity per rule and the registry, and one
module per family owns the implementations.
"""

from __future__ import annotations

from rosio.drift.rules import (
    COMMITTED,
    findings_for,
    registered,
    severity_of,
)

__all__ = ["COMMITTED", "findings_for", "registered", "severity_of"]
