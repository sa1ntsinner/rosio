"""The agent surface: the atlas answered as tools instead of read as a file.

The documented entry point is `python -m rosio.serve`. `rosio.serve.tools` is the same
tool set as plain functions over a loaded atlas, with no mcp package involved, so anything
that is not an MCP client can use it too.
"""

from __future__ import annotations

from rosio.serve.server import Options, build, run
from rosio.serve.tools import AtlasSource

__all__ = ["AtlasSource", "Options", "build", "run"]
