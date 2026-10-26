"""The agent surface: the install glue every assistant reads, beside the MCP server.

The two entry points are `rosio.agent.install.install` and
`rosio.agent.install.uninstall`. They are not re exported here, because the names would
shadow the modules of the same names and a shadowed submodule is a confusing import error.

The MCP server itself lives in `rosio.serve`. `serve` is resolved lazily here so that
nothing on the install path can pull in the optional `mcp` package.
"""

from __future__ import annotations

from typing import Any

__all__ = ["serve"]


def __getattr__(name: str) -> Any:
    if name == "serve":
        try:
            from rosio.serve import run
        except ImportError as exc:
            from rosio.errors import CollectorError

            raise CollectorError(
                f"the MCP server is not available: {exc}. "
                "install the extra with: pip install 'rosio[mcp]'",
                collector="serve",
            ) from exc
        return run
    raise AttributeError(name)
