"""`python -m rosio.serve`, the documented way to start the agent surface.

    python -m rosio.serve --atlas rosio-out/atlas.json
    python -m rosio.serve --atlas rosio-out --http --api-key $ROSIO_KEY --host 0.0.0.0

The first form spawns one process per developer over stdio. The second serves a team from
one process over streamable HTTP.
"""

from __future__ import annotations

import argparse
import sys

from rosio import __version__
from rosio.errors import RosioError, exit_code_for
from rosio.serve.server import DEFAULT_PATH, DEFAULT_PORT, Options, run

PROG = "python -m rosio.serve"
DEFAULT_ATLAS = "rosio-out/atlas.json"


def parser() -> argparse.ArgumentParser:
    parsed = argparse.ArgumentParser(
        prog=PROG,
        description=(
            "Serve one rosio atlas to an assistant as MCP tools. stdio by default, one "
            "process per developer. Streamable HTTP with --http, one process for a team. "
            "The bind is loopback only unless --api-key is set."
        ),
    )
    parsed.add_argument(
        "--atlas",
        default=DEFAULT_ATLAS,
        metavar="PATH",
        help=f"atlas.json to answer from, or a directory holding one, default {DEFAULT_ATLAS}",
    )
    parsed.add_argument(
        "--http",
        action="store_true",
        help="serve streamable HTTP instead of stdio, for a shared deployment",
    )
    parsed.add_argument(
        "--host", default="127.0.0.1", help="http bind address, default 127.0.0.1 (loopback)"
    )
    parsed.add_argument(
        "--port", type=int, default=DEFAULT_PORT, help=f"http port, default {DEFAULT_PORT}"
    )
    parsed.add_argument(
        "--api-key",
        default="",
        metavar="KEY",
        help="require this key as Authorization: Bearer <key> or as an x-api-key header. "
        "required to bind anywhere but loopback",
    )
    parsed.add_argument(
        "--path", default=DEFAULT_PATH, help=f"http path to serve on, default {DEFAULT_PATH}"
    )
    parsed.add_argument(
        "--stateless",
        action="store_true",
        help="hold no session between requests, for a load balanced deployment",
    )
    parsed.add_argument("--version", action="version", version=f"rosio {__version__}")
    return parsed


def options(argv: list[str] | None = None) -> Options:
    args = parser().parse_args(argv)
    return Options(
        atlas=args.atlas,
        http=args.http,
        host=args.host,
        port=args.port,
        api_key=args.api_key,
        path=args.path,
        stateless=args.stateless,
    ).check()


def main(argv: list[str] | None = None) -> int:
    try:
        chosen = options(argv)
    except RosioError as exc:
        print(f"{PROG}: {exc}", file=sys.stderr)
        return exit_code_for(exc)
    if chosen.http:
        where = f"http://{chosen.host}:{chosen.port}{chosen.path}"
        guard = "key required" if chosen.api_key else "loopback only, no key"
        print(f"{PROG}: serving {chosen.atlas} on {where} ({guard})", file=sys.stderr)
    try:
        run(chosen)
    except KeyboardInterrupt:
        return 130
    except RosioError as exc:
        print(f"{PROG}: {exc}", file=sys.stderr)
        return exit_code_for(exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
