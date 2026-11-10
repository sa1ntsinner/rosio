"""rosio: a four layer atlas of a ROS 2 system, and the drift between the layers."""

__version__ = "0.2.0"

ATLAS_SCHEMA_VERSION = "1"

LAYERS = ("declared", "coded", "observed", "recorded")

LAYER_WORDS = {"declared": "config", "coded": "code", "observed": "live", "recorded": "bag"}
"""What a person calls each layer, by what it is read from. The atlas, the ids and the
contract keep the names in LAYERS; these are for a screen."""


def extra_line(extra: str) -> str:
    """How to add an optional extra, the same on every machine, because the sentence can
    land in an atlas. rosio is installed from a checkout until it is released."""
    return (
        f"pip install '<checkout>[{extra}]' from a rosio checkout, "
        f"or 'rosio[{extra}]' once released"
    )


def extra_command(extra: str) -> str:
    """The line that adds an optional extra to the environment this process runs in: the
    path rosio was installed from, and uv when uv made the environment, which has no pip."""
    import sys
    from pathlib import Path

    try:
        config = (Path(sys.prefix) / "pyvenv.cfg").read_text(encoding="utf-8")
    except OSError:
        config = ""
    tool = "uv pip" if any(line.startswith("uv ") for line in config.splitlines()) else "pip"
    return f"{tool} install '{origin()}[{extra}]'"


def here(text: str) -> str:
    """Text an atlas stored with the machine independent install line, as this machine runs
    it: the atlas keeps `extra_line`, the terminal shows `extra_command`."""
    for extra in ("cpp", "mcp", "stamps"):
        stored = extra_line(extra)
        if stored in text:
            text = text.replace(stored, extra_command(extra))
    return text


def origin() -> str:
    """Where rosio was installed from: the checkout path, or `rosio` for a release."""
    import json
    from importlib.metadata import PackageNotFoundError, distribution
    from urllib.parse import unquote, urlparse

    try:
        url = json.loads(distribution("rosio").read_text("direct_url.json") or "{}").get("url")
    except (PackageNotFoundError, ValueError, AttributeError):
        url = None
    if isinstance(url, str) and url.startswith("file://"):
        return unquote(urlparse(url).path)
    return "rosio"


__all__ = [
    "ATLAS_SCHEMA_VERSION",
    "LAYERS",
    "__version__",
    "extra_command",
    "extra_line",
    "here",
    "origin",
]
