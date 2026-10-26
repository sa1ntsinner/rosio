"""Collect what each ROS 2 distribution ships, for `rosio migrate`.

Two sources, both primary:

* rosdistro's `distribution.yaml` per distribution, which lists every released package.
  A dependency that is released for the old distribution and not for the new one is the
  first thing that stops an upgrade (rosdep cannot resolve it).
* the official `ros:<distro>-ros-base` images, which hold the installed headers and the
  toolchain. A header that is installed by a package in the old distribution and missing
  from the same package in the new one is a build failure for every file that includes it.

The result is written to `rosio/migrate/data/distros.json`. Only what differs between the
distributions is kept in detail: a header present everywhere says nothing about an upgrade,
and a package released everywhere is listed by name only.

Run from the repository root on a machine with docker:

    python corpus/distros/collect.py

The images are pulled when missing. The output is sorted and byte stable for the same inputs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import urllib.request
from pathlib import Path

import yaml

DISTROS = ("humble", "iron", "jazzy", "kilted", "lyrical")
ROSDISTRO = "https://raw.githubusercontent.com/ros/rosdistro/master/{distro}/distribution.yaml"
DATA = Path(__file__).resolve().parents[2] / "rosio" / "migrate" / "data"
OUT = DATA / "distros.json"
RULES = DATA / "rules.yaml"

EXTRA = (
    "cv-bridge",
    "image-geometry",
    "image-transport",
    "camera-info-manager",
    "pcl-conversions",
    "laser-geometry",
    "diagnostic-updater",
    "ros2-control",
    "ros2-controllers",
    "realtime-tools",
    "control-toolbox",
    "navigation2",
    "behaviortree-cpp",
    "tf2-sensor-msgs",
    "tf2-eigen",
    "angles",
    "behaviortree-cpp-v3",
)
"""Packages that ros-base leaves out and that most robots build against, installed on top of
it before measuring. One a release does not ship is skipped there, which the rosdistro half
already reports."""

SYSTEM = ("libceres-dev", "libopencv-dev", "libpcl-dev")

INSTALL = r"""
apt-get update -qq
for pkg in $EXTRA; do
  apt-get install -y -qq --no-upgrade --no-install-recommends ros-$ROS_DISTRO-$pkg >/dev/null 2>&1 \
    || echo "skipped ros-$ROS_DISTRO-$pkg" >&2
done
apt-get install -y -qq --no-upgrade --no-install-recommends $SYSTEM
"""

LISTING = r"""
set -e
. /opt/ros/$ROS_DISTRO/setup.sh
echo "ubuntu $(. /etc/os-release && echo $VERSION_ID)"
echo "python $(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
echo "cmake $(cmake --version | head -1 | awk '{print $3}')"
for pkg in $(ls /opt/ros/$ROS_DISTRO/share); do echo "package $pkg"; done
cd /opt/ros/$ROS_DISTRO/include
find . -type f \( -name '*.h' -o -name '*.hpp' \) | sed 's|^\./|header |'
grep -rliE '^\s*#\s*(warning|pragma\s+message).*deprecat' . | sed 's|^\./|deprecated |' || true
for pkg in libceres-dev libopencv-dev libpcl-dev; do
  dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q 'install ok installed' || continue
  echo "package $pkg"
  if [ "$pkg" = libopencv-dev ]; then
    dpkg-query -W -f='${binary:Package}\n' 'libopencv-*-dev' libopencv-dev | xargs -r dpkg -L
  else
    dpkg -L "$pkg"
  fi | while read -r path; do
    case "$path" in
      /usr/include/ceres/*.h|/usr/include/opencv4/*.h|/usr/include/opencv4/*.hpp|/usr/include/pcl-*/*.h|/usr/include/pcl-*/*.hpp)
        test -f "$path" || continue
        include=${path#/usr/include/}
        case "$include" in opencv4/*|pcl-*/*) include=${include#*/};; esac
        echo "system_header $pkg $include"
        ;;
    esac
  done
done
"""


def measuring_image(distro: str) -> str:
    """ros-base with the measured packages, refreshed when the package set changes."""
    image = f"rosio-facts:{distro}"
    signature = hashlib.sha256((INSTALL + " ".join((*EXTRA, *SYSTEM))).encode()).hexdigest()
    present = subprocess.run(["docker", "image", "inspect", image], capture_output=True)
    if present.returncode == 0:
        labels = json.loads(present.stdout)[0]["Config"].get("Labels") or {}
        if labels.get("rosio.facts.packages") == signature:
            return image
    name = f"rosio-facts-{distro}"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    subprocess.run(
        [
            "docker",
            "run",
            "--name",
            name,
            "-e",
            f"EXTRA={' '.join(EXTRA)}",
            "-e",
            f"SYSTEM={' '.join(SYSTEM)}",
            image if present.returncode == 0 else f"ros:{distro}-ros-base",
            "bash",
            "-c",
            INSTALL,
        ],
        check=True,
    )
    subprocess.run(
        ["docker", "commit", "--change", f'LABEL rosio.facts.packages="{signature}"', name, image],
        check=True,
        capture_output=True,
    )
    subprocess.run(["docker", "rm", name], check=True, capture_output=True)
    return image


def released(distro: str) -> set[str]:
    with urllib.request.urlopen(ROSDISTRO.format(distro=distro), timeout=60) as response:
        data = yaml.safe_load(response)
    names: set[str] = set()
    for repo, entry in data.get("repositories", {}).items():
        release = entry.get("release")
        if release:
            names.update(release.get("packages") or [repo])
    return names


def image_listing(distro: str) -> dict:
    run = subprocess.run(
        ["docker", "run", "--rm", measuring_image(distro), "bash", "-c", LISTING],
        check=True,
        capture_output=True,
        text=True,
    )
    toolchain: dict[str, str] = {}
    packages: set[str] = set()
    installed: list[list[str]] = []
    system_headers: dict[str, str] = {}
    warns: set[str] = set()
    for line in run.stdout.splitlines():
        kind, _, value = line.partition(" ")
        if kind == "package":
            packages.add(value)
        elif kind == "header":
            installed.append(value.split("/"))
        elif kind == "deprecated":
            warns.add(value)
        elif kind == "system_header":
            owner, include = value.split(" ", 1)
            system_headers[include] = owner
        else:
            toolchain[kind] = value
    # ROS packages install into include/<pkg>/ and the code includes what is below it, so
    # include/rcpputils/rcppmath/clamp.hpp is written <rcppmath/clamp.hpp>. A package that
    # installs at least one include/<pkg>/<pkg>/ header uses that layout; anything else
    # (Fast DDS, vendored libraries) puts its own tree straight under include/ and is left
    # to hand written rules.
    layout = {parts[0] for parts in installed if len(parts) >= 3 and parts[0] == parts[1]}
    headers = {
        "/".join(parts[1:]): parts[0]
        for parts in installed
        if len(parts) >= 3 and parts[0] in layout and parts[0] in packages
    }
    headers.update(
        {
            "/".join(parts): parts[0]
            for parts in installed
            if parts[0] in {"behaviortree_cpp_v3", "behaviortree_cpp"} - layout
        }
    )
    headers.update(system_headers)
    deprecated = {
        "/".join(value.split("/")[1:]) for value in warns if value.split("/")[0] in layout
    }
    return {
        "toolchain": toolchain,
        "packages": packages,
        "headers": headers,
        "deprecated": deprecated & set(headers),
    }


def sibling(include: str) -> str:
    stem, dot, ext = include.rpartition(".")
    return f"{stem}.{'hpp' if ext == 'h' else 'h'}" if dot else include


def successor_headers() -> set[str]:
    rules = yaml.safe_load(RULES.read_text(encoding="utf-8"))
    return {entry["use"] for entry in rules.get("headers", {}).values() if entry.get("use")}


def collect() -> dict:
    releases = {distro: released(distro) for distro in DISTROS}
    images = {distro: image_listing(distro) for distro in DISTROS}
    prefixes = yaml.safe_load(RULES.read_text(encoding="utf-8")).get("header_prefixes", {})

    every = set.union(*releases.values())
    packages = {
        name: [d for d in DISTROS if name in releases[d]]
        for name in sorted(every)
        if not all(name in releases[d] for d in DISTROS)
    }

    # A header counts as removed only when its package is installed in that image; a package
    # outside ros-base says nothing about its headers in a distribution.
    every_header = set().union(*(images[d]["headers"] for d in DISTROS))

    def measured(include: str) -> dict:
        owner = next(
            images[d]["headers"][include] for d in DISTROS if include in images[d]["headers"]
        )
        owners = {owner}
        for prefix, successor in prefixes.items():
            if include.startswith(prefix):
                owners.add(successor["use"].rstrip("/"))
        known = [
            d
            for d in DISTROS
            if owner in images[d]["packages"]
            or (owner not in releases[d] and owners & images[d]["packages"])
        ]
        present = [d for d in known if include in images[d]["headers"]]
        entry = {"package": owner, "present": present, "checked": known}
        warned = [d for d in present if include in images[d]["deprecated"]]
        if warned:
            entry["warns"] = warned
        return entry

    headers: dict[str, dict] = {}
    for include in sorted(every_header):
        entry = measured(include)
        changes = entry["present"] != entry["checked"] or entry.get("warns")
        if len(entry["checked"]) >= 2 and changes:
            headers[include] = entry
    # The replacements a rule points at are recorded too, present everywhere or not, so a fix
    # is only offered where the header it writes is measured to exist: the other spelling of a
    # changed header, and every successor named in the rule table.
    wanted = {sibling(include) for include in headers} | successor_headers()
    wanted.update(
        successor["use"] + include[len(prefix) :]
        for include in headers
        for prefix, successor in prefixes.items()
        if include.startswith(prefix)
    )
    for include in sorted(wanted & every_header):
        headers.setdefault(include, measured(include))

    return {
        "source": {
            "rosdistro": ROSDISTRO.format(distro="<distro>"),
            "images": "ros:<distro>-ros-base",
            "installed": list(EXTRA),
            "system": list(SYSTEM),
        },
        "distros": list(DISTROS),
        "toolchain": {d: images[d]["toolchain"] for d in DISTROS},
        "packages": packages,
        # Every package released in all of them, so a rule can tell a ROS package (which
        # exports <name>_TARGETS) from a system library found by the same find_package.
        "everywhere": sorted(name for name in every if name not in packages),
        "headers": headers,
    }


def dump(data: dict) -> str:
    """JSON with one entry per line under each top level key, so a regenerated file diffs
    by package and by header instead of by bracket."""
    lines = ["{"]
    keys = sorted(data)
    for index, key in enumerate(keys):
        value = data[key]
        tail = "," if index < len(keys) - 1 else ""
        if isinstance(value, dict) and value:
            lines.append(f"  {json.dumps(key)}: {{")
            items = sorted(value.items())
            for inner, (name, entry) in enumerate(items):
                comma = "," if inner < len(items) - 1 else ""
                lines.append(f"    {json.dumps(name)}: {json.dumps(entry, sort_keys=True)}{comma}")
            lines.append(f"  }}{tail}")
        elif isinstance(value, list) and len(value) > len(DISTROS):
            lines.append(f"  {json.dumps(key)}: [")
            lines.extend(
                f"    {json.dumps(item)}{',' if inner < len(value) - 1 else ''}"
                for inner, item in enumerate(value)
            )
            lines.append(f"  ]{tail}")
        else:
            lines.append(f"  {json.dumps(key)}: {json.dumps(value, sort_keys=True)}{tail}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    data = collect()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(dump(data), encoding="utf-8")
    print(
        f"{args.out}: {len(data['packages'])} packages, {len(data['headers'])} headers",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
