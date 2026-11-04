# Commands

The whole command line, how to install from a checkout, what to do when a run goes wrong, and how to waive a finding. The [README](../README.md) has the short version.

## Installing from a checkout

To run the development version, install from a checkout of this repository. On a stock Ubuntu
24.04 with ROS 2 jazzy:

```bash
sudo apt install git python3-venv    # Ubuntu ships python3 without the venv module
git clone https://github.com/sa1ntsinner/rosio ~/rosio
python3 -m venv --system-site-packages ~/.venvs/rosio && source ~/.venvs/rosio/bin/activate
pip install "$HOME/rosio[cpp,stamps,mcp]"
source /opt/ros/jazzy/setup.bash

cd ~/ros2_ws
rosio init                     # rosio.yaml with one profile per launch file, to edit
rosio extract                  # writes rosio-out/atlas.json here
rosio drift                    # what is wrong, worst first, each with the line to edit
rosio render                   # ATLAS_REPORT.md, DRIFT.md and atlas.html
rosio install claude-code      # register rosio with your assistant
```

With uv instead of the apt package, `uv venv --system-site-packages ~/.venvs/rosio` makes the
environment and `uv pip install "$HOME/rosio[cpp,stamps,mcp]"` installs into it, because a uv
environment has no `pip`. Once a release is on PyPI, `pip install "rosio[cpp,stamps,mcp]"`
replaces the clone.

Every verb reads and writes `rosio-out` in the directory it runs in, and `rosio install` writes
into it too, so run them from the workspace.

Walking a launch tree runs `python3` from your PATH, which is the environment's own interpreter
while it is active. ROS installs part of the `launch` dependency chain as operating system
packages (`lark`, for one) that a plain virtual environment hides, so an active environment
without `--system-site-packages` cannot walk anything, and the run summary says so first, above
the counts.
`mcp` is the extra that `rosio serve` and the server entry `rosio install` writes need.

`init` also takes an install tree, such as `/opt/ros/jazzy` or a built workspace's `install/`.
An installed distribution holds a great many launch files, so name the packages you want. Nobody
writes into `/opt/ros`, so the file goes into the directory you run from:

```bash
mkdir nav2 && cd nav2
rosio init /opt/ros/jazzy --package nav2_bringup --config rosio.yaml
rosio extract /opt/ros/jazzy --config rosio.yaml
```

A build kept apart from the sources is named with
`rosio extract ~/ros2_ws --install-tree ~/builds/ros2_ws/install`.

`extract` writes the model, `render` writes the three things you read:

```
rosio-out/
├── atlas.json        the whole model, queried by the CLI and the editor, never read whole by a person
├── ATLAS_REPORT.md   what this system is, per profile, with the coverage stated honestly
├── DRIFT.md          what is wrong, ordered by severity, each with the line to edit
└── atlas.html        the graph in one self contained file, opens with no network
```

Developed and tested on ROS 2 jazzy with Fast DDS and `rmw_zenoh_cpp`; CI runs jazzy and humble. Python 3.10 and newer.

## Command reference

```
rosio init [--force] [--print] [--package NAME] [--config FILE] [PATH]  # write rosio.yaml: roots, setup files, profiles to edit
rosio extract [--layer {declared,coded}] [--launch-sandbox {auto,required,off}] [--install-tree DIR] [--profile NAME] [--config FILE] [--collector NAME] [--bag PATH] [--no-runtime] [--fail-on-violation] [PATH]
rosio stamps [--topics TOPIC] [--limit N] [--json-verdict] [--stats] [BAG]
rosio probe [--stamps] [--duration SECONDS] [--settle SECONDS] [--domain-id N] [--localhost-only] [--rmw NAME] [--ros-setup FILE] [--profile NAME] [--config FILE] [--atlas FILE] [PATH]
rosio drift [--fail-on {error,warn,info,never}] [--min-severity {error,warn,info}] [--only RULE] [--waivers FILE] [--config FILE] [--atlas FILE] [--limit N]
rosio param [--profile NAME] [--atlas FILE] [--limit N] [SELECTOR ...]
rosio explain [--atlas FILE] [--limit N] [SELECTOR]
rosio query [--atlas FILE] [--limit N] [WORDS ...]
rosio path [--max-hops N] [--atlas FILE] [--limit N] [SOURCE] [TARGET]
rosio unresolved [--want WANT] [--atlas FILE] [--limit N]
rosio graph [--profile NAME] [--focus NODE_OR_CHANNEL] [--depth N] [--wrappers] [--format {text,dot,mermaid}] [--atlas FILE] [--limit N]
rosio diff [--profile NAME] [--rev REV] [--repo DIR] [--cache-rules] [--workspace PATH] [--atlas FILE] [--limit N] [BEFORE] [AFTER]
rosio render [--format {md,html}] [--atlas FILE] [--limit N]
rosio install [--list] [--scope SCOPE] [--root DIR] [--home DIR] [--dry-run] [--force] [TARGET]
rosio uninstall [--scope SCOPE] [--root DIR] [--home DIR] [--dry-run] [--force] [TARGET]
rosio serve [--atlas FILE] [--stdio | --http] [--host ADDR] [--port N] [--api-key KEY] [--path PATH] [--stateless] [--workspace DIR]
rosio hook install [--repo DIR] [--python PATH] [--workspace PATH] [--config FILE] [--force] [--dry-run] [--no-measure]
rosio hook status [--repo DIR]
rosio hook coverage  # Claude Code Stop hook registered by rosio install claude-code
rosio hook uninstall [--repo DIR] [--force] [--dry-run]
rosio hook run [--repo DIR] [--event NAME] [--no-cache]
rosio hook merge [--path PATH] [--result FILE] BASE OURS THEIRS  # the merge driver git calls for atlas.json
rosio bundle create [--out FILE] [--atlas FILE] [--include {findings,cited}] [--note TEXT] [PATH]
rosio bundle open [--into DIR] [--force] FILE
rosio migrate plan [--config FILE] [--from RELEASE] --to RELEASE [PATH]
rosio migrate apply [--config FILE] [--from RELEASE] --to RELEASE [--force] [PATH]
rosio migrate verify [--config FILE] [--from RELEASE] --to RELEASE [--image NAME] [PATH]
rosio version
```

The output flags `--out DIR`, `--json`, `--quiet`, `--deterministic` and `--no-deterministic`
are left out of the lines above; `rosio <verb> --help` says which of them a verb takes.

Use `rosio probe --stamps --duration 10` to check header stamps in the running system.

`probe` detects middleware implementations from node and launcher environments and runs one
isolated child per implementation. Repeat `--rmw NAME` to force the passes, for example
`--rmw default --rmw rmw_zenoh_cpp`. `default` unsets `RMW_IMPLEMENTATION` after sourcing.
Node facts carry `seen_through`, and refused passes remain on the observed run.

`hook` is the git side: it writes the hooks that rebuild the atlas after a commit, a checkout and
a merge, and registers the merge driver that resolves two rebuilds of `atlas.json` without conflict
markers. It is not part of `install`, which registers rosio with an assistant, because the two
share no flag.

Whether to commit `rosio-out/atlas.json` is a choice, and the hooks assume you do not: put
`rosio-out/` in `.gitignore` and each clone keeps its own atlas current. A team that wants the
atlas in history (to review it in a pull request or to run `rosio diff` against a released
one) builds it with `rosio extract` or `rosio hook run`, commits it with the sources it
describes, and lets CI check it is current. A hook leaves a tracked atlas as committed, because
rewriting it after a commit leaves the work tree dirty and stops the next `git checkout`. A
rebuild is an extraction, so it holds no observed or recorded layer: a hook that replaces an
atlas `rosio probe` extended says so on stderr, and `rosio probe` restates it.

Configuration lives in `rosio.yaml`: the roots to read, and one profile per system you actually
run, each naming its entry points and their arguments. No single launch file describes a running
robot, so a profile is an ordered set of entry points rather than one file. Every key, its
default and where the file is looked for are in [configuration.md](configuration.md).

```yaml
roots:
  - kind: install
    path: /opt/ros/jazzy
ros_setup:
  - /opt/ros/jazzy/setup.bash
ignore:
  - /include
  - /opt
  - /lib/python*
profiles:
  - slug: nav2_bringup_composed
    entrypoints:
      - launch: nav2_bringup bringup_launch.py
        args:
          use_composition: "True"
```

`ignore` keeps the headers, vendored libraries and python modules of the prefix out of the index,
which rosio reads nothing from.

## Waiving a finding you have already decided about

A team that cannot silence one known finding turns the whole gate off. `rosio drift` takes a
waiver file, and it is the only verb that does: a waiver decides how a finding is graded, never
whether it is in the atlas, so a machine with the file and a machine without one still extract the
same bytes.

```yaml
# rosio-waivers.yaml
waivers:
  - key: find:extract.input_refused|file:nav2_theta_star_planner%7Cglobal_planner_plugin.xml|-
    reason: upstream packaging defect, reported to nav2
  - key: sha256:dd3e814875c51fa4
    reason: third party package, tracked in ticket ROS-412
```

Two keys and nothing else. A `key` is a finding id or a `sha256:` fingerprint, both printed by
`rosio drift --json`, and a rule id is refused rather than accepted: it reads like one finding and
would silence a whole rule. A `reason` is mandatory.

```bash
rosio drift --waivers rosio-waivers.yaml --fail-on error
```

The flag is repeatable (`--waivers FILE`) and wins whole over the `waivers` key of `rosio.yaml`.
A waived finding is still printed, and under the findings comes what the waivers did:

```
  1 of 2 waivers matched, printed above and not graded by --fail-on
        waived find:extract.input_refused|file:nav2_theta_star_planner%7Cglobal_planner_plugin.xml|-
          upstream packaging defect, reported to nav2
        waiver matched nothing, delete it: sha256:dd3e814875c51fa4
```

A key that matches nothing says so every run, so a waiver that went stale when its finding was
fixed reports itself instead of quietly widening the file.

## Troubleshooting

**The atlas has almost no nodes and no parameters.** Your virtual environment hides the system
packages, so the launch layer never ran. The run summary says so first and names the fix:
recreate the environment with `--system-site-packages`.

**`pytest` ends in a `PluginValidationError` before any test runs.** With a ROS setup sourced,
humble registers `launch_testing` as a pytest11 plugin whose hook signature no current pytest
accepts, so run the suite with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

**`rosio: command not found`.** The install directory is not on your PATH. `pipx ensurepath` or
`uv tool update-shell`, then open a new shell.

**A launch file raised.** Read the reason in the run summary, and `rosio drift` names the fix. A
launch file that needs an argument you did not supply is the common one, and profiles in
`rosio.yaml` are where arguments live. An environment variable such as a robot model name goes
under `env` of the profile, and a package no root holds is installed or added as a root.

**The C++ collector refuses.** `pip install "$HOME/rosio[cpp]"`, from the checkout above.
Without it the coded layer covers Python only, rosio records that rather than pretending
otherwise, and the atlas carries one `extract.pass_incomplete` at error, so
`rosio drift --fail-on error` exits 5.

**`rosio stamps` says it needs the stamps extra.** `pip install "$HOME/rosio[stamps]"`, which
brings `rosbags`, the reader that needs no ROS installation.
