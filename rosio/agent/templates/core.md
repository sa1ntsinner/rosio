`rosio` answers questions about this ROS 2 workspace from a prebuilt atlas of four layers:
declared (launch, yaml, urdf), coded (C++ and python), observed (a live probe) and recorded
(rosbag2). The same question returns the same bytes.

The rule that matters: a yaml value is usually not the value that reached the node.
Overlays, launch inline dicts, `--ros-args` and launch rewrites outrank the base yaml. Ask
rosio for the effective value and the file to edit.

```
rosio param <node> <name>        the candidate chain behind one parameter, winner first
rosio query <text>               find a node, topic, parameter, frame, plugin or symbol
rosio explain <id or finding>    one entity or one finding in full, with provenance
rosio explain <file>:<line>      what that line sets and whether it wins
rosio drift --min-severity warn  where the layers disagree, each with a fix
rosio graph --focus <node>       who talks to what, with QoS
rosio migrate plan --to jazzy    what a release upgrade changes, with file:line
```

To fix a finding:

1. Run `rosio explain <file>:<line>` on the line you mean to edit. It says whether each
   value there wins or which line outbids it.
2. `edit <file>:<line>: replace 'a' with 'b'` under a finding is an exact fix, and
   `rosio drift --fix` writes every one of them.
3. Run `rosio extract` again. It lists the findings that went and the ones that arrived.
   The fix is done when its finding is gone and nothing new arrived.

Over MCP the same steps are the `drift`, `explain` and `check` tools.

In the rosio skill folder (`.claude/skills/rosio/` or `.agents/skills/rosio/`): read
`review.md` before hunting bugs or reviewing, `coding.md` before writing or debugging code.

An accepted finding goes in `rosio-waivers.yaml`: a `waivers:` list of `key` (finding id or
`sha256:` fingerprint) and a mandatory `reason`. `rosio drift --waivers rosio-waivers.yaml`
stops grading those and names every key that matched nothing, so a
stale waiver reports itself. Propose a waiver with a reason rather than lowering `--fail-on`.

Add `--atlas <path>/atlas.json` when the atlas is not under `./rosio-out`. With none,
`rosio extract <workspace src> --out rosio-out` builds one; it starts no ROS node.

Example, the `nav2_bringup` jazzy ships, profile `bringup_launch.py` with
`use_composition:=True`. Does the controller run on sim time?

```
$ rosio param /controller_server use_sim_time --profile nav2_bringup_composed
use_sim_time on /controller_server   state effective
  profile nav2_bringup_composed
    effective False (bool)   edit nav2_bringup/launch/navigation_launch.py:253 [install]
    rank 0 wins launch_inline_dict   False (bool)
        nav2_bringup/launch/navigation_launch.py:253 [install] SetParameter
```

Line 253 is a `SetParameter` handing each component the `use_sim_time` launch argument;
grepping yaml finds nothing. If rosio says a value is unknown, say so instead of quoting a yaml
number.
