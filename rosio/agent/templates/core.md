`rosio` answers questions about the ROS 2 workspace in this repository from an atlas it
built beforehand. The atlas holds four layers: declared (launch files, yaml, urdf), coded
(the C++ and python sources), observed (a live probe) and recorded (rosbag2). Answers come
from the CLI, not a model: no tokens, no network, the same question returns the same bytes.

The rule that matters: a yaml value is usually not the value that reached the node. Overlay
files, launch inline dicts, `--ros-args` and launch time rewrites all outrank the base yaml.
Ask rosio for the effective value and the file to edit. Reach for grep only after rosio
says it could not resolve something.

Commands worth knowing, all read only:

```
rosio param <node> <name>        the candidate chain behind one parameter, winner first
rosio query <text>               find a node, topic, parameter, frame, plugin or symbol
rosio explain <id or finding>    one entity or one finding in full, with provenance
rosio drift --min-severity warn  where the layers disagree, each with a fix
rosio graph --focus <node>       who talks to what, with QoS
rosio migrate plan --to jazzy    what a release upgrade changes, with file:line and source
```

Run `rosio migrate apply --to <release>` and make the by hand changes. Finish with
`rosio migrate verify --to <release>`.

A finding already accepted here lives in `rosio-waivers.yaml`, a `waivers:` list of `key`
(a finding id or a `sha256:` fingerprint, both printed by `rosio drift --json`) and a
mandatory `reason`. `rosio drift --waivers rosio-waivers.yaml` prints those anyway with
the reason, stops `--fail-on` grading them, and names every key that matched nothing, so a
stale waiver reports itself. Propose a waiver with a reason rather than lowering `--fail-on`.

Add `--atlas <path>/atlas.json` when the atlas is not under `./rosio-out`. If there is no
atlas yet, build one with `rosio extract <workspace src> --out rosio-out`, which reads
files only and starts no ROS node.

Worked example, from the `nav2_bringup` that ships with ROS 2 jazzy, in a profile that runs
`bringup_launch.py` with `use_composition:=True`. Question: does the controller run on
simulated time?

```
$ rosio param /controller_server use_sim_time --profile nav2_bringup_composed
use_sim_time on /controller_server   state effective
  profile nav2_bringup_composed
    effective False (bool)   edit nav2_bringup/launch/navigation_launch.py:253 [install]
    rank 0 wins launch_inline_dict   False (bool)
        nav2_bringup/launch/navigation_launch.py:253 [install] SetParameter
```

No parameters file sets it. Line 253 of `navigation_launch.py` is a `SetParameter` in the
group that loads the components, and it hands each of them the `use_sim_time` launch
argument, false unless the caller passes it. Grepping the yaml finds nothing.

Answer from that output and stop. If rosio reports a value as unknown or unresolved, say
so instead of quoting a number out of a yaml file.
