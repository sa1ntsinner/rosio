# rosio.yaml

`rosio extract` reads the first of these that exists: the file `--config` names, the file the
`ROSIO_CONFIG` environment variable names, `rosio.yaml` in the workspace or the directory above
it, `rosio.yaml` in the working directory or the directory above it when one of its roots is the
workspace or lies inside it. A file in the working directory written for another tree is
ignored, and extract says so. Without one it extracts one profile per launch file it finds. The run summary names the file it read. `rosio init` writes the
fallback out as a file to edit. A launch file that another launch file of its package includes
and starts with no argument is written commented out, because read on its own it repeats every
finding of the file that starts it.

Relative paths in the file are read from the directory that holds it. A key this page does not
list is refused, so a typing error never passes unnoticed.

## Top level

| key | value | default |
| --- | --- | --- |
| `roots` | the trees rosio reads, a list. Each item is a path, read as a source tree, or a root mapping (below). | `src` and the `install` beside it; a directory that is an installed ROS prefix, such as `/opt/ros/jazzy`, is one install root |
| `ignore` | paths under a root that are skipped. A pattern holding a slash matches the path inside the root from its top, `/include` for one; a pattern without one matches a file or directory name anywhere. | `test` and `tests`, the directories of test sources that no profile starts; for an installed ROS prefix read on its own: `/include`, `/opt` and `/lib/python*`, which hold headers, vendored libraries and python modules that rosio does not read. An empty list skips nothing. |
| `ros_setup` | the setup files sourced before a launch file is walked, a path or a list. An empty list sources nothing. | `/opt/ros/$ROS_DISTRO/setup.bash` and `setup.bash` of the install root, when they exist |
| `launch_timeout` | seconds one launch walk may take. | `60` |
| `launch_sandbox` | `auto` walks launch files inside bubblewrap when it works, `required` refuses to walk without it, `off` (or `false`) never uses it. `--launch-sandbox` overrides it. | `auto` |
| `profiles` | one per system you actually run, a list of profile mappings (below). | one profile per launch file, leaving out ROS 1 and test launch files; an empty list is taken as written and extracts no profile |
| `out` | the output directory, relative to the working directory. `--out` overrides it. Keep it in `.gitignore` when the git hooks rebuild it: a hook leaves a tracked `atlas.json` as committed. | `rosio-out` |
| `out_dir` | the older spelling of `out`. | |
| `deterministic` | accepted for older files. `rosio extract` is deterministic unless `--no-deterministic` is passed. | |
| `waivers` | waiver files `rosio drift` grades with, a path or a list. `--waivers` replaces it. Never read by extract. | none |
| `resolution_budget` | the resolution rate under which `resolution.below_budget` fires: a number between 0 and 1, or a mapping of want (`topic_name`, `param_value`, ...) to number, where `overall` sets the overall rate. A key that names no rate is refused. `rosio unresolved` lists what keeps the rate down. | `0.95` overall |

## A root

| key | value |
| --- | --- |
| `kind` | `src` for sources, `install` for a built or installed tree, `bag` for a directory of recordings. Default `src`. |
| `path` | the directory. A configured `src` or `install` root that does not exist is refused, with the ways out. |

## A profile

| key | value |
| --- | --- |
| `slug` | the name findings and `--profile` use. Required, unique in the file. |
| `label` | one line for people. |
| `entrypoints` | what starts this system, a list of entry points (below) in the order they are started, which is the precedence between them. |
| `env` | environment variables the launch files read, as a mapping (`TURTLEBOT3_MODEL: burger`) or a list of `NAME=value`. `rosio init` writes a commented block for every variable a launch file reads with no default. |

## An entry point

An entry point is a command as a script would write it, `ros2 launch nav2_bringup
bringup_launch.py use_composition:=True`, or a mapping:

| key | value |
| --- | --- |
| `launch` | `<package> <launch file>`, a `ros2 launch` entry point. |
| `run` | `<package> <executable>`, a `ros2 run` entry point. |
| `verb` | `launch` or `run`, for the long form with `pkg` and `target`. |
| `pkg` | the package, long form. |
| `package` | the same as `pkg`. |
| `target` | the launch file or the executable, long form. |
| `args` | launch arguments, a mapping or a list of `name:=value`. |
| `argv` | further words of the command line, a list, such as `--ros-args -p rate:=10`. |

```yaml
profiles:
  - slug: robot
    env:
      TURTLEBOT3_MODEL: burger
      LDS_MODEL: LDS-01
    entrypoints:
      - launch: turtlebot3_bringup robot.launch.py
        args:
          usb_port: /dev/ttyACM0
      - ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## A waiver file

`rosio-waivers.yaml` holds one top level list, `waivers`, and each item has exactly these keys:

| key | value |
| --- | --- |
| `key` | a finding id (`find:...`) or fingerprint (`sha256:...`), both printed by `rosio drift --json`. |
| `reason` | why the finding is accepted. Required. |
