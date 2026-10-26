# The reference workspace, and the file that describes it

Six tests read a real ROS 2 workspace instead of the committed fixtures. A fixture workspace is
written to exercise one behaviour at a time and it is always small enough to be wrong in a way
nobody notices. A real workspace is the only input that carries hundreds of packages, a build tree
that is current with its sources, launch files that were written by people who were not thinking
about this tool, and enough parameter yaml for the chain to be worth resolving. Those six tests
are where rosio is checked against that.

No such workspace can be committed here, and nothing about the one the authors use may be written
down in this repository. So the tests take both the workspace and what it should yield from the
environment, and skip when neither is there. This document is the schema of that description, so
that anybody with a workspace of their own can point the suite at it.

## Pointing the suite at a workspace

Two variables, both absolute paths.

| variable | what it names |
| --- | --- |
| `ROSIO_REFERENCE_WS` | a directory holding `src/` and `install/`, a colcon workspace that has been built |
| `ROSIO_REFERENCE_EXPECT` | a json file, the shape below, describing that workspace |

```bash
export ROSIO_REFERENCE_WS=/path/to/ws
export ROSIO_REFERENCE_EXPECT=/path/to/reference-expect.json
pytest tests/test_workspace.py -k reference
```

The json file belongs outside this checkout. It names packages and files of a tree that is not
public, and a path inside the repository is an invitation to commit it.

All six read both variables, and each skips with the name of the variable it wanted, so a run
that is missing one says which.

## What the file holds

Every key is required. The tests read them by name and a missing key is a `KeyError`, not a skip,
because a half written description is worse than none.

| key | type | what it asserts |
| --- | --- | --- |
| `package_count` | int | how many packages `Workspace.discover` finds over both roots |
| `build_types` | object, package to string | the `build_type` of each named package, `ament_cmake`, `ament_python` or `cmake` |
| `executables_contain` | object, package to list | each executable listed is among that package's, others are allowed |
| `executables_exact` | object, package to list | the package's executables are exactly this list, in sorted order |
| `plugin_manifests` | object, package to list | the base names of that package's pluginlib manifests, in order, and each file exists. An empty list asserts that a package exports none |
| `launch_file_count` | int | launch files found in the whole tree, `.launch.py`, `.launch.xml` and `.launch` together |
| `legacy_launch_count` | int | how many of those end in a bare `.launch`, which is the ROS 1 shape |
| `param_file_count` | int | files carrying a `ros__parameters` key |
| `param_file_names` | list | base names that must be among the parameter files |
| `not_param_file_names` | list | base names that must not be, yaml that looks like parameters and is not |
| `share_package` | string | one package to check share reads against, see the constraint below |
| `share_config_names` | list | base names under that package's installed `share/<package>/config` |
| `repo_dir_name` | string | the base name of the directory `Workspace.repo_root` resolves to |
| `launch_file_package` | string | the package owning the launch file the id check uses |
| `launch_file_relpath` | string | that launch file's path relative to its package |

Two constraints are worth stating because both cost a debugging round to find.

`share_package` must be a package whose source directory carries the same name as the package
itself. The share read check builds `<ws>/src/<share_package>/config/<name>` by hand and compares it
with what `Workspace.cite` returns, so a package declaring `foo_msgs` from a directory named
`FooMsgs` fails on the spelling and not on the behaviour.

`share_config_names` must be files the build actually copied, unchanged, from the source tree. The
test asserts that citing the installed copy lands on the editable file and that every share read is
identical, which is a statement about a current build. Rebuild before running it.

## An example

Names invented for this page. Nothing here is a real workspace.

```json
{
  "package_count": 12,
  "build_types": {
    "yard_bringup": "ament_cmake",
    "yard_drivers": "ament_python"
  },
  "executables_contain": {
    "yard_drivers": ["lidar_driver", "imu_driver"]
  },
  "executables_exact": {
    "yard_planner": ["route_planner"]
  },
  "plugin_manifests": {
    "yard_planner": ["route_planner_plugin.xml"],
    "yard_drivers": []
  },
  "launch_file_count": 19,
  "legacy_launch_count": 1,
  "param_file_count": 14,
  "param_file_names": ["nav2_params.yaml", "ekf.yaml"],
  "not_param_file_names": ["package_index.yaml", "rosdoc.yaml"],
  "share_package": "yard_perception",
  "share_config_names": ["yard_perception.yaml"],
  "repo_dir_name": "yard",
  "launch_file_package": "yard_bringup",
  "launch_file_relpath": "launch/robot_state_publisher.launch.py"
}
```

## Why the numbers are not in the test file

They were, until the workspace gained launch files and the assertion that counted them had
been failing for long enough that nobody looked at it any more. A
count of somebody else's tree is not a constant, and writing it into a test turns every ordinary day
of work on that tree into a red suite. Keeping it in a file the owner of the tree maintains puts the
number where the tree is, and leaves the test asserting the behaviour: that discovery finds every
package, that a build type is read from the manifest and not guessed, that an installed config is
cited on its editable source.

Restate the file when the workspace moves, and restate it from a measurement.
