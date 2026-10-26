# Fixtures

`ws_launch/` is the synthetic launch and parameter workspace. Every hard case from
`bench/hard-cases.md` and every launch or parameter gotcha from `bench/recon-facts.md` that a
resolver can get wrong has one named file here, so a test can point at exactly one of them.
`expected/launch/` and `expected/params/` hold the answers.

`ws_static/` is a separate fixture workspace for the source, urdf, interface and plugin side.
It is not described here.

## How the expected files were produced

Every file under `expected/launch/` was written by hand and then cross checked by executing the
launch file with a real `launch` and `launch_ros` from ROS 2 humble, with process spawning
stubbed out, and comparing package, executable, plugin, name, namespace, the remap list in
order, the parameter inputs in order and the gate trail. The 19 walks that resolve a node set
all agree with what launch actually resolves, and the 2 that record an exception were checked
the same way, by confirming the exception type the real machinery raises. The 22nd file,
`legacy_ros1.json`, is a refusal and is never executed.

`expected/params/zoo_chains.json` was written by hand and then cross checked by recomputing
every winner and every shadowed candidate straight out of the yaml files, with the rules stated
inside that file, including value identity and value type. `expected/params/shape_detection.json`
was cross checked against the full yaml inventory of the workspace, so it cannot fall behind a
new file.

The parts no machine can confirm from launch alone, and which are therefore hand derivation of
runtime semantics, are marked in the notes: the `--ros-args` parse in `exec_process`, the
parameter states (`effective`, `inert_by_design`, `owner_name_mismatch`) and the shadow kinds.

## Reading conventions in the expected files

These are the contract. An implementer who disagrees with one of them should say so in a report
rather than quietly write a different value.

* Paths are relative to `ws_launch/`, always with forward slashes.
* `namespace` is `""` when launch did not state one. This is not a stand in for the root
  namespace: `launch_ros` itself keeps `<node_namespace_unspecified>` in that case, the runtime
  name comes from the executable, and inventing `/` here fabricates drift. A namespace stated as
  the empty string, as `composition.launch.py` and `nav_stack.launch.py` do, resolves to `/` and
  is recorded as `/`.
* `name` is `null` when launch did not state one, with `name_source` `executable_default`.
* `fqn` is `null` whenever either half is unknown. It is never guessed.
* `nodes` are sorted by `(namespace, name, pkg, executable, plugin_class)`. Within a node,
  `remaps`, `params` and `gates` keep their real order, because in all three the order is the
  precedence rule.
* `params[].order` is the index in the parameters list after group scoped entries. A group
  scoped `SetParameter` or `SetRemap` entry comes first and is marked in `note` and, for remaps,
  by `scope: global`.
* Required gate fields are `mechanism`, `target` and `period`. The `conditions` strings and the
  `predicates` keys are advisory: the content that matters is which mechanism gated the node and,
  for a filesystem probe, which boolean the run recorded.
* `loadable` describes the file, not the run. `env_required` stays loadable and records an
  exception, `legacy_ros1` is not loadable at all.
* Types are exact. `20` is an int, `20.0` is a float, `""` is a string value, `null` is a null
  value, and none of the three is the same as an absent key.

## ws_launch layout

```
ws_launch/
  .ruff.toml                     fixture files keep os.path and open(), see the comment in it
  src/lx_bringup/                ament_cmake: 16 python launch files plus one ROS 1 XML file, the parameter zoo, one C++ target
  src/lx_sensors/                ament_python: six console_scripts, one launch file, one config
  install/                       an install tree, with one deliberately stale config copy
  tools/launcher.py              a profile composer outside any package
```

`lx_sensors/setup.py` maps the entry point `health_monitor` to the module `watchdog`, so the
executable name and the module name disagree on purpose (recon:params, an entry point whose
console script name and module name differ, and the join key is the entry point, never the
module).

## Launch fixtures

| file | what it encodes | where it comes from |
| --- | --- | --- |
| `plain_node.launch.py` | a plain `Node` with a literal name, two remaps, one parameters file. The `/tf` to `tf` pair is the absolute to relative remap that matters for namespaced deployments | recon:launch, remaps in both nav2 stacks |
| `default_name.launch.py` | no `name=` and no `namespace=`, so both stay unknown and the effective name comes from the executable | HC3, recon:launch gotcha "the observed layer will see node names the launch files never state" |
| `opaque_probe.launch.py` | nodes only inside an `OpaqueFunction` that performs a `LaunchConfiguration`, probes the filesystem with `os.path.exists` and picks a different executable per outcome | HC1, recon:launch gotcha "node identity depends on data files, not only on launch args" |
| `event_gate.launch.py` | nodes only inside `RegisterEventHandler(OnProcessExit(target_action=..., on_exit=[...]))`. The gate process itself is not a node, since it carries no `--ros-args` | HC1, recon:launch, two sensor bringup files that gate a driver on a port probe |
| `timer_chain.launch.py` | a `TimerAction` delaying a node, and a gate whose `on_exit` contains another gate, so reachability is a trail and not a boolean | recon:launch gotcha "ordering only relations exist that no static graph shows" |
| `conditions.launch.py` | `IfCondition` and `UnlessCondition` on two nodes, both branches present, only one alive per argument value | recon:launch, 16 `IfCondition` occurrences; `UnlessCondition` appears nowhere in the real tree, which is exactly why it needs a fixture |
| `group_scope.launch.py` | `GroupAction` with `PushRosNamespace`, plus `SetRemap` and `SetParameter` applied to the group and to nothing outside it | recon:launch, `GroupAction`/`PushRosNamespace`/`SetRemap` are unused in the real tree and the `SetParameter` gotcha is a systematic `use_sim_time` drift |
| `composition.launch.py` | a `ComposableNodeContainer` with two `ComposableNode` descriptions, one namespaced, so a component is a sibling of its container and has no executable | AGENTS 3.4 rule 4, and `nav2_bringup` composed, where `controller_server` is a sibling of `nav2_container` |
| `exec_process.launch.py` | an `ExecuteProcess` that is really a ROS node: `-p`, `-r`, bare `__node:=` and `__ns:=`, and a `--params-file` after the assignments | recon:launch gotcha "nodes started as ExecuteProcess are invisible to node shaped extraction" |
| `include_parent.launch.py`, `include_child.launch.py` | `IncludeLaunchDescription` with `launch_arguments`, a substituted child path, and a child that also stands alone with its own defaults | recon:launch, the include graph and "the include child path itself can be a substitution" |
| `env_required.launch.py` | raises at description build time unless `LX_ROOT` is set. Safe to import, fatal to describe | recon:launch, a bringup file raising `RuntimeError` when its repository root environment variable is unset |
| `legacy_ros1.launch` | ROS 1 XML with `type=`, `rosparam command=load`, `$(find ...)`, `clear_params`, `launch-prefix`, `<group if=>`. Detected and refused, never parsed | recon:launch, the two dead XML files, `launch.unloadable` |
| `param_order.launch.py` | `[file, dict]` so the dict wins and `[dict, file]` so the file wins, over the same key in the same file | recon:params gotcha "parameters list order is the precedence rule", plus the documented `yaml_filename` inversion |
| `no_default_arg.launch.py` | a `DeclareLaunchArgument` with no default, so a walk with no arguments fails while the arguments are visited | HC5, recon:launch, a bringup file whose `params_file` argument has no default |
| `rewritten.launch.py` | local stand ins for `nav2_common` `RewrittenYaml` and `launch_ros` `ParameterFile`, writing a temp file at evaluation time | HC2, recon:params HARD TO PREDICT 3, 4 and 5 |
| `nav_stack.launch.py` | the owner node set for the parameter zoo: six nodes with explicit names and namespaces, two of them fed a base and an overlay in order | recon:params, the base and overlay pair and the depth three costmap shape |
| `sensors_only.launch.py` | a launch file in the second package that resolves its config out of the source tree next to itself, while the same file also sits in the install share | recon:launch, "three launch files resolve config out of the SOURCE tree via realpath of `__file__`" and the install tree being a copy |
| `tools/launcher.py` | a composer outside any package: `ros2 launch` and `ros2 run` strings built by f string interpolation, one naming a launch file that does not exist, one running a node with `--params-file` and `-p`, and one merging the base and the overlay into a temp file itself | HC4, recon:launch "a desktop launcher composes the real profiles" and "references a launch file that does not exist", recon:params "the overlay is merged by the launcher, not by a launch file" |

Three traps inside `rewritten.launch.py` are verified rather than asserted: `autostart` is
rewritten at node level and at nested level because the match is on the bare key at every depth,
`anchor_tag` turns from the string `v1` into the float `2.5` because `convert_types` inspects the
text, and the `use_sim_time` rewrite is a silent no op because the source file has no such key,
which is why the launch file appends an explicit dict after the file.

## Parameter zoo

All under `src/lx_bringup/config/` unless stated.

| file | shape or trap | where it comes from |
| --- | --- | --- |
| `zoo_base.yaml` | SHAPE 1 node name key at depth 2, SHAPE 4 two level costmap key at depth 3, a list value, a list delivered as a quoted string, an explicit empty string, an inert plugin block kept out of `controller_plugins`, a commented out value, and a block whose node name matches no launched node | recon:params SHAPES 1 and 4, a disabled controller block left in the file, `lifecycle_manager` versus `lifecycle_manager_navigation`, `footprint` as a string |
| `zoo_overlay.yaml` | the overlay half: one shadowed key, two restated identical keys, one key the base never had, one list replaced wholesale | recon:params EXACT SHADOW SET, "restated identical keys are common", "an overlay may introduce keys the base never had" |
| `zoo_wildcard.yaml` | SHAPE 2, bare `/**` root, with a nested dict that becomes a dotted name | recon:params SHAPE 2, a diagnostics analyzer file |
| `zoo_node_wildcard.yaml` | SHAPE 3, `/**/os_driver`, matching the node in any namespace including the root one, plus an empty string value | recon:params SHAPE 3, a lidar driver parameter file |
| `zoo_dotted.yaml` | SHAPE 5, dotted keys directly under `ros__parameters` next to the nested dict form, plus a bare key with a null value | recon:params SHAPE 5, two files in one package that spell the same shape both ways |
| `order_base.yaml` | one file read by two nodes in the two possible list orders | recon:params gotcha on list order |
| `rewrite_source.yaml` | has `autostart` at two depths, a string `anchor_tag`, and no `use_sim_time` anywhere | recon:params HARD TO PREDICT 3 and 4 |
| `slam.yaml` | `ceres_loss_function: None`, which yaml resolves to the string `None` and not to null | recon:params, a `slam_toolbox` parameter file |
| `crop.yaml` | the file half of the `--ros-args` chain, keyed on the name the argv gave the node | recon:launch, the `ExecuteProcess` nodes |
| `not_params_map.yaml` | a map descriptor, not a parameter file | HC6, a `nav2_map_server` map descriptor under `maps/` |
| `not_params_list.yaml` | a yaml document whose root is a list | HC6, a `rosdoc.yaml` and a ROS 1 to ROS 2 name mapping file |
| `flat_looks_like_params.yaml` | flat, real parameter names at the root, no node key and no `ros__parameters`. The device address is an RFC 5737 documentation address and names nothing | HC6, three upstream gnss driver configs written this way |
| `imu_driver.yaml`, `filter.yaml`, `lidar_driver.yaml` | ordinary node keyed files, one of which shares its basename with `src/lx_sensors/config/lidar_driver.yaml` and carries a different value | recon:launch, two roots for one basename |
| `install/lx_bringup/share/lx_bringup/config/zoo_base.yaml` | a stale install copy with a different `controller_frequency`, so a mixed root is one `build.stale` finding and not a pile of value drifts | AGENTS 9.9, recon:launch "the install tree contains COPIES, not symlinks" |

## Expected files

| file | contents |
| --- | --- |
| `expected/launch/<stem>.json` | one walk per launch file. `<stem>.<case>.json` when one file has more than one outcome: `opaque_probe.no_map` and `.with_map`, `conditions.default` and `.enabled`, `env_required.missing` and `.set`, `no_default_arg.missing` and `.supplied` |
| `expected/params/zoo_chains.json` | 35 parameters owned by the six `nav_stack.launch.py` nodes, each with its winner, every shadowed candidate, the shadow kind and the state. Five have a shadowed candidate, two are restated identical, one is `inert_by_design`, two are `owner_name_mismatch` with no owner and no effective value |
| `expected/params/argv_chains.json` | the chain of the `ExecuteProcess` node, where the `--params-file` listed after two `-p` assignments wins `rate_hz` |
| `expected/params/shape_detection.json` | which yaml files are parameter files, which are refused and why, and the two install copies |

The cases that need something set up before the walk:

* `opaque_probe.with_map` needs a file at `<map_base>.posegraph` and `map_base` passed as an
  argument. The expected file records the predicate as true.
* `env_required.set` needs `LX_ROOT` in the child environment.
* `no_default_arg.supplied` needs `params_file` passed as an argument.
