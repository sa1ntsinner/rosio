# ws_code

The fixture workspace for the coded layer. One ament_cmake package, `cx_nodes`, and one
ament_python package, `px_nodes`. Every C++ and rclpy idiom that appears in `bench/recon-facts.md`
section `recon:code` has one named site here, including the ones that cannot be resolved
statically, because reporting ignorance correctly is half of what the layer is measured on.

The answers are in `tests/fixtures/expected/code/`, one JSON file per source file plus
`index.json` with the totals.

```
ws_code/
  .ruff.toml                     fixture modules keep os.environ and unused callback arguments
  src/cx_nodes/                  ament_cmake, 7 translation units, one header, one plugin manifest
  src/px_nodes/                  ament_python, 4 node modules, entry points that rename two of them
```

## What each source file is for

| file | what it encodes |
| --- | --- |
| `cx_nodes/src/cx_literal_endpoints.cpp` | literal topic names with every QoS form in use: `SensorDataQoS()`, `QoS(KeepLast(n))`, a bare integer depth, reliability and durability set explicitly, a QoS held in a variable, and a QoS mutated after construction. Also `create_service`, `create_client`, a literal `create_wall_timer` and a `SubscriptionOptions` fourth positional argument |
| `cx_nodes/src/cx_param_endpoints.cpp` | the five non literal name spellings (a local from `declare_parameter`, a member from `declare_parameter`, a concatenation, a file scope constant, a preprocessor macro), `declare_parameter` with and without a default, `declare_parameters` with a map, both `get_parameter` shapes, a parameter callback, a nested name built by concatenation, a timer period from a parameter, and one loop whose names come from a file on disk |
| `cx_nodes/src/cx_tf_node.cpp` | `TransformBroadcaster` behind a `publish_tf` guard, `StaticTransformBroadcaster`, `Buffer` with `lookupTransform` and `canTransform`, frames from parameters, one frame from a message header, and a node whose name is a constructor argument |
| `cx_nodes/src/cx_lifecycle_node.cpp` | a lifecycle node with an empty constructor body: publishers and the subscription in `on_configure`, the timer in `on_activate`, and a user written `declareOrGetParameter` wrapper |
| `cx_nodes/src/cx_component_node.cpp` | `RCLCPP_COMPONENTS_REGISTER_NODE` with no `main` and no `add_executable`, so the executable name lives only in CMakeLists |
| `cx_nodes/src/cx_layer_plugin.cpp` | `PLUGINLIB_EXPORT_CLASS` wrapped over two lines, and the costmap asymmetry: `declareParameter("topic", ...)` auto prefixes while the read side spells `name_ + "." + "topic"` |
| `cx_nodes/src/cx_wrappers.cpp` | the two known wrapper cases, `diagnostic_updater::Updater` and `message_filters::Subscriber`, in a node with no endpoint call of its own |
| `px_nodes/px_nodes/literal_node.py` | the rclpy literal case: `qos_profile_sensor_data`, a bare depth, a `QoSProfile` variable, a `QoSProfile` mutated after construction, a service, a client and a timer |
| `px_nodes/px_nodes/param_node.py` | `declare_parameter` with and without a default, `declare_parameters` with a namespace, a dotted literal name, a name built with an f string, a topic built with an f string, a module constant topic, a publisher per element of a list valued parameter, a default that is a runtime attribute, and a topic from the environment |
| `px_nodes/px_nodes/tf_node.py` | `TransformBroadcaster` behind `publish_tf`, `StaticTransformBroadcaster`, `Buffer` with `TransformListener`, `lookup_transform` and `can_transform`, and a node name passed in as a variable |
| `px_nodes/px_nodes/wrappers_node.py` | `diagnostic_updater.Updater` and `message_filters.Subscriber`, in the module whose entry point is renamed to `px_health` |
| `px_nodes/px_nodes/tuning.py` | no ROS at all. It exists so that the `gain` default is an attribute of an object built at runtime |

## Reading conventions in the expected files

These are the contract. An implementer who disagrees with one of them should say so in a report
rather than quietly write a different value.

* `path` is relative to the repository root, `package_relative_path` is what goes inside a
  `file` or `sym` id. Ids never contain an absolute path, a temp path or a line number.
* `line_hint` is advisory. It is present so a human can find the site, and it never enters a
  comparison, an id or a digest.
* `name` is the resolved ROS name. `name_state` says how it was resolved and is one of
  `literal_absolute`, `literal_relative`, `resolved_from_parameter`, `resolved_from_constant`,
  `resolved_from_macro`, `resolved_from_concat`, `resolved_from_fstring`,
  `resolved_from_plugin_parameter_default`, `wrapper_constant`, `unresolved`, `not_applicable`.
* A relative name keeps its raw spelling and mints no `topic` id. It is bound only when a remap
  or a namespace resolves it, which is the declared layer's job, not this one's.
* `node_id` is always null here. The code states a node name, never a namespace, and an fqn with
  a guessed namespace is a fabricated identity. `name_source` is `literal` or `argv`.
* A `qos` field that is null means the source did not state that property. It is never the rmw
  default: a QoS reported as fact that the code did not state manufactures drift against the
  observed layer. `qos` itself is null when the whole profile is chosen inside a wrapper.
* A bare integer depth records `history: "keep_last"`, because converting the int to keep last of
  that depth is what the API call means. A `QoS`, `QoSProfile` or preset records only the
  properties the expression names.
* `via_known_wrapper` marks an endpoint that exists but has no `create_publisher` or
  `create_subscription` call. Those endpoints are attributed, never reported as observed only.
* Frames are normalised without a leading slash, tf2 convention.
* Plugin parameters are suffixes bound to a plugin class, never names starting with a dot and
  never node scoped names. The prefix is the plugin instance name from the yaml layer, which is
  why one `unresolved` entry with reason `plugin_instance_prefix` sits next to them.
* Types are exact. `1` is an int, `1.0` is a float, `""` is a string value, `null` is a null
  value, and none of the three is the same as an absent key.
* Lists are sorted: `endpoints` by (kind, name, raw) with a null name last, `parameters` and
  `plugin_parameters` by suffix, `tf.broadcasts` by (parent, child), `tf.lookups` by
  (api, target, source), `unresolved` by (want, expr).

## The nine unresolved entries, and why each one is honest

| where | want | expression | why nothing can resolve it here |
| --- | --- | --- | --- |
| `cx_param_endpoints.cpp` | `topic_name` | `name` | the loop reads a descriptor file, so both the names and the number of subscriptions come from data |
| `cx_param_endpoints.cpp` | `param_value` | `tuning_.gain` | the default is an attribute of a type defined in another package |
| `cx_tf_node.cpp` | `frame_id` | `msg->header.frame_id` | the frame is whatever arrived on the topic |
| `cx_tf_node.cpp` | `node_name` | `node_name` | the base constructor is handed a constructor argument filled from argv |
| `cx_layer_plugin.cpp` | `other` | `name_ + "."` | the parameter prefix is the plugin instance name chosen by the yaml |
| `param_node.py` | `topic_name` | `env_topic` | the name, and the existence of the endpoint, come from the process environment |
| `param_node.py` | `param_value` | `self._tuning.gain` | the default is built by `load_tuning` from a file named in the environment |
| `tf_node.py` | `frame_id` | `msg.header.frame_id` | the frame is whatever arrived on the topic |
| `tf_node.py` | `node_name` | `node_name` | the name is a constructor argument filled from an argument parser |

Everything else in this workspace is resolvable with a single function dataflow pass plus module
level literal evaluation, which is the same boundary the reference stack draws.

## Things that are deliberately not what they look like

* `cx_marker_node` has no `main` and no `add_executable`. A node inventory built from files with
  a `main` misses it, and a join to a launch file that names `executable='cx_marker_node'` fails
  unless CMakeLists is read.
* `px_health` is the entry point of the module `wrappers_node`. The join key from a launch file
  is the console_scripts entry point, never the module name.
* The lifecycle node creates nothing in its constructor. Walking constructors finds no endpoints
  in that file at all.
* `PLUGINLIB_EXPORT_CLASS` is wrapped over three lines, so a single line regex misses it.
* Neither `PLUGINLIB_EXPORT_CLASS` nor `RCLCPP_COMPONENTS_REGISTER_NODE` ends in a semicolon,
  which is how both are written on the reference stack. A C++ grammar reports a missing `;` at
  both sites, so an extractor that refuses a file with a parse error refuses two real files here.
* `cx_layers_plugin.xml` declares `<library path="cx_nodes_layers">`, which is a CMake target
  name and not the package name.
* `cx_core` is not in this workspace. The plugin base class and the tuning type resolve to
  nothing here, which is the normal case for a first party package that extends a third party one.
