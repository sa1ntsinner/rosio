# rosio contract

This file is the source of truth for every later agent and every later milestone. If code
and this file disagree, the code is wrong. Read it fully before writing anything under
`rosio/`.

rosio builds a four layer atlas of a ROS 2 system and reports the drift between the
layers. The core is fully deterministic. There is no language model anywhere in
extraction, join, drift or rendering. Any agent, including a small local model, consumes
`rosio-out/atlas.json` and the CLI subcommands.

## 1. Hard rules

1. Python 3.10 is the target (ROS humble ships 3.10.12). No syntax that needs 3.11 or
   later. `from __future__ import annotations` is allowed and preferred.
2. The only core runtime dependency is pyyaml. Everything else is an optional extra.
   Never import `rclpy`, `launch`, `launch_ros`, `rosbag2_py`, `tree_sitter` or any other
   optional package at module import time in a core module. Import inside the function
   that needs it, and raise `CollectorError` when it is missing.
3. Distribution name on PyPI is `rosio`, the python module is `rosio`, the CLI
   command is `rosio`. Licence Apache-2.0.
4. Prose style, in every file including code comments, docstrings, reports and commit
   messages: no em dashes, and no dash used as punctuation. Use commas, colons or
   parentheses. Plain engineer English, no marketing adjectives, no bullet padding.
5. Never mention AI assistants or AI tooling as the author of any code, comment, document
   or commit. Describing coding agents as the intended audience of the product is expected
   and fine.
6. Comments are rare and short. Self explanatory code beats commentary.
7. Determinism: every output is sorted and stable, and no wall clock value is ever inside
   compared data. See section 9.
8. Every module that raises does so with a class from `rosio.errors`. Do not invent new
   exception hierarchies.
9. Nothing about the workspace this tool was developed against ships. No file path, package
   name, node name, address, contract or configuration value from a private stack may appear
   in the wheel, in the sdist, in the templates the installers copy into a user's repository,
   or in any published document. Measurements against it live in `bench/`, which is not
   tracked. A worked example comes from something the reader can open, and `nav2_bringup`
   ships with ROS 2 humble.

## 2. The four layers

A layer is a source of claims, not a stage of a pipeline. A layer collector never reads
another layer's output and never reads another collector's atlas entries. Joining happens
once, afterwards, in `rosio/join.py`.

Independence stops there, and section 2.1 states what it stops at. Two collectors of the
declared layer hand records to a third through a shared bundle, which is an ordering
obligation between collectors and not a layer reading a layer. This file said the collectors
were independent for six milestones while `extract.ORDER` put one before the other and a
docstring carried the reason, so the rule a contributor read and the rule the code kept were
two different rules.

| layer | what it claims | how it is produced |
| --- | --- | --- |
| `declared` | what the repository says should run: launch files, yaml parameter profiles, urdf and xacro, interface definitions, pluginlib manifests, and the non launch entry points that compose real profiles (a desktop launcher building `ros2 launch` and `ros2 run` command strings) | static reading plus sandboxed execution of `generate_launch_description` |
| `coded` | what the C++ and Python sources actually publish, subscribe, declare, read and broadcast | AST and source analysis, never execution |
| `observed` | what a live system reports: node names, endpoints with QoS, effective parameter values, TF, rates, lifecycle state | a short rclpy probe |
| `recorded` | what a real run contains: rosbag2 metadata and per topic statistics | rosbag2 metadata reading |

The observed probe runs one isolated child per middleware stated by process and launcher
environments. Repeat `rosio probe --rmw NAME` to force the set of passes. `default` unsets
`RMW_IMPLEMENTATION` after sourcing setup files. Node facts carry `seen_through`; a process
table claim retains authority over `rmw`. The run keeps `middleware_passes` with refusal
reasons and the separate `observations`, so disagreement between passes stays readable.

Two rules about layer claims:

* A layer never creates a parallel entity for something another layer already named. It
  contributes a `LayerFact` on the existing id.
* A layer that could not determine a value says `unknown` and stops. A fabricated default
  (a QoS depth of 10, a missing namespace assumed to be root) manufactures false drift,
  which is the fastest way to make a drift report get ignored.

### 2.1 The declared bundle, the one handoff between collectors

`collect.CollectContext.declared` carries `resolve.records.DeclaredBundle`, three lists of
intermediate records. It is the only channel between collectors and it never holds an atlas
id.

| list | written by | read by | what it holds |
| --- | --- | --- | --- |
| `walks` | `launch_declared` | `params` | one `LaunchWalk` per entry point: the nodes, remaps and parameter sources one launch walk produced |
| `scraped` | `entrypoints` | `params` | the `ros2 launch` and `ros2 run` commands a non launch composer builds |
| `pipelines` | `entrypoints` | `params` | the parameter documents a launcher assembles in python before launch sees one |

The obligation that creates, in full:

* A producer may read the bundle, because appending to a list means reading it first. A
  consumer may not write it.
* Every producer runs before every consumer. `extract.ORDER` is where that order is
  committed, and it is the only dependency between the collectors of a run.
* A consumer whose producer has not run raises `ContractError` from
  `CollectContext.require_bundle` and its pass is recorded as `refused`, which
  `extract.pass_incomplete` grades at `error`. This is the case the rule exists for: an empty
  bundle reads exactly like a workspace that launches nothing, so a parameter pass that had
  read nothing was indistinguishable from one that had read everything, and the atlas that
  came out of it was thin rather than wrong.
* A producer that was reached and had nothing to read opens the bundle anyway. A workspace
  that declares no profile is not a workspace nobody read, so the consumer that follows it is
  `not_attempted` over an empty bundle rather than refused.
* A caller that hands a bundle in owns it, which is how one collector is tested against
  records a test wrote. Only a collector the registry hands out is bound to the rule, and
  `collect.BUNDLE_WRITERS` and `collect.BUNDLE_READERS` are the two halves, named there.

### 2.2 The coded record block, the one shape the join consumes

A coded collector states what a file or a symbol holds as a record block, in the `values` of
a `coded` `LayerFact` or, for a collector that records an inventory rather than a fact, in
the entity's `attrs`. `join.RECORD_KEYS` is the block and this is the whole of it. Every key
is optional, every value is a list of flat rows, and a key the block does not name is not a
record and is not read.

| key | one row |
| --- | --- |
| `nodes` | `{name, name_source, namespace, namespace_source, form, symbol, class}` |
| `classes` | `{name}`, a qualified C++ class or struct declaration, including classes with no ROS records |
| `class_templates` | `{name, definitions, specialized}`, the count of class template bodies in one file and whether a class or member specialization exists; explicit instantiations and forward declarations contribute no definition |
| `constructions` | `{class, owner_symbol, line_hint, definition_file}`, the literal type constructed in `main`, as written, with an optional file id for a unique class in an included header of the same package |
| `type_aliases` | `{name, target, namespace, line_hint}`, a namespace scope using or typedef alias with its qualified name and target spelling |
| `endpoints` | `{kind, raw, name, name_state, type, type_id, qos, owner_symbol, via_known_wrapper, condition, guard_parameters, line_hint}` |
| `parameters` | `{name, type, default, has_default, default_state, reads, owner_symbol}` |
| `parameter_reads` | `{name, api, target, owner_symbol, line_hint}` |
| `plugin_parameters` | `{base_class, lookup_name, suffix, type, default, has_default, default_state, reads, owner_symbol}` |
| `tf_broadcasts` | `{kind, parent, child, parent_state, child_state, broadcaster, condition, guard_parameters, parent_from_parameter, child_from_parameter, owner_symbol, line_hint}` |
| `tf_lookups` | `{api, target, source, target_state, source_state, target_from_parameter, source_from_parameter, owner_symbol, line_hint}` |

* `kind` on an endpoint is one of `publisher`, `subscriber`, `service_server`,
  `service_client`, `action_server`, `action_client`, `timer`.
* `name` is the raw name the source states, absolute or relative, absent when the source
  could not be read statically. A record that already states the name its wrapper creates is
  taken as it stands.
* A `parameter_reads` row is a read of a parameter no declaration in the same file carries,
  which is what tells a declaration apart from a value the code actually uses.
* `parameters` and `plugin_parameters` are two record kinds and not one list: a row carrying
  a base class or a suffix is a suffix only declaration bound to a plugin and never becomes a
  node parameter. The join reads both at one call site, `Carrier.declarations`, because the
  row decides which it is.
* The carrier is an `exe`, a `sym` or a `file` entity, and `join.carriers` merges the rows an
  entity states in its attrs with the rows its `coded` facts state, by content, so a record
  stated twice is read once.

A row may carry `construction`, a content digest over the constructing file, class and
ordered arguments. Records carrying it belong only to that context, while `owner_symbol`
continues to name the real source symbol. The join selects contexts from the executable's
node records before binding endpoints, parameters or TF. Multiple constructed nodes do not
become one declared node. Identical construction content carries `construction_ambiguous`
on the node row and remains a blind spot, never an ordinal or a combined endpoint site.

Endpoint records may carry `parameter_suffix` beside `from_parameter`. TF records use the
corresponding `<role>_parameter_suffix` and `<role>_expr`. A string accessor preserves the
parameter dependency, and the join resolves it from that profile's candidate chain before
adding the suffix. A source blind spot retires only when every selected profile and context
resolved that source expression.

`condition` is source text and nothing evaluates it. A record whose guard is exactly one
parameter of the node, possibly negated, or a conjunction of such terms, also carries
`guard_parameters`, one `{name, value}` row per term naming the bool the term needs. A guard
with any other term carries none, because one spelling (`debug`) can be a parameter or a local
and the text cannot say which. The join reads each term from that profile's candidate chain,
a code default included when it wins: an endpoint or broadcast whose term reads the other
bool is not bound in that profile, and a term whose value is unknown or not a bool binds the
record as before. It is not bound rather than bound and marked, because every coded edge is
already one profile's claim that the node does this, and a claim carrying its own denial
would have to be filtered by every rule that reads an endpoint. A launch site differs: it is
an entity that exists whatever its gate says, so it keeps the gate and `rules.site_live`
reads it.

Every record key has exactly one spelling and there is no fallback. The last one outside the
block was a symbol fact whose node rows sat under `node` while the file fact of the same pass
wrote them under `nodes`; both collectors now write `nodes` and the compatibility path in the
join is gone. Reintroducing a second spelling is a contract violation and not a convenience,
because the join reads the block by name and a key the block does not name is silently not a
record. Three other spellings went with the alias table that held them, `plugin_parameters` into a kind of its own and `broadcasts`,
`lookups` and the nested `tf` block they sat in into nothing, because no collector ever wrote
that block into a fact.

## 3. Identity

### 3.1 Grammar

```
id       = kind ":" body
kind     = [a-z][a-z0-9_]*         recoverable from the text before the first colon
body     = segment ("|" segment)*  ordered, kind specific, segment count checked
segment  = printable ASCII, with "%", "|" and "~" always percent escaped
```

`|` is the single reserved separator inside a body, escaped as `%7C` wherever it appears in
a name. `%` is escaped as `%25`, and `~` as `%7E` so that only the synthetic marker below can
ever be a raw `~`. Anything outside printable ASCII is percent escaped byte by byte.
Therefore an id is always typeable, greppable and splittable, and `rosio.ids.parse` round
trips every id built by `rosio.ids.build*`.

A body whose first character is `~` carries a **synthetic identity**: the runtime name of
the entity either is not known yet or is known to be randomised at runtime. Synthetic ids
have their own segment counts (see `ids.SYNTHETIC_SEGMENTS`) and never appear in human
facing prose.

Three refusals, enforced by `rosio/ids.py`:

* No id embeds another id's kind prefix, with exactly two exceptions: an edge endpoint and
  a finding subject, which are complete entity references by definition. A parameter is
  `param:<owner node body>|<dotted name>`, never `param:node:...`.
* No id contains an ordinal, a line number, an absolute path, a temp path or a clock.
  Repeated things are separated by a digest over their own distinguishing content, so
  inserting a sibling adds one id and changes none. `ids.build_launchent` and
  `ids.build_finding` refuse a bare integer discriminator.
* No id contains a python local variable name. Launch step discriminators come from the
  executed action's own attributes (a declared name, an executable, or a digest), because a
  local name does not survive `generate_launch_description`.

A digest frames what it covers, and a separator is not a frame. `ids.digest` states the
length of every part before the part, so no single part can answer for two, whatever byte it
holds: a command line value carrying a NUL used to make `digest('a', 'b')` and
`digest('a\x00b')` one value. `ids.digest_fields` states a sequence's length and escapes
every member. A caller that joins its members into one string before it gets here, with a
space, a comma or a newline, has already lost the boundary and the framing below it cannot
give it back, so spread the sequence and anchor each member to what it belongs to. Every
digest over a sequence is one of those two functions, or it states why not.

### 3.2 Id table

| kind | form | example |
| --- | --- | --- |
| `pkg` | `pkg:<name>` | `pkg:nav2_bringup` |
| `file` | `file:<pkg>\|<package relative path>` | `file:nav2_bringup\|params/nav2_params.yaml` |
| `file` (no package) | `file:@repo\|<repo relative path>` | `file:@repo\|local/lib/python3.10/dist-packages/nav2_common/launch/rewritten_yaml.py` |
| `sym` | `sym:<pkg>\|<relpath>\|<qualified name>` | `sym:nav2_controller\|src/controller_server.cpp\|nav2_controller::ControllerServer::on_configure` |
| `exe` | `exe:<pkg>\|<executable>` | `exe:nav2_controller\|controller_server` |
| `launchfile` | `launchfile:<pkg>\|<relpath>` | `launchfile:nav2_bringup\|launch/bringup_launch.py` |
| `launchent` | `launchent:<pkg>\|<relpath>\|<Action(disc)>...` | `launchent:nav2_bringup\|launch/bringup_launch.py\|Include(navigation_launch.py)\|ComposableNode(controller_server)` |
| `profile` | `profile:<slug>` | `profile:nav2_bringup_composed` |
| `node` | `node:<fully qualified name>` | `node:/controller_server` |
| `node` (synthetic) | `node:~<pkg>\|<exe>\|<namespace>\|<role digest>` | `node:~nav2_controller\|controller_server\|/\|75ed3551` |
| `topic` | `topic:<resolved absolute name>` | `topic:/cmd_vel_nav` |
| `service` | `service:<resolved absolute name>` | `service:/navigate_to_pose/_action/cancel_goal` |
| `action` | `action:<resolved absolute name>` | `action:/navigate_to_pose` |
| `param` | `param:<owner node body>\|<dotted name>` | `param:/local_costmap/local_costmap\|width` |
| `pparam` | `pparam:<base class>\|<lookup name>\|<suffix>` | `pparam:nav2_core::Controller\|dwb_core::DWBLocalPlanner\|BaseObstacle.scale` |
| `frame` | `frame:<frame id, no leading slash>` | `frame:base_scan` |
| `type` | `type:<pkg>\|<msg\|srv\|action>\|<Name>` | `type:nav2_msgs\|action\|NavigateToPose` |
| `plugin` | `plugin:<base class>\|<lookup name>` | `plugin:nav2_costmap_2d::Layer\|nav2_costmap_2d::ObstacleLayer` |
| `pinst` | `pinst:<owner node body>\|<instance>` | `pinst:/controller_server\|FollowPath` |
| `run` | `run:<layer>\|<profile slug>\|<input digest>` | `run:declared\|nav2_bringup_composed\|9f2c1a4b` |
| `bag` | `bag:<directory basename>` | `bag:rosbag2_2026_08_12-14_03_05` |
| `bagtopic` | `bagtopic:<bag>\|<topic>` | `bagtopic:rosbag2_2026_08_12-14_03_05\|/scan` |
| `edge` | `edge:<kind>\|<src id>\|<dst id>\|<key or ->` | `edge:publishes\|node:/controller_server\|topic:/cmd_vel_nav\|a1b2c3d4` |
| `find` | `find:<rule>\|<subject id>\|<disc or ->` | `find:param.shadowed\|param:/planner_server%7CGridBased.tolerance\|-` |
| `unres` | `unres:<digest of site, expression and want>` | `unres:3f9a1c22` |

Only the kinds in `ids.ENTITY_KINDS` live in `atlas.entities`. `edge`, `find` and `unres`
are handles, not entities.

### 3.3 Names

`rosio/ids.py` owns every name rule, and every layer uses the same functions. Do not
write a second normaliser anywhere.

* `normalise_ros_name(name, namespace, node_name)` resolves absolute names, relative names,
  and private `~` names. It is used for source literals, remap targets **and relative names
  that appear in parameter values** (a `node_names` entry such as `costmap/costmap`, a
  `sensors.*.topic`, a `cmd_vel_in_topic`). Without one shared rule,
  `node:costmap/costmap` and `node:/costmap/costmap` become two ids for one node.
* A relative or unresolved topic name never mints a `topic` id. It stays as `raw_name` on
  the endpoint edge, and the destination is bound only when a remap or a namespace resolves
  it. If nothing resolves it, the site becomes an `unresolved` entry.
* `normalise_param_name` folds nested yaml dicts and dotted yaml keys into one dotted name
  before any id is formed or any comparison is made.
* `param_key_matches(key, node_fqn)` is the only wildcard rule: an exact node name, `/**`,
  and `/**/<node>` where the wildcard means zero or more namespace levels and must also
  match the root namespace.
* `normalise_frame` strips the leading slash, tf2 convention.

### 3.4 Node identity, which is the sharpest edge in the whole design

The fully qualified name alone is not identity. Verified on the reference stack: every
`static_transform_publisher` gets a runtime suffix appended to its name, and an unnamed
`rviz2` randomises. Those are exactly the nodes that own the frame layer.

Rules:

1. `node:<fqn>` is used only when the name is a stable literal (a launch `name=`, or a
   literal passed to `super().__init__` or the `rclcpp::Node` base constructor).
2. Otherwise the id is synthetic: `node:~<pkg>|<exe>|<namespace>|<role digest>`, where the
   role digest comes from `ids.role_digest(...)` over the node's own distinguishing content
   (argv, remaps, the ordered parameters list, the declared parent and child frame pair).
   A randomised name family joins on the role digest and never on the name.
3. `name_source` is mandatory on every node entity and takes one of `literal`,
   `executable_default`, `argv`, `observed`, `runtime_randomised`, `unknown`.
4. Composable nodes take their namespace from `launch_ros`, not from the container:
   `make_namespace_absolute(prefix_namespace(ros_namespace, description.node_namespace))`.
   In `nav2_bringup` composed, the `controller_server` component is `/controller_server`,
   while its container is `/nav2_container`. A component has no
   executable, so its path to code is plugin lookup name, then registered class, then
   `sym`, never through an `exe` id.
5. Third party executables whose source is not in the workspace need a committed name
   resolution table (`nav2_controller/controller_server`,
   `nav2_collision_monitor/collision_monitor`, `nav2_map_server/map_saver_server`, and so
   on). It lives in `rosio/resolve/known_names.py` and is data, cited as evidence.

### 3.5 Supersession

When a synthetic identity resolves to a real one, call `AtlasBuilder.supersede(old, new)`.
It merges facts and attrs, keeps the old id in `aka`, rewrites every edge endpoint,
cascades to every derived id (parameters and plugin instances of that node) and deletes the
old entity, in one pass, with one survivor.

An unresolved entry is keyed by a digest over its owner, so a supersession decides its id
too, and three synthetic identities of one process resolving to one name land on one entry.
`supersede` merges those into it, keeping every evidence row and every `resolvable_by` hint,
and the result is the same whatever order the identities resolve in. Two rows sharing one id
is a schema violation, and collapsing them afterwards in the orchestrator only hid the ones
that happened to be byte identical.

* Ambiguous matches must not call it. Keep both entities and emit exactly one
  `node.ambiguous_identity` finding. Silently merging two unnamed static publishers in one
  namespace fabricates TF edges.
* A regression from a resolved id back to a synthetic id is an `error` severity finding
  (`node.identity_regressed`), never a silent id change.

## 4. Entities

`atlas.entities` is one flat map keyed by id. Kind is always recoverable from the prefix.
Every entity carries `id`, `kind`, `label`, `layers`, `profiles`, `agreement`, `conflicts`,
`attrs` and `facts`. Parameters additionally carry `state`, `candidates` and `effective`.
Nodes carry `name_source`. Anything superseded carries `aka`. Anything with a blind spot
carries `unresolved`.

Populated in M1 and M2: `pkg`, `file`, `sym`, `exe`, `launchfile`, `launchent`, `profile`,
`node`, `topic`, `service`, `action`, `param`, `pparam`, `frame`, `type`, `plugin`, `pinst`,
`run`.

Reserved, in the closed vocabulary already so that M5 is an additive minor: `bag`,
`bagtopic`.

**A refused entity has one shape, and it is attributes.** An input a collector could not
read carries `attrs.loadable = False`, `attrs.refused_because` (a short code from that
collector's own closed set, never a message) and, when the reader said something worth
keeping, `attrs.refused_detail` with every machine path replaced by the entity id. It
carries **no fact**. A refusal is a statement about the reader and not a claim any layer
made about the system, so the entity's `layers` stays empty and a consumer filtering by
layer meets either every refusal or none. The other shape, a `LayerFact` carrying
`parsed: false` and a reason, is retired: it was written by `collect/params.py` and
`collect/py_code.py` while the other five collectors wrote attributes, so a reader had to
know both to see all of them. The blind spot beside it is what puts the loss in the rate,
and `extract.input_refused` is the rule that names it.

`pinst` exists for one reason: plugin identity gates which parameter sub keys of a block
are live. Without it, swapping `FollowPath.plugin` on the reference stack reports a
phantom parameter drift for every sub key of the old plugin, none of them real.

## 5. Edges

`atlas.edges` is a map keyed by the edge handle. Every edge object carries `src`, `kind`,
`dst` and `key` as plain fields. Read those fields, do not parse the handle.

`model.EDGE_DOMAINS` is the closed table of allowed (source kind, destination kind) pairs.
An edge outside the table is a contract violation, not a warning: it means a collector
invented a relation.

`model.KEYED_EDGE_KINDS` lists the kinds that legitimately repeat between the same pair and
therefore carry a **mandatory** key over the producing site: `publishes`, `subscribes`,
`serves`, `calls`, `sets_param`, `remaps`, `broadcasts_tf`, `listens_tf`, `tf_parent`,
`launches`, `gates`, `manages_lifecycle`, `declares_param`, `reads_param`. Build the key
with `ids.edge_key(...)` over the launch entity, the profile, the source symbol, the urdf
model or the frame pair.

A second write of one `(kind, src, dst, key)` raises `ContractError` when it states anything
the edge does not already hold, and passes when it is an exact restatement. The literal rule,
that any duplicate raises, cannot ship and was never implemented: `join.py` re emits every
keyed edge it walks and point 6 of section 9 requires the join to be a fixpoint, so the
literal rule refuses a rerun of the tool on its own output. Two layers restating one site is
the same case, because the key is the site and the layer is not in it. What counts as new is
decided in one place, `model._second_site`: an attribute the edge does not already carry with
the same value, or, once the `(layer, profile)` slot exists, a value, a confidence or an
evidence row that slot does not already hold. A layer that has not written this edge before
opens its own slot and is a new view of one site rather than a second site. The case the rule
exists for still raises: two producing sites folded into one `publishes` edge, with one QoS
block standing for both.

Three reasons that rule exists, all from the reference stack: the same relative code name
is remapped to the same topic by two different launch files, `base_link` to `os_sensor` is
declared with three different origins in three urdf models, and `odom` to `base_footprint`
is broadcast from two different methods of one node.

Notes on specific kinds:

* `manages_lifecycle` accepts a `node`, a `launchent` or a `profile` as its source, because
  one launch file drives the transitions with `EmitEvent` plus `OnStateTransition` and there
  is no manager node at all. The mechanism goes in the edge values.
* `gates` carries the mechanism (`OnProcessExit`, `OnStateTransition`, `TimerAction`,
  `IfCondition`, `OpaqueFunction`, `filesystem_probe`), the expression and the period, so
  reachability is a path and not a boolean. Two nodes that both exist can still never
  coexist.
* `remaps` accepts a `file` source as well as a `launchent`, because a urdf gazebo plugin
  block `<ros><remapping>` is a real remap source.
* `tf_parent` must record which model or profile produced the geometry in its key and its
  values. A single valued origin publishes a wrong number as fact for two profiles out of
  three.

## 6. Facts, and how disagreement is represented

A `LayerFact` is what one layer, in one profile, said about one entity or one edge. Its
slot is `(layer, profile)`, where the profile is a profile id or `*` for a fact that holds
regardless of profile.

Keying by `(layer, profile)` and not by layer is not optional. On the reference stack, three
mutually exclusive `ExecuteProcess` variants all resolve to one node name with different
argv, and one sim launch file puts different parameters on the same nodes depending on one
argument. A single declared slot would drop two truths out of three while the profiles list
claims all three.

Rules:

* A slot is written by exactly one run. Collectors extending the same recorded slot use
  `CollectContext.run`, which the orchestrator supplies, so independent readers cite one
  extraction without reading each other. A second write may add keys, including nested keys,
  never change a stated value.
  Changing one raises `ContractError`.
* `confidence` is one of `certain`, `resolved`, `inferred`, `unknown`. A fact with
  `unknown` confidence is skipped by every comparison. Never default a value you failed to
  track.
* Volatile data goes under a `volatile` key inside `values`: rates, sample counts, message
  counts, durations, wall clocks. Volatile keys never enter a conflict, an effective value
  or a diff.
* `conflicts` is precomputed on the entity and on the edge: the sorted dotted field names
  that two layers disagree about **inside one profile**. Two profiles disagreeing is
  configuration, not drift. An empty array lets a reader stop after one line.
* `agreement` takes `agree`, `differ`, `single_layer` or `unknown`. It is a different
  vocabulary from a parameter `state`, and the two never share a field name.
* `effective` is precomputed per profile per dotted key, with the layer that won it.
  Precedence is fixed and stated in the file: `observed`, `recorded`, `declared`, `coded`.
  A running system outbids a written intent, and every loser stays readable.

Entity level presence, meaning seen in one layer and absent in another, is not a field. It
is `layers` on the entity plus `indexes.layer_only`. Legitimately observed only endpoints
exist on this stack (`/diagnostics` publishers behind `diagnostic_updater.Updater`, `/tf`
subscriptions behind `tf2_ros.TransformListener`, `PointCloud2` subscriptions behind
`message_filters.Subscriber`). They must be marked
`via_known_wrapper` from the table in `rosio/resolve/wrappers.py`, not reported.

## 7. Parameters

A parameter is an entity, not a value on a node, because it owns an ordered candidate chain
with per candidate provenance.

* `state` is mandatory and closed: `effective`, `inert_by_design`, `undeclared`,
  `declared_unread`, `owner_name_mismatch`, `unresolved_owner`, `other`.
* `effective` is empty unless the state is `effective`. A yaml block whose key matches no
  launched node (`lifecycle_manager:` when the node is `lifecycle_manager_navigation`,
  `map_saver:` when the node is `map_saver_server`) gets state `owner_name_mismatch` and no
  effective value, so it can never be read as live configuration.
* A parameter value's authority is the candidate chain, not the observed layer. This is the
  one documented exception to the precedence in section 6, and the reason is concrete: the
  observed value can come from a merged temp file that exists in no repository, so an agent
  told to read the effective value would be handed a number it cannot edit anywhere. The
  observed value stays visible as a fact.
* Every candidate carries `rank` (0 wins), `wins`, `mechanism`, `source`, `profile`, and
  when it lost, `shadow_kind` and `shadowed_by`. Ranks are unique per profile and at most
  one candidate wins per profile, both enforced.
* The chain is per profile and `AtlasBuilder.set_param` merges on that key: the rows of the
  profiles named in the call replace those profiles, the rows of every other profile stay,
  and the two invariants above are checked over the result. A run covers one profile while
  the parameter entity is shared by all of them, so assigning the whole list would leave a
  chain only for the profile written last. The entity keeps one `state`, the strongest of
  the profiles written (`effective`, `inert_by_design`, `owner_name_mismatch`,
  `unresolved_owner`, `other`), because a profile that could not follow a parameter must not
  delete the answer a profile that could follow it produced. The blind profile stays honest:
  none of its candidates wins, so it gets no effective entry, and its unresolved entry is
  mirrored on the entity.
* Precedence between two entry points of one profile is the order the profile declares them
  in, which is the order the launcher issues the commands in. It is never the sorted launch
  file path, and a pipeline that assembles a parameters file outside launch is bound to the
  one entry point its command names, so a profile is never handed values from a command it
  does not run.
* `mechanism` is closed and includes the mechanisms that are neither yaml nor launch
  parameters nor source code: `python_override` (a launcher rewriting the merged dict) and
  `param_set_runtime` (a launch file looping `ros2 param set` from a file some seconds after
  start). Without the last one, those parameters read as pure drift.
* Provenance always terminates at a source file or at the symbol that performed a merge
  (`applied_by`). It never points at a temp path. The merged params file the running stack
  was handed is a random name under `/tmp` that exists in no repository.
* Code side suffix only declarations (nav2 plugin parameters, and the costmap `Layer` case
  where the declare side auto prefixes and the read side spells `name_ + "." + key` by
  hand) are `pparam` entities bound to a `plugin`. The drift engine crosses the yaml plugin
  selector list to mint the node scoped `param` and links the two with `declares_param`.
* Overlay order is load bearing. Record the order of the `parameters=` list and the merge
  site, never a single hard precedence rule. One documented inversion exists on this stack:
  an inline `yaml_filename` override loses to the file value, which is why `RewrittenYaml`
  is used there instead.
* A wildcard yaml key expands one source line into N per node parameters. Record the key
  itself as the provenance locator so one deletable line is one finding, not N.
* A parameters file that a launch argument default names and the profile displaced is still a
  candidate, with `shadow_kind` `not_read_in_profile`. It never wins, and it never mints a
  parameter of its own: a key that only the displaced file spells is not configured in this
  profile at all, and minting it would invent configuration. The mark is scoped to the whole
  profile and not to one entry point: a line another entry point of the same profile reads is
  read in this profile, so that row drops and the real candidate on the line states it. Two
  arguments defaulting to one file are one candidate, because the line is deletable once. The
  launch resolver has to hand over the `DeclareLaunchArgument` rows of the files an entry point
  includes for this to reach the reference stack, because the two displaced rows acceptance 6
  wants are declared inside `collision_monitor.launch.py` and `_launch_child` keeps only the
  entry point's own.
* A python write whose value folded but whose guard did not is a candidate carrying the guard
  text, and remains a blind spot: the parameter stays `other` in that profile and nothing
  wins. Dropping the write hides the only line an engineer can edit, and letting it win names
  a value that is wrong on the other branch. A write whose value did not fold adds no
  candidate, because nothing about it is known.

## 8. Unresolved values

Nothing unresolved is ever promoted to an entity and nothing unresolved ever becomes a
placeholder id. It becomes one entry in `atlas.unresolved`, keyed by a digest over the site,
the expression and what was wanted, so two blind spots can neither merge nor multiply.

* Every entry is mirrored on the owning entity by `recompute`, so a reader of one entity
  slice cannot mistake a hollow extraction for a complete one.
* `counters.resolution_rate` is the CI gate surface. A drop fails the build before any
  finding does. On a real industrial stack the honest floor is a handful of Python and C++
  endpoint names built at runtime, plus gazebo loops over SDF elements and behavior tree
  blackboard ports.
* `counters.passes` sits beside it and answers a question the rate cannot: whether the atlas
  is whole. The rate is resolved over resolved plus unresolved per want, so a pass that died
  contributes neither and its want either keeps a perfect rate over a smaller denominator or
  leaves the counter entirely. The more a collector loses, the healthier the ratio reads.
  The counter states `attempted`, `completed`, `refused`, `not_attempted`,
  `refused_collectors` and one boolean `complete`, summed from the pass rows the runs carry,
  so a reader and a gate both ask once and neither walks the run entities. `complete` false
  is an `extract.pass_incomplete` finding per stopped pass, which is `error`, so an atlas
  that dropped a pass never exits zero under `--fail-on error`. A pass row belongs to the
  run that wrote it, so an atlas a later verb extends still reports the pass that died in
  the earlier one, which is the truth: that data is still not in the file.
* Report ignorance rather than guessing. Truly unresolvable on this bench: a gazebo plugin
  subscribing inside a loop over SDF `<topic>` elements, a behavior tree node reading its
  topic from a blackboard port with no code side default, and parameter defaults that are
  runtime object attributes.

## 9. Determinism

1. One encoder. `model.dumps` is the only place that calls `json.dumps`: sorted keys,
   indent 2, ensure_ascii, LF, trailing newline, `allow_nan=False`. No module may call
   `json.dumps` directly.
2. One value encoder. `model.canonical_value` preserves types: an int stays an int, a float
   stays a float. Never turn 20 into 20.0, that alone is a phantom drift.
3. `atlas.to_bytes()` is everything. `atlas.canonical_bytes()` is everything that is
   compared: it drops the `generated` block and every `volatile` and `line_hint` key at
   every depth. Two runs over unchanged inputs produce identical canonical bytes, and CI
   asserts it. The `generated` block sits outside the comparison and inside the file, and
   the file is committed, so it states no absolute path either: a root is stated by its
   kind, its label, the hash of its content and its place in the checkout, which is what a
   second machine can match a root by. It held the absolute path of the workspace once, and
   every clone therefore rewrote `atlas.json` on its first rebuild, which left the work tree
   dirty and stopped `git merge` and `git pull` before they started. One workspace built at
   two absolute paths is one file, byte for byte, and a test asserts that too. Where the
   workspace sits on one machine stays in that machine's configuration and command line,
   which is where a person debugging that machine already looks.
4. `line_hint` is advisory, for humans. It never appears in an id and never enters a
   semantic diff. Provenance is inline `{file, locator, line_hint, root}`, never an index
   into a shared table: a slice of the atlas has to be self sufficient, and an index shifts
   when a file gains a line.
5. Every collection is sorted by a declared key. Facts sort by layer order then profile,
   evidence by (file, locator, line hint, root), candidates by (profile, rank, file,
   locator), findings by (severity, rule, subject, discriminator), everything else by id.
   One exception, and it is stated here rather than left for a reader to infer from the
   code: a sequence whose order is itself a fact keeps that order, and the declared key is
   the position it was read at. Three sequences are in it today, all written by
   `rosio/collect/proc.py` from one command line: `params_files`, `remaps` and
   `assignments`. Later wins for a parameters file, the **last** assignment wins for a
   repeated `-p`, and the **first** rule wins for a repeated remap, so sorting any of the
   three would state a precedence that is not the one the process runs under.

   Those last two are different rules and the remap one is the opposite of what a reader
   assumes, so it is measured rather than asserted. One rclpy process per case, humble,
   `ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1`, each process printing its own name and its
   own publisher's resolved topic: `-r __node:=first -r __node:=second` runs as `first`, the
   same pair split across two `--ros-args` groups also runs as `first`, `-r __ns:=/a -r
   __ns:=/b` lands in `/a`, `-r chatter:=/first -r chatter:=/second` publishes on `/first`,
   and `-p k:=1 -p k:=2` reads 2. rcl keeps the first rule that names a thing and the last
   assignment that gives a parameter a value. `rosio/resolve/ros_args.py` therefore keeps
   the first `__node` and the first `__ns` rule of an argument list, and the last `-p`.

   The launch system inverts what the same reader would assume next.
   `launch_ros/actions/node.py` builds `[executable] + arguments + ['--ros-args'] +
   ros_arguments + ['--ros-args', '-r', <the name= rule>]`, so the rule launch states for
   itself is the last one on the command line and therefore the losing one. Measured through
   a real `ros2 launch`: `Node(name="stated", arguments=["--ros-args", "-r",
   "__node:=from_argv"])` starts a node called `from_argv`. A name a launch file states does
   not outrank a name the argument list carries, `resolve/ros_args._apply` is written on that
   measurement, and `ids.NAME_SOURCE_ORDER` ranks how well rosio knows a name rather than
   claiming to be the runtime rule.

   Anything not named here is sorted. A sequence added to this
   exception is named here in the same commit, and its order is a fact of the source, never
   an iteration order.
6. `AtlasBuilder.recompute()` is a pure function of entities, edges, findings and
   unresolved entries, and a fixpoint. Running it twice changes nothing.
7. Layer collectors are pure and never read another layer's output. Reordering two
   collectors that share nothing must produce the same atlas. The one order that is not free
   is the declared bundle of section 2.1, where every producer runs before every consumer,
   and a consumer that runs first refuses rather than reporting a workspace it never read.
8. `run` records carry the machine dependent inputs and, in `passes`, one row per collector
   saying whether that pass finished: the input digest, the roots with a
   hash each, the injected environment, and the filesystem predicates that decided a branch
   (whether a `.posegraph` file exists, which is what selects the slam executable on this
   stack). Two boxes that disagree only about a file on disk must show an input difference,
   not drift.

   `launch_sandbox:bwrap` states whether the launch walk used bubblewrap. It is recorded
   outside the input digest and never enters an id, so tool availability changes the run's
   stated inputs rather than its identity.
9. Every evidence row carries its `root` (`src`, `install`, `runtime`, `bag`). The install
   tree here holds copies, not symlinks, and can be older than src, so a mixed root on one
   entity raises one `build.stale` finding instead of dozens of phantom content drifts.
10. A root hash covers every name under the root and the content of every file worth
    reading, and the absolute prefix this machine holds the workspace at is replaced by the
    root label before those bytes are hashed. It is the one bounded exception to point 8,
    and it is what makes the atlas portable for a checkout that was built rather than only
    for one that was not. The build tool writes the prefix of the machine it ran on into its
    own setup files, those files sit inside a root, and two clones of identical sources
    therefore hashed their install root differently and produced two atlases, which is
    nearly every ROS developer. Measured on one workspace built by colcon at several absolute
    paths: the install tree files that differ are all generated setup and dsv files, and the
    whole difference is the prefix.

    The hypothesis this replaces was to hash only what rosio read from a root, on the
    ground that a setup file nobody reads cannot change a fact. It was measured and it is
    false: `collect/entrypoints.py` scrapes every `.sh`, `.bash`, `.zsh` and `.py` file in
    every root for a `ros2 launch` or `ros2 run` command, so the setup files are read, and a
    launcher script somebody wrote is read by the same pass and must stay in the hash. Read
    is the wrong predicate. Mounted somewhere else is the right one, it is decidable without
    running a collector, and it is the same translation `Workspace.stated` and
    `Workspace.token` already apply to every path that reaches compared data.

    The gate that separates two builds keeps working, and this is the property to check
    before touching `Workspace.content_marks`: only the substring naming the mount point is
    normalised, so a setup file that gained a package, an overlay prefix outside the
    workspace, a hand written line, an added or removed file, and any edit to a file a
    collector reads all still move the hash. What is left is stated rather than claimed
    closed: a file whose bytes are not hashed contributes its size, and a compiled artefact
    whose size changes with the length of an embedded path would still separate two clones.
    On the measured bench it does not, because the sizes are equal and nothing under
    `__pycache__` is walked at all.

## 9a. Schema evolution

`schema_version` is `major.minor`. A minor bump is additive only: a new node kind, a new
edge kind, a new layer key, a new fact key, a new attribute. A major bump is anything that
removes a key, changes the meaning of a key, changes an id form, or changes the effective
value precedence.

* Id form changes are major bumps only. A minor bump must never invalidate a stored finding
  id, a waiver or a consumer cache.
* `Atlas.from_json` accepts a newer minor and refuses a newer major. Unknown top level keys,
  unknown entity keys and unknown edge keys are preserved and re emitted, never dropped and
  never an error.
* Unknown enum members are tolerated on load and passed through as strings. A consumer must
  never switch exhaustively on a closed vocabulary without a default branch.
* `schema.validate` is stricter than the loader on purpose: it reports unknown members and
  unknown keys as violations, so a mixed version pipeline is visible. Loading does not call
  it.
* `capabilities` is the feature probe. Test it rather than guessing from an absent key.
  Adding the recorded layer appends `layer.recorded`, `edge.timing` and `bagtopic.stats`.
  The pass record still has no mark of its own, which is owed and lives in `rosio/extract.py`.

The current version is **`1.14`**, and every addition it holds over `1.0` is additive, so no
stored finding id moved, no waiver stopped matching and no consumer cache was invalidated.
What was added, in the order it arrived:

* the `passes` list on a run record and the `counters.passes` object derived from it, which
  is a new attribute and a new counter key;
* the rule ids `extract.pass_incomplete` and `extract.input_refused`, which are two new
  members of the finding vocabulary;
* the `refused_because` and `refused_detail` attributes a refused entity carries, and inside
  the first of them one renamed value, `manifest_unreadable` to `plugin_manifest_unreadable`.
  A renamed **value** of an open attribute is not the same as a renamed **rule id**: it takes
  no `ALIASES` entry because no waiver keys on it, and a consumer that switched on the old
  string sees an unknown member, which the bullet above says it must tolerate. It is
  announced here because nothing else announced it.
* recorded `stamps` checks on each bagtopic with header stamps, volatile stamp rates,
  `stamp_verdict` on each bag, and the seven `time.*` finding ids, added in `1.2`.
  Stamp measurements round
  seconds to nine decimal places and rates to six.

* observed node `rmw` and `zenoh_environment` presence, observed run `zenoh_router`,
  optional observed run `rmw_default`, and the `rmw.mixed` and
  `zenoh.router_unreachable` finding ids, added in `1.3`. An unreadable process
  environment states `unknown`; an absent RMW_IMPLEMENTATION states `default`.
  Zenoh configuration variables carry presence only, never their values.

* the `qos.incompatible` finding id, added in `1.4`.

* observed topic `volatile.stamps` checks and run `volatile.stamp_verdict`, added in `1.5`.

* the `param.sim_time_mixed` finding id, added in `1.6`.

* the `launch.file_not_installed` finding id and the `file_not_installed` value of
  `refused_because`, added in `1.7`.

* launch entity `file_reads`, with file ids and reader roles for parameters files, robot
  models and node arguments, added in `1.8`. Commands are never run by the launch walk.
  Their parameter values remain unknown candidates with `command_substitution` blind spots.

* construction records gained optional `definition_file` in `1.9`. It names a unique class's
  included header inside the same package, so the join can bind its methods to the caller.

* the optional `class_templates` record block was added in `1.10`. It counts definitions
  without collapsing conditional definitions in one file, and records specializations.
  A construction or alias target binds by template name only when exactly one definition
  and no specialization exists in the package. Explicit instantiations do not count as
  definitions. Missing or ambiguous definitions stay unresolved with a reason.

* an unresolved entry gained optional `construction` in `1.11`, the construction context of
  a source blind spot. It enters the entry's id only when stated, and one expression blind in
  two contexts is two entries rather than one that hides the second. An entry without a
  context kept its id. An entry with one did not: before `1.11` the context rode inside
  `role` (`cpp_source:<digest>`, `publisher:<digest>`) and the id was a digest over owner,
  expression and want alone, so the two contexts shared one id. Every such id moved, and
  `role` is now the bare word and no longer carries the construction digest, which changes
  the meaning of an existing value. No release had shipped, so no stored id or reader of the
  old `role` existed. The blind spot of a parameters file a launch file wrote under a temp
  directory moved in the same version: its expression named the item's position in the
  node's parameters list, which a `SetParameter` in any enclosing group shifted, and it now
  names the item as the launch file spells it.

* a launch exception row gained optional `kept` in `1.12`, the action the walk still recorded
  without the value that failed to expand, such as `Node(rviz2)` whose `-d`
  names an unbuilt package. A `launch.raised` finding over such a row names that node and
  stops claiming the walk stopped or that nodes are missing.

* endpoint and tf broadcast records gained optional `guard_parameters` in `1.13`, the
  parameter terms of a guard with the bool each needs. The join no longer binds a record in
  a profile whose candidate chain reads a term the other way, so that profile loses the
  edge and, when nothing else names it, the topic or frame. No id form moved.

* a frame gained the attribute `urdf_geometry` in `1.14`, keyed by the model file like
  `tf_parent`, holding the `visuals` and `collisions` rows of that link as the model writes
  them: the shape and its stated numbers, the `<origin>` of the block when one is written, the
  material name and colour, and for a mesh the `package://` name as written plus the file id
  when the workspace holds the file. An absent value stays absent and a number that does not
  read is named in `unreadable`. The evidence row of a declared link now carries the line of
  its `<link>` element, which is advisory and outside compared data.

One key went in `1.6` without a major bump, and it is named here rather than left for a reader
to find. The declared `node` fact carried `conditions`, the conditions of the launch site that
started the node. A node started from two sites with different conditions (nav2's
`localization_launch.py` loads `map_server` under `map == ''` and under `map != ''`) made two
writes to one `(layer, profile)` slot that disagree, which section 6 refuses, so the key could
only ever hold the case where it said nothing. The conditions stay on each `launchent` fact and
on its `gates` edges, where one site is one entity. No release had shipped when it went, so no
reader of the key existed.

One finding id form moved in `1.10` without a major bump, for the same reason. A
`remap.unresolved` finding was about the launch entity of one entry point, so one remap line
read through eleven entry points was eleven findings and a new profile moved every id. Its
subject is now the `file` that holds the remap line, with the same discriminator, so one
deletable line is one finding. No release had shipped, so no stored waiver keyed on the old
form.

The same holds for `launch.raised` in `1.12`. A row carrying `kept` adds it to the
discriminator, so a failure inside one node is one finding per node rather than one per
message, which had folded two nodes failing on one missing package into a single finding
that named neither. A row without `kept` keeps the id it had.

A reader pinned to `1.0` still loads a `1.14` file: `Atlas.from_json` compares the major
alone, and the keys and the enum members that build never heard of are preserved and re
emitted rather than dropped or refused. `rosio/model.py` states the same list beside
`SCHEMA_MINOR`, and a test loads a file carrying the additions with the version pinned
back to `1.0`.

## 10. Findings

A finding is a CI failure line and a chat sentence at once. Fields:

| field | meaning |
| --- | --- |
| `rule` | closed vocabulary, kebab and dots, for example `param.shadowed` |
| `severity` | `error`, `warn`, `info` |
| `about` | `{"entity": <id>}` or `{"edge": {src, kind, dst, key}}`, never an edge triple inside an id |
| `title` | one short line |
| `detail` | why it is true |
| `fix` | what to change |
| `say` | one sentence a small model may quote verbatim |
| `where` | inline provenance rows |
| `fingerprint` | sha256 over rule, subject, discriminator and normalised detail |
| `discriminator` | derived from sorted content, never from iteration order |

`say` must name the concrete values and the file to edit, and must never contain a
synthetic id, a finding id, an evidence index or an edge triple. The builder refuses a
`say` or a `title` containing `:~`.

Committed rule ids for M3. `rosio/drift/rules.py` must match this list exactly:

```
param.shadowed              param.restated_identical      param.set_undeclared
param.declared_unread       param.orphan_block            param.type_changed
param.owner_name_mismatch   plugin.inert_by_design        plugin.unused
plugin.unresolved           node.declared_not_observed    node.observed_only
node.ambiguous_identity     node.identity_regressed       node.unnamed
topic.no_subscriber         topic.no_publisher            qos.differ
type.differ                 rate.below_declared           frame.no_publisher
frame.orphan_root           frame.not_in_urdf             frame.parent_differs
remap.unresolved            lifecycle.unmanaged           build.stale
launch.unloadable           launch.raised                 endpoint.observed_only
resolution.below_budget     extract.pass_incomplete       extract.input_refused
time.backward               time.epoch_offset             time.topic_skew
time.frozen                 time.zero                     time.gap
time.drift
rmw.mixed                   zenoh.router_unreachable      qos.incompatible
param.sim_time_mixed        launch.file_not_installed
```

`launch.file_not_installed` is `error`. It fires once per file a launch file reads
from a package share directory that the install tree does not hold while the package source
holds the same package relative path, which is a build file missing its install line. The
parameter collector records it as the refusal `file_not_installed` with the launch site as
evidence, and `extract.input_refused` leaves that reason to this rule. The fix names the
`install(DIRECTORY ...)` line of `CMakeLists.txt` or the `data_files` entry of `setup.py`, by
the package's build type. A path absent from the source as well is a wrong path and stays a
refusal, and a package with no install tree is not built, so neither fires it.

`param.sim_time_mixed` is `warn`. It fires once per profile whose nodes state both values of
`use_sim_time` in their winning candidates, names the minority side and the line that sets
it, and never assumes a value for a node whose `use_sim_time` no file sets.

`qos.incompatible` is `error`. It compares a subscription with every publisher of its topic in
one layer and one profile, from the policies each side states or the preset it names
(`rosio/resolve/qos_presets.py`), and fires when a subscription requests reliable from a best
effort publisher or transient local from a volatile one. `qos.differ` is a different question,
two layers disagreeing about one endpoint.

`extract.pass_incomplete` is `error`, and it is one of the two findings that report the
extraction itself rather than the system it describes. It fires once per collector pass that
stopped with data still to read, naming the collector, the recorded reason and what the atlas
therefore does not hold. A pass whose inputs were absent never fires it: a collector with
nothing to read is complete, and a rule that reports every small repository is ignored
within a week.

`extract.input_refused` is the other one, and it is `warn`. It fires once per entity
carrying `attrs.refused_because`, which is one input a collector could not read inside a pass
that finished, and it names the file or the recording, the recorded reason and what the atlas
therefore does not hold. It cannot fire for a file that was read, because only a collector
that refused an input writes that attribute. It is graded under `extract.pass_incomplete` on
purpose: the pass did reach the end of its inputs, the blind spot every refusal leaves
already carries the loss into `counters.resolution_rate` and therefore into
`resolution.below_budget`, and a third party package that exports a plugin manifest it does
not ship is a true refusal which must not turn a user's error gate red on its own. On the
installed humble tree it fires on the manifest `nav2_theta_star_planner/package.xml` exports
and the installed share directory does not hold, recorded reason `plugin_manifest_missing`,
and `drift --fail-on error` still exits 0. On the public `grid_map` sources it fires with
reason `plugin_manifest_unreadable` on `grid_map_cv/filter_plugins.xml` and
`grid_map_filters/filter_plugins.xml`, both of which spell an unescaped `<` inside an
attribute and are therefore not parsable xml.

Adding a rule id is a minor schema bump, because it is additive: no stored finding id moves
and no waiver stops matching. A rename is not additive and takes an `ALIASES` entry in the
same commit, never a silent replacement. Removing a rule id is a major bump.

Findings are sorted by severity first, so the head of the array is the failure list top to
bottom. `counters.findings` and `counters.resolution_rate` sit near the top of the file so
both consumers get a summary without reading the body.

### 10.1 Waivers

Waivers key on the finding `id` or on the `fingerprint`. Rule ids are a committed vocabulary
with a deprecation alias table, so a rename never silently un waives a finding: `engine.waive`
follows `rules.ALIASES` backwards and matches a finding against the id **and** the fingerprint
it would have had under every retired name that now points at its rule. The alias table only
repairs the id by itself, because the fingerprint is a digest over the rule id, so the
backwards match is the half that makes the sentence true for both key kinds.

A team that cannot silence one known finding turns the whole gate off, so there is an entry
point and it is `rosio drift`, never `extract`.

* **The file.** yaml, `rosio-waivers.yaml` by convention, one top level `waivers` list of
  mappings with exactly two keys, `key` and `reason`. The reason is mandatory. A waiver with
  no reason is how a gate rots: a year later nobody can tell whether the finding was
  understood or only silenced, so nobody deletes it. A key that is neither a finding id nor a
  `sha256:` fingerprint is a usage error, which is what catches a rule id typed into the file
  by somebody who meant to silence a whole rule and would otherwise have silenced nothing.
  One key is stated once. An exact restatement passes, the way a keyed edge's does in
  section 5, and two files accepting one finding for two reasons is a usage error rather
  than a file order.
* **The flag.** `rosio drift --waivers FILE`, repeatable. The command line wins whole over
  the configuration rather than adding to it.
* **The configuration key.** `waivers` in `rosio.yaml`, a string or a list, resolved
  against the directory of that file. `rosio drift --config FILE` names the file, and
  without it `rosio.yaml` here or one directory up is read. `config.waiver_files` reads
  that one key without discovering a workspace, because a gate verb has no workspace.
* **What a waiver does and does not do.** It takes the finding out of what `--fail-on`
  grades. It does not take it out of the report: `rosio drift` prints every finding it
  always printed, and adds the count that matched, each waived finding id with its reason,
  and every key that matched nothing. A finding nobody can see is a finding whose waiver
  nobody ever deletes, and a stale key is the thing that quietly widens a waiver file.
* **Never an extraction input.** `atlas.json` carries every finding whatever a repository has
  accepted, so a machine with a waiver file and a machine without one write the same bytes,
  and `Config.fingerprint` leaves the key out for that reason. Grading is a decision about a
  file, not a fact about the system.

## 11. Where the judged tradeoffs landed, and why

* **Flat id keyed maps, not arrays.** `entities` and `edges` are maps, so one lookup answers
  one question with no client library. Arrays would force a scan.
* **Inline provenance, no shared evidence table.** A shared table with integer references
  is about a third smaller and completely unusable: a raw slice loses its provenance, and
  the indices churn when a file gains a line.
* **Spelled out field names.** `effective`, `agreement`, `conflicts`, `evidence`, not `e`,
  `s`, `v`, `ev`. A small model reading a raw slice with no schema in context must not have
  to guess. The CLI can compress on the way out.
* **Precomputed answers on the entity.** About thirty percent size overhead, in exchange
  for answering the highest value question in the product from one read.
* **Adjacency index.** `indexes.out` and `indexes.in` hold plain edge handle lists, so fan
  in and fan out never scan the edges table.
* **One enum space per field name.** Layer disagreement is `agreement`. Parameter lifecycle
  is `state`. They are disjoint closed vocabularies.
* **No mini formats inside JSON strings.** Adjacency entries are edge handles, not encoded
  triples.
* **Readable edge handles.** `edge:<kind>|<src>|<dst>|<key>` keeps the common case
  readable, and an embedded compound id pays one `%7C` per separator plus one `%7E` for a
  synthetic marker. Consumers read the `src`, `kind` and `dst` fields instead of parsing.
* **The atlas file is not the agent surface.** A full atlas on a real stack is megabytes no
  matter how it is shaped. The 4k surface is `rosio explain`, `rosio param`,
  `rosio query`, `rosio diff`, `rosio unresolved`, `rosio drift --min-severity`,
  each returning one entity or a short list.
* **The candidate chain is stated once.** Parameter material used to be stated twice, and
  removing the restatement was the largest size cut the atlas has had. What went, and why each
  carried nothing: the `sets_param` edge, whose source file, locator, mechanism and profile are
  the four fields of the candidate row it points at; its half of the adjacency index; the
  parameter fact's `evidence`, which was the sorted candidate sources of the same profile in
  every fact; the fact's `name`, always equal to the entity label; and the fact's `candidates`
  count, always equal to the length of that profile's chain. Nothing was shortened and no field
  was renamed, because a small model reading a raw slice with no schema in context must not have
  to guess. What remains is composition and not restatement: the parameter entities with their
  chains, per profile facts and `effective` blocks, the `has_param` edges and the `indexes`
  block.
* **The size ceiling is stated over an input a stranger holds.** Two gates, both in
  `tests/test_extract.py`, both recomputed by the suite on any clone: `FIXTURE_CEILING_BYTES`
  over the committed fixture workspace, which is the whole atlas in bytes, and
  `FIXTURE_BYTES_PER_ENTITY` over the same input, which is the one that catches an encoder
  writing more per row, because there the input cannot move under the number. Absolute size on
  a real code base is measured privately under `bench/` and never published, for the reason
  section 11a gives. A ceiling read off the workspace this tool was developed against is not a
  gate either: nobody else has that tree, so nobody else can reproduce, check or raise the
  number, and section 1 rule 9 already says nothing of that workspace ships. One such ceiling
  went stale twice because the workspace under it gained packages rather than because the
  encoder changed, which is the failure mode a number over a moving private input has and a
  ratio over a committed one does not. What the gate is for stays: raise it only with a
  measurement, never to make a red gate green, and a measurement states the input, the commit
  and which of the two moved, the encoder or the stack.
* **One named restatement is still in, and on purpose.** A `has_param` edge repeats the owner
  that the parameter id already spells out. Both halves of it have to go at once: the declared
  half is written by `collect/params.py` and the coded half by `join.py`, and removing one
  leaves a relation that covers the parameters the coded layer reached and none of the others,
  which is worse than either. Whoever owns both in one milestone removes it, and a node's
  parameters are then the prefix scan `param:<owner body>|` over the sorted entity map.
* **The verb set is committed here, and it is not closed.** `extract`, `probe`, `drift`,
  `explain`, `param`, `query`, `path`, `diff`, `unresolved`, `render`, `install`,
  `uninstall`, `serve`, `hook`, `version`, `stamps`, `init`, `bundle`, `migrate`, `graph`. Twenty. It said closed and final at
  fourteen for four milestones and the fifteenth shipped anyway, justified in a docstring of `cli.py`
  where no reader of this file would meet it, so the sentence was not a rule, it was a
  prediction that turned out false and hid a decision. The rule that replaces it is one this
  tree can keep: a verb exists when it is in this list, adding one is a line here in the same
  commit as the code with the reason beside it, and renaming or removing one is a break for
  every caller and every script and needs a deprecation period rather than a commit.
  `migrate` lists and rewrites what a move between two ROS 2 releases asks of the workspace's own
  packages, then verifies them with a build in the target container. It reads source files and
  three data tables, never an atlas, so it is a verb of its own.
  `graph` gives a terminal, a CI job or an agent without MCP the answer of the MCP `graph`
  tool, from the same selection, and draws it as a Graphviz or Mermaid diagram. It is not
  `query`, which matches words to entities, and not `render`, which writes files.
  `bundle` carries an atlas, its reports and cited files so another person can open the problem.
  `stamps` checks header stamps in one recording without a workspace or a ROS installation.
  `init` writes the configuration extraction would fall back to anyway as a `rosio.yaml` a
  person edits, because a profile is knowledge about how a robot is started that no file in
  the workspace states.
  `hook` registers the git hooks and the `atlas.json` merge driver in a repository. It is not
  folded into `install`, which registers this tool with an assistant: the two share no flag,
  `--scope project|user` means nothing to a git hook and `--python` means nothing to an
  assistant, and one `--list` answering about both would answer neither well.
  `explain` is the verb for one finding in full, and `diff` compares two
  atlases. `query` and `path` keep the names every user of this category already knows.
  `drift` takes `--fail-on {error,warn,info,never}` for the CI gate and
  `--min-severity {error,warn,info}` for output filtering, because the gate severity and
  the print threshold are two decisions and one flag cannot carry both. It takes
  `--waivers FILE` for the third decision, which findings this repository has already
  accepted, and section 10.1 says why that one never reaches an extraction.

## 11a. Published numbers

A tracked document (this file, `README.md`, the site, every page under `docs/`) carries checked
facts only. A checked fact is recomputed from the package or from a file in this repository,
and `tests/test_cli_docs.py` reads it out of the document and compares it with its source: the
verb set, the rule vocabulary and its count, the tool names the server exposes, the schema
version and the size of the file an install writes. Editing one of them in a document fails
the suite.

A measurement never appears in a tracked document or a commit message as an argument for the
tool: no finding count per stack, no atlas byte count or digest, no resolution rate, no timing,
no "N of N". A number published by hand goes stale the moment the tree under it moves, and one
taken over a tree outside this repository goes stale where no test can see it. Measurements
live untracked under `bench/` and are for the people developing rosio.

A capability claim in a document is one a reader can verify by running the tool on a public
input, a committed example such as `examples/rover_ws` or a stack that ships with ROS 2 such as
`nav2_bringup`, with the command beside the claim.

## 12. Module ownership map

One owner per module per milestone. Do not edit a module you do not own in your milestone.

| module | milestone | purpose |
| --- | --- | --- |
| `rosio/__init__.py` | M0 | version, layer names |
| `rosio/errors.py` | M0 | exceptions and exit codes |
| `rosio/ids.py` | M0 | identity: build, parse, normalise |
| `rosio/model.py` | M0 | atlas dataclasses, builder, serialisation |
| `rosio/schema.py` | M0 | JSON Schema and the dependency free validator |
| `rosio/cli.py` | M0, extended per milestone | argparse entry point |
| `rosio/scaffold.py` | M8 | `rosio init`, the fallback configuration written out to edit |
| `rosio/bundle.py` | M8 | portable archives of an atlas, reports and cited files |
| `rosio/migrate/__init__.py` | M9 | `rosio migrate`: the source packages of a workspace, plan and apply |
| `rosio/migrate/verify.py` | M9 | copied source builds in the target container, diagnostics and plan matching |
| `rosio/migrate/distros.py` | M9 | what each release ships, read from the measured `data/distros.json` |
| `rosio/migrate/rules.py` | M9 | the hand written rules of `data/rules.yaml`, checked at load |
| `rosio/migrate/scan.py` | M9 | the matchers: manifests, CMake calls, includes, C++ identifiers, python imports, parameter yaml |
| `rosio/migrate/cmake.py` | M9 | CMake command invocations and their argument spans |
| `rosio/migrate/report.py` | M9 | the plan as text and as data |
| `rosio/__main__.py` | M0 | `python -m rosio`, the same entry point as the command |
| `rosio/extract.py` | M1 | the extract command: collector order, one run per layer per profile, validate, write |
| `rosio/config.py` | M1 | run configuration, workspace roots, profile selection, the waiver file |
| `rosio/workspace.py` | M1 | package discovery, ament index, install tree, root hashes |
| `rosio/collect/__init__.py` | M1 | collector registry, the collect context, and the declared bundle handoff of section 2.1 |
| `rosio/resolve/__init__.py` | M1 | the resolve package surface, no atlas id anywhere under it |
| `rosio/collect/launch_declared.py` | M1 | launch files, executed in a sandbox |
| `rosio/collect/params.py` | M1 | yaml parameter profiles and the candidate chain |
| `rosio/collect/urdf.py` | M1 | urdf and xacro, frames, gazebo plugin remaps |
| `rosio/collect/interfaces.py` | M1 | msg, srv and action definitions |
| `rosio/collect/plugins.py` | M1 | pluginlib xml, package.xml exports, CMake exports |
| `rosio/collect/entrypoints.py` | M1 | non launch profile composers, `ros2 launch` and `ros2 run` argv |
| `rosio/resolve/substitutions.py` | M1 | substitution evaluation |
| `rosio/resolve/launch_exec.py` | M1 | sandboxed `generate_launch_description`, OpaqueFunction, event handlers |
| `rosio/resolve/rewritten_yaml.py` | M1 | shim for `nav2_common` RewrittenYaml and ParameterFile |
| `rosio/resolve/merge.py` | M1 | the one deep merge rule, with the merge site recorded |
| `rosio/resolve/ros_args.py` | M1 | argv parser for `--ros-args`, `-p`, `-r`, `--params-file`, `__node`, `__ns` |
| `rosio/resolve/known_names.py` | M1 | third party node name table |
| `rosio/resolve/records.py` | M1 | the intermediate records of the declared layer, none of them holding an atlas id |
| `rosio/resolve/yaml_lines.py` | M1 | yaml loading that keeps a line hint per key path |
| `rosio/resolve/_launch_child.py` | M1 | the sandbox child, one launch description per request over stdin |
| `rosio/resolve/py_pipeline.py` | M1 | parameter documents a launcher assembles in python before launch sees one |
| `rosio/resolve/identity.py` | M1 | role content, role digest and the one node id rule every collector asks |
| `rosio/collect/py_code.py` | M2 | rclpy sources: endpoints, parameters, TF, node names |
| `rosio/collect/cpp_code.py` | M2 | C++ sources, tree-sitter optional extra |
| `rosio/resolve/wrappers.py` | M2 | known wrapper table (diagnostic_updater, message_filters, tf2_ros) |
| `rosio/join.py` | M2 | the single join phase, supersession, idempotent |
| `rosio/drift/__init__.py` | M3 | the drift package surface, re exporting the rule registry |
| `rosio/drift/rules.py` | M3 | the closed rule vocabulary, the alias table, the severity per rule and the registry |
| `rosio/drift/engine.py` | M3 | run order, rule isolation, deduplication, waivers, filtering and the counters |
| `rosio/drift/{build,frames,nodes,params,plugins,time,topics}.py` | M3 | one module per rule family |
| `rosio/drift/middleware.py` | M5 | mixed middleware and missing Zenoh routers per observed profile |
| `rosio/query.py` | M3 | CLI slices: explain, param, query, path, diff, unresolved, drift |
| `rosio/render/__init__.py` | M4 | the render package surface, pure functions of a loaded atlas |
| `rosio/render/report.py` | M4 | ATLAS_REPORT.md |
| `rosio/render/drift_md.py` | M4 | DRIFT.md |
| `rosio/render/html.py` | M4 | single file atlas.html, nothing fetched at run time |
| `rosio/collect/probe.py` | M5 | the rclpy probe, observed layer, one isolated child per middleware |
| `rosio/collect/_probe_child.py` | M5 | the child that imports rclpy, one request over stdin, so a stuck discovery costs one probe |
| `rosio/collect/proc.py` | M5 | the process table half of the observed layer: argv, params files, remaps, middleware and router presence |
| `rosio/collect/bag.py` | M5 | rosbag2 metadata, recorded layer |
| `rosio/collect/stamps.py` | M5 | recorded header stamp checks and clock verdicts |
| `rosio/stamps/__init__.py` | M5 | the public stamp check helpers and thresholds |
| `rosio/stamps/__main__.py` | M5 | the standalone header stamp command entry point |
| `rosio/stamps/checks.py` | M5 | the registry of stream and clock checks |
| `rosio/stamps/cli.py` | M5 | the stamps command arguments and rendered verdict |
| `rosio/stamps/clock.py` | M5 | epoch offsets, clock drift and skew between topics |
| `rosio/stamps/fallback.py` | M5 | tolerant readers for recordings with damaged definitions |
| `rosio/stamps/model.py` | M5 | stamp series and individual check result records |
| `rosio/stamps/mutate.py` | M5 | controlled timestamp mutations for recorded fixtures |
| `rosio/stamps/read.py` | M5 | header stamp series and reader degradation notices |
| `rosio/stamps/stream.py` | M5 | backward steps, frozen and zero stamps, gaps and rates |
| `rosio/stamps/verdict.py` | M5 | clock diagnosis from the combined check results |
| `rosio/agent/__init__.py` | M6 | the agent package surface, which resolves `serve` lazily so an install never pulls in `mcp` |
| `rosio/agent/platforms.py` | M6 | the table of assistants, one row per file, with the verified flag |
| `rosio/agent/install.py` | M6 | the install and uninstall writers, idempotent, no silent overwrite |
| `rosio/agent/templates/` | M6 | the four files the installers copy into a user's repository |
| `rosio/serve/__init__.py` | M6 | the serve package surface |
| `rosio/serve/tools.py` | M6 | the eleven tools as plain functions, ten over a loaded atlas and `migrate` over the sources, no mcp involved |
| `rosio/serve/server.py` | M6 | the mcp server, both transports, the key check |
| `rosio/serve/__main__.py` | M6 | `python -m rosio.serve`, the same options as `rosio serve` |
| `rosio/history.py` | M7 | the git hooks, the rebuild they run and the two revision mode of `diff` |
| `rosio/mergetool.py` | M7 | the `atlas.json` merge driver, union with every disagreement stated |
| `pyproject.toml` | M0, extended per milestone | build, package data, and the version range of every optional extra |

The agent surface is two packages, not one module. `rosio/agent` writes files and imports
nothing optional, `rosio/serve` answers questions and is the only place the `mcp` extra is
reached. Nothing in `rosio/agent` may import `rosio/serve` at module import time, or an
install fails on a machine that never wanted the server.

An optional extra's range lives in exactly two places, `pyproject.toml` and the module that
raises when the package is missing, and a test holds them equal. A message telling a user to
install something they already have, in a version the code refuses, costs more than saying
nothing. Every other reader imports the module's name rather than spelling the range again,
the cli included, and the release bounds a version check compares against are derived from
that one string and never written beside it. Both ranges of the mcp extra are covered:
`MCP_EXTRA` and `UVICORN_EXTRA` in `rosio/serve/server.py`, each held equal to the
packaging file by a test. An untested pin is a pin that drifts.

`verified` in the platform table means one thing: somebody started that tool and watched it
read the file at this path. Documentation is not verification.

## 13. How to add a collector

1. Decide the layer and the profile. If your source can describe more than one running
   system, it produces one profile per system, and every fact carries that profile id.
2. Register a run first:
   `run = builder.add_run(layer, profile=profile_id, input_digest=..., roots=..., predicates=...)`,
   then `builder.declare_layer(layer, present=True, run=run)`. The input digest covers the
   ordered input list, the injected arguments and the probed predicates, never a clock.
   The launch sandbox predicate is the exception stated in section 9 point 8.
3. Create the `pkg` and `file` entities for everything you read, including files you refused
   to read (a ROS 1 XML launch file gets `attrs.loadable = False` and a `launch.unloadable`
   finding). Every evidence row must cite a `file` entity that exists. A refused entity takes
   the one shape section 4 states, attributes and no fact, and `extract.input_refused` is the
   rule that reads it.

   **A refused input leaves its denominator behind.** A want whose only site was inside the
   file you refused becomes one `unresolved` entry owned by that file, with the reason and
   the raw expression, so the want stays in `counters.resolution_rate` and the rate falls.
   Dropping the want instead reports a perfect rate over a smaller denominator, which is the
   exact shape this whole counter exists to refuse: the more a refusal costs, the healthier
   the atlas reads. Two measured cases say it plainly. A urdf that is not parsable xml took
   the `frame_id` want out of the counter with it and the atlas kept every rate at 1.000. A
   python source that is not parseable took `node_name` and `topic_name`. Neither is a dead
   pass, so no pass row and no rate can report it, and the blind spot is the only thing a
   reader meets. The file entity states `attrs.loadable = False` beside it, and the pass is
   still `completed`, because the pass did finish.
4. Emit facts with `builder.fact(entity_id, layer, profile=..., run=run, values=...,
   confidence=..., evidence=[Source(...)])`. The entity is created for you with the kind
   from its id prefix.
5. Emit edges with `builder.edge(kind, src, dst, key=ids.edge_key(...), layer=..., ...)`.
   Both endpoints must exist first. Keyed kinds need a key.
6. For every value you could not resolve, add
   `builder.unresolved(Unresolved(want=..., owner=<sym or node id>, expr=<raw source text>,
   reason=<code>, resolvable_by=[...], evidence=[...]))` and call
   `builder.count_resolved(want, n)` for the ones you did resolve, so the rate has a
   denominator.
7. Never call `supersede` from a collector for an ambiguous match, never write into another
   layer's facts, and never guess a value.
8. Catch exceptions per input file, record them in the run's `exceptions` list, and carry
   on. One launch file that raises at description build time must not abort the layer.
   Whether the pass itself reached the end is recorded by the orchestrator, one row per
   collector under the run's `passes`, through `AtlasBuilder.record_pass`. Three states, and
   a reader of `counters.passes` has to be told which is which, because they mean three
   different things about the same atlas:

   * `completed`: the collector reached the end of its inputs. A collector that finished
     over an empty tree is `completed`, because it read everything there was, and so is one
     that refused a file and carried on. What that file cost is stated by point 3 above and
     by `extract.input_refused`, never by this row.
   * `refused`: the pass stopped with data still to read, so the atlas is missing whatever
     that collector is the only source of. The row carries the reason and what is therefore
     missing. This is the only state `extract.pass_incomplete` fires on, at `error`, and the
     only one `complete` counts: `complete` is `refused == 0`.
   * `not_attempted`: the inputs were absent, so nothing was read and nothing was lost. A
     collector says so by implementing the optional `inputs_present(ctx)`, which returns
     True, or False or the sentence naming what is absent; a collector that does not
     implement it is attempted. `attempted` counts every row that is not this one.

   Six collectors implement `inputs_present` today and each names what is absent in its own
   words: `entrypoints` when the workspace holds no indexed file, `cpp_code` when it holds no
   C++ source, `launch_declared` when the configuration declares no profile, `params` when no
   launch walk reached it, and `bag` when no bag root is configured or none holds a
   recording, with `stamps` using the same discovery as `bag`. Two of those were the wolf cry
   and the false silence at once. `cpp_code` used to load its optional parser before it took its input list, so an installed ROS tree with
   zero C++ files on a machine without the extra was `refused` at `error` on a perfectly
   healthy workspace. `params` used to answer `completed` over an empty walk list, which is
   also exactly what a dead `launch_declared` pass leaves behind, so a parameter pass that
   read nothing was indistinguishable from one that read everything. Take the input list
   before the optional dependency, always.

   A configured root holding no file is `not_attempted` and never `refused`. Reporting an
   absence as a loss makes the gate go red where nothing is wrong, and a rule that cries
   wolf is switched off inside a week, which reopens the hole it closes behind a false sense
   of safety. The opposite mistake is worse and is refused for the same reason: a collector
   that really failed must never report that it had nothing to read.
9. Tests: one fixture package under `tests/fixtures/`, one test that the collector output
   validates against `schema.validate`, and one test that two runs give identical
   `canonical_bytes`.

## 14. How to add a drift rule

1. Add the rule id to the committed list in section 10 and to `rosio/drift/rules.py`. A
   rename needs an alias entry, never a silent replacement.

   There is one registry and `rosio/drift/rules.py` owns it. `@rules.register("<rule id>")`
   over the function is the whole of it, the id has to be in `COMMITTED` before it will bind,
   and `rosio/drift/engine.py` takes its rules from there. Nothing registers a rule with the
   engine, and the engine's `run` takes a rule list only so a test can hand it one. The engine
   kept a second registry with a second `register` and a second `canonical` until this was
   written, reached from no production path, and a contributor arriving at the easiest
   contribution in this tree met two of everything.
2. A rule is a pure function from the joined atlas to a list of `Finding` objects. It reads
   `conflicts`, `layers`, `state`, `candidates` and `effective`. It never re extracts and
   never mutates entities.
3. Scope the rule by profile. A rule that compares across profiles reports every deliberate
   difference between sim and real as drift.

   One rule is the stated exception: `frame.parent_differs` compares the robot models the
   profiles load with each other, so the sensor that sits at one origin in the real model and
   at another in the sim model is reported, which is the defect it exists for (section 5
   cites three models with three origins for one joint). It names the profile loading each
   model, keys on the sorted model list so a deliberate difference is waived once, and
   compares no model that no profile loads.
4. Skip anything whose confidence is `unknown`, anything marked `via_known_wrapper`, and
   any parameter sub key whose plugin instance is not selected.
5. Give the finding a `say` that names the value and the file to edit, a `fix`, and a
   `discriminator` derived from sorted content when one rule can fire twice on one subject.
6. Tests: one test that the rule fires on a crafted atlas, one that it stays silent on the
   near miss case, and one that the finding id is stable across two runs.

## 15. Refusals

A collector, a rule or a renderer must never:

* invent a topic, service, parameter, node or frame id for something it could not resolve
* write a `/tmp` path, an absolute path, a line number or a clock into an id
* write an absolute path into compared data, a run exception row and a refusal detail
  included. The same refusal on two machines is one refusal, and it has to be one atlas: a
  file a collector could not open is named by its `file` id, never by where it sits
* default a QoS, a namespace, a type or a parameter value it failed to track
* merge two sites into one edge by omitting the key
* report a parameter as effective when its state is not `effective`
* claim a declared node set for a profile without naming the entry point it came from, since
  a workspace wide union of launch files describes a system that never runs
* put a synthetic id into human facing prose
