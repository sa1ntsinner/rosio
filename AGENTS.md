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
| `declared` | what the repository says should run: launch files, yaml parameter profiles, urdf and xacro, interface definitions, pluginlib manifests, and the non launch entry points that compose real profiles (a desktop launcher building `ros2 launch` and `ros2 run` command strings) | static reading plus sandboxed execution of `generate_launch_description` and of XML and YAML launch frontends |
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
| `nodes` | `{name, name_source, namespace, namespace_source, form, symbol, class, auto_declare, use_global_arguments}` |
| `classes` | `{name}`, a qualified C++ class or struct declaration, including classes with no ROS records |
| `class_templates` | `{name, definitions, specialized}`, the count of class template bodies in one file and whether a class or member specialization exists; explicit instantiations and forward declarations contribute no definition |
| `constructions` | `{class, owner_symbol, line_hint, definition_file, auto_declare, use_global_arguments}`, the literal type constructed in `main`, as written, with an optional file id for a unique class in an included header of the same package |
| `type_aliases` | `{name, target, namespace, line_hint}`, a namespace scope using or typedef alias with its qualified name and target spelling |
| `endpoints` | `{kind, raw, name, name_state, type, type_id, qos, owner_symbol, via_known_wrapper, condition, guard_parameters, line_hint}` |
| `parameters` | `{name, type, default, has_default, default_state, reads, owner_symbol}` |
| `parameter_reads` | `{name, api, target, owner_symbol, line_hint}` |
| `parameter_writes` | `{name, value, value_state, expr, condition, owner_symbol, line_hint}`, a value the node sets on its own declared parameter |
| `plugin_parameters` | `{base_class, lookup_name, suffix, type, default, has_default, default_state, reads, owner_symbol}` |
| `tf_broadcasts` | `{kind, parent, child, parent_state, child_state, broadcaster, condition, guard_parameters, parent_from_parameter, child_from_parameter, owner_symbol, line_hint}` |
| `tf_lookups` | `{api, target, source, target_state, source_state, target_from_parameter, source_from_parameter, owner_symbol, line_hint}` |
| `plugin_loaders` | `{base_class}`, a `pluginlib::ClassLoader<Base>` the file holds, with the base class as the source spells it |

* `kind` on an endpoint is one of `publisher`, `subscriber`, `service_server`,
  `service_client`, `action_server`, `action_client`, `timer`.
* A python or C++ `timer` row may carry `callback`, the qualified callback symbol, and,
  when its period is read from a parameter, `period_form`: `inverse` with `period_numerator`
  for `1.0 / rate`, or `direct`. A python or C++ `publisher` row may carry `attribute`, the member it
  is assigned to, and `published_from`, the methods that call `publish` on that member. The
  join states `period_s` on a coded `publishes` edge when every publish is in the callback of
  one timer of the same source, from the parameter's winning value in that profile, and
  states nothing when that value is unknown.
  A C++ bound method names that method, and a timer lambda names the enclosing symbol plus
  `lambda_<digest>` over its own text. Literal chrono durations fold to seconds. A parameter
  read already tied to a numeric member keeps that dependency for a seconds duration or an
  inverse rate. A lambda calling another method does not make that method its callback.
* `name` is the raw name the source states, absolute or relative, absent when the source
  could not be read statically. A record that already states the name its wrapper creates is
  taken as it stands.
* A `parameter_reads` row is a read of a parameter no declaration in the same file carries,
  which is what tells a declaration apart from a value the code actually uses.
* `auto_declare` on a `nodes` or `constructions` row is `true` when the node is built with
  `automatically_declare_parameters_from_overrides` or `allow_undeclared_parameters` set to a
  literal true (an rclpy keyword, or a `NodeOptions` call in the C++ file), and `unknown` when
  the value is not a literal. The join states it on the node's `coded` fact, and a node that
  carries it, or whose sources read the parameter back, is never said to ignore a value a file
  sets. A C++ `NodeOptions` call is decided per construction: the `main` construction whose
  arguments hold it, directly or through the options variable it sets, or the class whose
  base initializer takes it. Only a call that cannot be traced to one construction marks every
  row of the file, and the join reads the construction rows of the node's own classes, so a
  helper node built to take any parameter speaks for itself alone.
* `use_global_arguments` on a C++ `nodes` or `constructions` row is `false` when the node is
  built with `NodeOptions().use_global_arguments(false)`, a literal, and `unknown` when the
  value is not a literal, traced per construction the way `auto_declare` is. Such a node reads
  none of the arguments its process was started with: measured on jazzy with an rclpy node,
  which reads the same rcl arguments, it ignored the `__node` and `__ns` rules, `-p` and
  `--params-file`. A launch name over a process that runs
  several nodes renames only the others, so the collision of section 3.5 leaves it out, and a
  split gives it its own namespace, no remap and no parameter the launch site hands the process.
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
When every node row of one process names a literal of its own, each with its own class or
construction, a launch site that names none of them starts one node per name (section 3.5),
and each node binds the records of its own class or construction. A record of a class none
of them is built from, a base or a free function, stays with every one of them.

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
   Beside it, `ENDPOINT_TABLE` lists the endpoints of
   `robot_state_publisher/robot_state_publisher` and
   `joint_state_publisher/joint_state_publisher`, checked against the humble and jazzy
   sources, with the source and reading method in each row. The join states declared endpoint edges only when it found no coded
   carrier and no read source for that executable, applying the same namespace and remap
   rules as coded endpoints. Each edge carries `via_known_wrapper` and cites its table row.
   This is a partial list, never proof that another endpoint is absent. A row holds only
   what both releases state alike: `ros2_control_node` subscribes `~/robot_description` on
   humble and `robot_description` on jazzy, and the join does not know the release, so it
   has no row.

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

One process can run several nodes, and rclcpp names each from its own source. A launch site
that states no name for such a process starts one node per literal name, so the synthetic
identity resolves to several real ones and `AtlasBuilder.split(old, news)` is the call: every
new identity takes a copy of the facts, the edges, the blind spots, the findings and the
derived parameters, keeps the old id in `aka`, and the old entity goes. A slot a new entity
already states is kept, because it was formed for that exact name. A launch `name=` or an
argv `__node` rule is the opposite case: it remaps every node of the process, so the nodes
share one name, and the join reports `node.ambiguous_identity` with a fix that removes the
name or remaps one node at a time (`<node>:__node:=<new name>`), never a fix that names the
site.

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

The join never raises that over source a user wrote, because it writes every coded keyed edge
through `join._site_edge`. Two sites of one symbol that state the same content, two
constructor overloads declaring one name or an `if` and an `else if` asking for one transform,
are one edge holding both evidence rows: a key that told them apart would need a line or an
ordinal, which section 3.1 refuses, so folding is the only form that keeps both sites
readable. The builder takes them with `fold=True`, which skips the evidence check and keeps
the value and confidence checks. Every site takes a digest of its own content and confidence
into its key, so one QoS block never stands for two sites, and a site written above another
adds one edge and moves none: a key that only the second site to arrive took a digest into
handed the plain key to whichever site came first, and inserting a sibling moved an id. A `ContractError` that still escapes the join leaves the atlas written:
`extract._join` records a refused `join` pass on a coded run with a problem row, and
`extract.pass_incomplete` reports it, rather than the run ending with nothing on disk.

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
* `tf_parent` runs from the child to the parent in every layer: a urdf joint, a broadcast
  the code sends, a transform the probe observed. The edge says the parent of `src` is
  `dst`, so a tree holds one parent per child, and one pair stated by two layers is one
  source and one destination with a key per site. Each site keeps its own keyed edge, so the
  `conflicts` of one edge hold one layer and nothing compares the layers on it: the layers
  meet on the pair, a reader that walks the pair (`rosio path`, the frame tree) sees every
  site of it, and a tree view shows one child per pair, the model's row when a model states it.
  A reader never reads the direction off the source of a fact.

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
* A key a file sets for a node whose code the join read whole and which declares no such
  parameter wins nothing in that profile: the node ignores it. The join takes the win off the
  line and notes why on the candidate, and a parameter that wins in no profile is
  `undeclared`, so `rosio param` and `param.set_undeclared` say the same thing about it.
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
* A key the yaml sets that no declaration of the node names is decided per key, never per
  file. A declaration whose name is built at run time hides only the keys its literal text
  could spell (`"sensor." + id` never builds `min_range`): such a key is `other` in that
  profile and wins nothing, with a note saying why, and any other key is `undeclared`. The
  literal text is read as pieces of the name only in a plain concatenation or a format string;
  a condition choosing between names, a literal inside a subscript, or a call taking a literal
  (`names_.at("gain_key")`) could build any name and hides every key, while a subscript
  holding no literal is an unknown piece like a variable (`ids_[i] + ".plugin"`). A plugin
  instance declares `<instance>.plugin` and every suffix its class declares, so a key under a
  selected instance is decided by the plugin and not by a blind spot. The C++ and the python
  collectors both record such a declaration as a `param_name` blind spot and count the
  declarations they read under the same want. What a node declares or reads back is taken
  from the profile the code ran in, because a sim and a real profile can start one node name
  from two executables.
* A node that sets its own declared parameter (`set_parameters` in rclpy, `set_parameter` or
  `set_parameters` in rclcpp) states a `parameter_writes` row. The join puts a `code_set`
  candidate at rank 0 of that profile's chain: a literal written unconditionally wins and every
  other row loses to it, and any other write wins nothing, leaves the parameter `other` and a
  `set_in_code` blind spot, because the value that runs is not one a file states.
* A python write whose value folded but whose guard did not is a candidate carrying the guard
  text, and remains a blind spot: the parameter stays `other` in that profile and nothing
  wins. Dropping the write hides the only line an engineer can edit, and letting it win names
  a value that is wrong on the other branch. A write whose value did not fold adds no
  candidate, because nothing about it is known.
* Every parameters file a launch site hands a process reaches every node of that process.
  When launch names neither the process nor its nodes and no table names the executable, the
  parameter collector follows the chain under the executable's name as before, and also under
  each other name a block of those files is keyed on in the node's namespace that no launched
  node of the profile owns (`params.alternate_owners`). Such a chain states that name as its
  `owner`, keeps only the keys a block of that name writes, and is `owner_name_mismatch` with
  nothing winning, because launch alone cannot say a node of that name runs. The join, once
  the source names that node, lets its first row win, and drops from a chain copied off the
  process every row of a block keyed on another name. Two mappings under one key in one
  parameters file are one mapping, the later key winning inside it, which is what rcl does:
  measured on jazzy with two `/**` keys in one file. The same measurement with two shapes
  under one key reads by flat parameter name: `a: 1` then `a: {b: 2}` starts the node with
  `a.b` alone, and `c: {d: 5}` then `c: 7` starts it with both `c` and `c.d`.
* A key the node's bound code does not declare is not called undeclared when code the node
  reaches without the join binding it declares the key. One rule decides what a node reaches,
  `join.parameter_declarations`, and nothing else adds names. Inside the node's own packages it
  is class by class: the classes of the bound sources, what they inherit, what a node class
  hands itself to (`node_helpers`, which include a member it initializes with itself,
  `fitter_(this)`), and what any reached class builds or calls with the node it holds (`node_handoffs`:
  `Param(node)`, `declare_config(node)`), plus a helper whose handle the reader could not
  trace (`node_handle_untraced`). An unqualified class name is looked up in the enclosing
  namespaces of the class that spells it first, as C++ does. A known wrapper a reached class
  constructs declares what `rosio/resolve/wrappers.py` states for it:
  `diagnostic_updater::Updater` declares `diagnostic_updater.period` and
  `diagnostic_updater.use_fqn`. Another package is reached whole, less
  its code that states a node of its own: the package of a class a node class hands itself to
  (`make_shared<mpc::Controller>(*this)`, `VehicleInfoUtils(*this)`), and every class a plugin
  manifest registers for a base class the node's package loads through
  `pluginlib::ClassLoader<Base>`, because the class it loads is a name only the run knows.
  Reach is followed through. A second executable of the same package declaring the name says
  nothing about this node, so `b` declaring `speed` leaves `speed` set for `a` undeclared.
  Reached declarations count only for whether any code declares the name: they bind no
  endpoint, no parameter and no read, and they never make a node whose own code states nothing
  look read. A name such code builds at run time (`ns + ".stop_margin"`) is a blind spot of
  the node, so a key it could build is `other` and any other key stays `undeclared`.
* A line a node ignores is still not reported when it is needed: another node of the profile
  has a candidate for the same name from the same file and locator and declares the name, the
  `SetParameter` of one group or a `/**` key of a file several nodes are handed. A `/**` key of
  a file only this node is handed reaches this node alone and is reported like any other line.
* A free function, a function template or a lambda a variable holds, that declares a
  parameter under a name it takes as a string argument, is a declare wrapper, and each call
  declares the name the caller passes. A template in a header is a wrapper for every file of
  its package. `get_or_declare_parameter<T>(node, name)` is a wrapper by name, because the
  library that defines it is a dependency the workspace does not hold.
* A node whose build links a `generate_parameter_library(<target> <definition>)` target
  declares every parameter the definition yaml states, nested keys folded into dotted names
  under the namespace key, which names no parameter. A `__map_<param>` group repeats its
  parameters once per value of `<param>`, so a name under it is a blind spot of the node
  stating its literal pieces, and a definition the reader cannot load is one that could build
  any name.

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
   which is where a person debugging that machine already looks. A text value that holds a
   path (the banner xacro writes into a robot description, a launch argument, a tool message)
   keeps its text and has each root's mount point replaced by the root label, `@src/...` or
   `@install/...`, through `Workspace.stated_value`, so the value a node receives is restated
   only where it names the machine.
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

   Two more hold a whole argument list as it was written, because every rule above is read
   off its order: the observed node `command_line`, which `rosio/collect/proc.py` writes from
   the process with the program first, and the declared node `argv`, which
   `rosio/collect/launch_declared.py` writes from the arguments a launch file passes (the
   `argv` of an entry point row in `rosio/collect/entrypoints.py` is the same kind of list).
   A positional argument means what its place says, the frame pair of a
   `static_transform_publisher` among them, so none of them is ever sorted.

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
   A profile reads one tree. A package the roots hold is read where the roots hold it, its
   install share when an install root has one, else its package source directory, and the
   launch walk answers its share and prefix lookups from there before the ament index is
   asked. A package a source root holds and no install root does is unbuilt, and its source
   directory stands in for the share, which is what `install(DIRECTORY ...)` and
   `data_files` copy in the common case. Only the directory is assumed: a file the source
   does not hold at the same package relative path stays the refusal it was, and one the
   source holds under another directory is `build.unbuilt`. The python packages such a
   package installs (`ament_python_install_package`, or the modules beside an ament_python
   `setup.py`) go ahead of any copy of the same name on the walk's python path, and the site
   directories of the install roots go ahead of the rest, as sourcing those trees does.
   Every file read that way cites `src`, and the launch file fact notes which share
   directories came from source.

   The other honest answer was the release. With only the release sourced, `ros2 launch`
   runs the released copy of a package src also holds, until the workspace is built and
   sourced. It was the rule until `1.25` and it is why it went: the entry point a profile
   names resolves in the workspace, and every include, every parameters file and every
   python module after it came from the release, so nav2 main over jazzy read main's
   `bringup_launch.py` with jazzy's navigation launch files beside it, started lifecycle
   managers of two releases on one controller server, cited jazzy values at the lines of
   main's yaml, and raised ImportError on every file that imports a name main's
   `nav2_common` added. Reading the release consistently would have needed the entry point
   from the release too, and then no line the atlas cites is a line in a root: provenance has
   to end at a file a person can edit, and the coded layer reads the workspace sources. So
   the declared layer reads the same tree the coded layer reads. The ament index is still
   asked for every lookup, and a package it answers from another tree is stated, not read:
   the package carries `launch_runs_from`, and `build.underlay` says once for the workspace
   which packages run from which tree until it is built and sourced.
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

The current version is **`1.34`**, and every addition it holds over `1.0` is additive, so no
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

* the `model.mesh_missing` finding id and the model file attributes `meshes_not_installed`
  and `meshes_absent`, added in `1.15`: the `package://` meshes the urdf collector looked for
  and did not find, split by whether the package source holds the same package relative path.

* in `1.16`, the launch exception row `env`, the environment variable a launch file read with
  `os.environ[...]` and the profile did not set, which `launch.raised` names in its fix. A
  launch entity `file_reads` row of role `robot_model` gained `xacro_args`, the arguments the
  `xacro` command of that node passes, sorted by name because xacro folds them into a map. A
  profile whose arguments change the links or joints of the model states that expansion in
  its own `(declared, profile)` slot: the frames, and tf_parent edges keyed by model, joint and
  profile whose values carry the same `xacro_args`. The expansion with the defaults stays the
  urdf collector's `*` statement. An argument the walk could not perform is the blind spot
  `xacro_argument_unresolved` under the frame want, and no frame is named for it. A model file
  named `.urdf` that holds xacro markup is now expanded as xacro, which ends the frame ids that
  spelled `${namespace}` and that no running system ever names.

* in `1.17`, the node attribute `child_of`, the node that creates this one as it starts (a
  costmap under the server that owns it), keyed by that node id with the package that ships it, the same whether the
  server runs as an executable or as a component, so two profiles state one value. `node.observed_only` stays
  silent on a node whose parent the profile launches. An observed node fact may carry
  `via_known_wrapper` from the node table of `rosio/resolve/wrappers.py`, for a whole node a
  library creates beside its owner.

* in `1.18`, an unresolved entry gained optional `written_in`, the file outside every root
  that writes the expression, stated the way `Workspace.stated_path` states it, when its
  evidence can only cite the include line a root holds. A `remap.unresolved` finding names
  that file as where the remap is written and the include as where it is changed. It is not
  part of the entry id, so no finding id moved; the detail of such a finding changed, and with
  it the fingerprint.

* in `1.19`, the rule id `build.unbuilt`, the `refused_because` value `file_after_build` with
  the file attributes `found_elsewhere` and `needs`, and the model file attribute
  `meshes_elsewhere`. A file a launch file reads from the share of a package no install root
  holds, which the package source holds under another directory, is refused as
  `file_after_build` and names in `found_elsewhere` where the source holds a file of that name.
  A robot model whose xacro include is such a file is refused the same way and names the
  include in `needs`. `build.unbuilt` reports all of them in one finding instead of
  `extract.input_refused` once per file. A `package://` mesh no tree holds names in
  `meshes_elsewhere` the paths inside the same package that hold a file of that name, and
  `model.mesh_missing` names them as the fix.

* in `1.20`, `auto_declare` on `nodes` and `constructions` record rows and on the node's
  `coded` fact. A parameter a file sets for such a node keeps its win, and so does one the
  node's sources read back with no declaration this reader follows.

* in `1.21`, the rule id `build.underlay`, the package attributes `share_read_from` and
  `install_not_sourced`, the declared run predicates `underlay:<package>` and
  `workspace:install_unnamed`, and the launch exception row `missing`. A package the roots
  hold whose share lookup the ament index answered from another tree (the release under
  `/opt/ros` for a package only src holds, or for one whose install tree is not sourced) is
  still read there, because that is what `ros2 launch` runs, and the package names the tree
  in `share_read_from` (gone in `1.25`, see below). A parameters file read that way is cited
  at the same package relative path in the workspace, at that file's own line. `missing` is the file id of a
  workspace file a walk step did not find.

* in `1.22`, the package attribute `share_build`, `copied` or `symlinked`, on a package whose
  share the walk read from the install tree beside a src root that no root names. That tree
  is the build of the src package, and `build.underlay` says so instead of calling the
  package unbuilt; `symlinked` means a `--symlink-install` build links files back to src, so
  an edit to those files takes effect without a build.

* in `1.23`, `use_global_arguments` on C++ `nodes` and `constructions` record rows, `false`
  for a node built to read no global argument and `unknown` when the value is not a literal.

* in `1.24`, the coded record `plugin_loaders`, and on a `library_targets` row of the CMake
  file the keys `definition`, `parameters`, `parameter_patterns` and `parameters_unread`,
  which state a `generate_parameter_library` target: its definition file, the dotted names it
  declares, the expressions of the names a mapped group builds, and a definition that would
  not load. Declarations in a member initializer list, through `node->template
  declare_parameter`, through a declare wrapper in a header of the package or a lambda, and
  through `get_or_declare_parameter` are read where they were missed, so the parameter
  records, the `param_name` rate and the parameter states of such stacks moved, and
  `param.set_undeclared` stopped firing on keys that code declares. The `classes` record row
  gained `node_handoffs`, the classes a class builds and the free functions (`name()`) it calls
  with the node it holds. A `+=` on a string extends it rather than replacing it. What a node
  declares through code it reaches is decided per node and no longer per package, so a second
  executable of the package no longer silences a key, and a shared line is silent only where
  another node is handed the same line and declares the name. No id form moved, so no stored
  id or waiver stopped matching.

* in `1.25`, the package attribute `launch_runs_from`, which replaced `share_read_from`
  and names the tree `ros2 launch` runs a package from while the walk read the workspace copy
  (section 9 point 9), and the plugin manifest attribute `manifests_held`, the manifests a
  package holds beside one an export names and it does not. `share_read_from` named the tree
  the walk read, which is no longer another tree, so the key went rather than change its
  meaning; no release had shipped, so no reader of it existed.

* in `1.26`, the `refused_because` value `cpp_refused_by_atlas`, a C++ source whose records
  the atlas refused (two sites folded into one statement), which is now a refused file entity
  with `refused_detail` instead of a run exception row alone, and the unresolved reason
  `frame_empty`, a frame literal that names no frame (`""`, `"/"`), which no longer mints
  `frame:`.

* in `1.27`, the unresolved reason `angle_unreadable`: a roll, pitch or yaw a
  `static_transform_publisher` is handed that is not a number leaves `rotation.x` to
  `rotation.w` of its node blind spots, where the rotation was dropped and nothing said so.

* in `1.28`, the rule id `remap.matches_nothing`; the candidate mechanism
  `launch_set_parameter`, a value a launch `SetParameter` hands every node of its group, which
  was `launch_inline_dict` before; the `manages_lifecycle` mechanism `autostart`, a
  `LifecycleNode(autostart=True)` that launch_ros configures and activates itself; the file
  attribute `similar_files`, the files in the directory of a missing parameters file whose name
  is close to its name (`lidar.yaml` beside a default naming `lidar_params.yaml`); and a declared `broadcasts_tf` edge from a `static_transform_publisher`
  whose frame pair its arguments spell as literals, with the value `static`, so the frames it
  names exist in the declared layer.

* in `1.29`, the coded file values `name_literals`, the string literals of a source shaped
  like a ROS name, and on a python source `node_handoffs`, the calls and bases outside the
  file that are handed the node (`InteractiveMarkerServer(self, 'markers')`, a node base
  another module defines). Both let `remap.matches_nothing` stay silent where a name could
  come from code nobody read.

* in `1.30`, the rule ids `type.incompatible`, `service.no_server`,
  `launch.argument_undeclared` and `frame.broadcast_twice`; on a declared include fact
  `args_unread`, a `{name, closest, line_hint}` row for every argument the include passes that
  no file of the included tree spells, with the closest name the tree declares and the line
  that writes the name, and `arg_lines`, the line of the including file that
  writes an argument's literal value when that is not the include line; the blind spot reason
  `component_class_unregistered`, a composable node whose `plugin=` no component registration
  of the workspace names; on a `file_reads` row of role `robot_model` `args_unread`, the xacro arguments
  the launch line passes that no file of the model declares or reads; on a `library_targets` row `components`, the classes an
  `rclcpp_components_register_nodes` line of the build registers for that target; and a declared `publishes` edge from a launch
  `static_transform_publisher` to `/tf_static`, marked `via_known_wrapper` like the one a
  `ros2 run` publisher has.

* in `1.31`, the rule ids `param.read_undeclared`, `controller.undefined` and
  `controller.joint_missing`, the last two a family of their own in
  `rosio/drift/controllers.py`. An rclpy read spelled `get_parameter_or(...).value`, or
  through a one hop accessor, states that call as its `api` where it stated
  `get_parameter_value`, so a reader can tell a read that raises from one that answers a
  default, which moved the keys of those `reads_param` edges once. A C++ read through an
  accessor (`get_parameter("x").as_double()`) states `get_parameter_or_accessor` when the call
  is `get_parameter_or`, and keeps `get_parameter_accessor` for `get_parameter`, which raises.
  A python or C++ `get_parameter`
  under a guard asking `has_parameter` for the name states the api `get_parameter_checked`,
  and `has_parameter(name)` is no longer read as the value of that parameter, so a guard on it
  is no `guard_parameters` term. The coded file value `stamped_frames`, a `{frame, line_hint}`
  row for every literal a python or C++ source writes into a message header
  (`msg.header.frame_id = "camera_frame"`), which `frame.not_in_urdf` reports when no model and
  no transform names the frame; in a profile that launches a node whose code was not read,
  which could broadcast any frame, it fires only when a model link names the same part with
  another suffix (`camera_link`). The model file attribute `mesh_lines`, the line that writes
  each missing mesh name as it is written, which `model.mesh_missing` cites. The coded
  `publishes` edge value `period_s` and the python endpoint row keys `callback`,
  `period_form`, `period_numerator`, `attribute` and `published_from` (section 2.2), so
  `rate.below_declared` holds a measured rate against the timer that drives a publisher; the
  C++ reader states the same keys for bound methods and timer lambdas, and for publisher
  members whose publish calls it reads. No release had shipped, so no stored id existed.
  Declared `publishes` and `subscribes` edges from the known executable endpoint table
  use the existing `raw_name`, `type`, `qos`, `remapped`, `remap` and `via_known_wrapper`
  keys, with the table row as evidence. They bind only executables whose source the coded
  layer did not read, and state no complete endpoint inventory. They add graph connections
  and can retire remap blind spots, so findings about those gaps can go. No schema key or
  stored id form changed, and the minor stays `31`.

One package key was replaced in `1.25` without a major bump. `share_read_from`, added in
`1.21`, named the tree the walk read a package from, and since the walk reads the workspace
copy of a package the roots hold that is no longer another tree. `launch_runs_from` names the
tree `ros2 launch` runs the package from instead, which is a different meaning, so the key
went rather than change what it says. No release had shipped, so no reader of it existed.

One observed key was renamed in `1.17` without a major bump. The process table wrote the
whole command line of a node under `argv`, the key under which the declared layer states only
the arguments a launch file passes, so every launched node with arguments disagreed with
itself. The process table now writes `command_line`. No release had shipped, so no reader of
the observed key existed.

Two kinds of id moved under `1.17` and `1.18` without a bump, for the same reason. The blind
spot of an include outside every root stated the machine path of the included file in its
expression until `07c05d6`, so its id differed between two boxes, and every such id moved
when the expression became the stated path. A file outside the checkout and every setup
directory is stated by its name alone there, and two includes of `robot.launch.py` from two
directories were one blind spot, so that form now carries the hash of the file's content as
`Workspace.stated_by_content` states it, and those ids moved again. An observed node whose
name one process start decides (`launch_ros_<pid>`, `_ros2cli_<pid>`,
`transform_listener_impl_<address>`) was `node:/<that name>`, a new id on every restart; it
is now synthetic over its package, its name stem and its namespace, with name source
`runtime_randomised` and no `name` value, so its parameter ids moved with it and the observed
run digest names it by that id. A `remap.unresolved` finding over one remap written both in a
root file and in an outside file it includes names each writer with the nodes it starts, which
changed its detail and its fingerprint and kept its id.

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

A model python builds is read the same way, inside `1.18` and with no new key. A launch file
that calls `xacro.process_file`, `xacro.process` or `xacro.parse` with `xacro.process_doc`
hands the walk its `mappings`, which become the `xacro_args` of the read, and a mapping value
that is not a string the walk performed (an unrun command) is the same
`xacro_argument_unresolved` blind spot. The role digest of a node handed such a document no
longer reads the text, whose banner names the model by where the machine holds it, but the
document spelled as its command with the path relative to its root, so the synthetic node id
and the launch entity id of that node moved once and stopped depending on the checkout. No
release had shipped, so no stored id existed.

Five changes inside `1.18` moved ids or text without a new key. A `PackageNotFoundError` row
names the prefixes the walk searched by root label or ROS release (`searched: ROS jazzy (install
root)`) instead of the paths, which `@outside` made unreadable, and the message is in the
`launch.raised` discriminator, so those finding ids moved once. The `env` value of a launch
exception row may name several variables, space separated, the one the walk stopped at first:
the others are what the same file reads with no default and nothing sets, so one extract names
them all. A configuration that states `resolution_budget` writes it under `x.drift`, where
`resolution.below_budget` already read it, and enters the run input digest. A configuration
`rosio extract` falls back to for an installed ROS prefix read alone ignores `/include`, `/opt`
and `/lib/python*`, which moves the root hash and the run ids of such an atlas. A synthetic
node whose executable is its package is named once in prose (`the rviz2 node in /robot1`),
which changed the detail and the fingerprint of findings naming one. No release had shipped,
so no stored id or waiver existed.

Inside `1.19` several findings moved without a new key. `resolution.below_budget` is one
finding per run, about the profile that owns the most blind spots and naming the worst three,
where it was one per profile, so its id moved. `param.orphan_block` is one finding per file and
block with the profiles it is dead in, its discriminator the block name alone where it was
`<profile>.<block>`, so one deletable block is one finding and its id moved. A
`ModuleNotFoundError` row names the ROS release whose python path was searched
(`searched: ROS jazzy`), so its `launch.raised` id moved, and a module a released ROS package
ships gets the apt package as its fix. An import that fails on a missing module is no longer
`launch.unloadable` as well: the file is a launch file that cannot run here, and
`launch.raised` says why. `launch.unloadable` fires on a profile entry point naming a launch file
no package in the roots holds, about the profile, discriminated by the entry point, and the
blind spot `launch_file_missing` owned by the profile takes the rate down with it. An XML or
YAML launch file is walked through the frontend `ros2 launch` uses instead of being refused by
shape. A package lookup that fails while a launch file builds its description is answered with
a directory that does not exist, so the description builds, the row the failure always gave is
kept, and the include that reads from that package is the blind spot `package_not_found`
rather than every node of the profile lost. A parameter a file sets and the node's code never
declares wins in no profile and is `undeclared` once no profile is left where it wins, with no
effective value; its candidate keeps the note that the node ignores it. A `model.mesh_missing`
finding over a mesh the package holds elsewhere changed its detail and its fingerprint.
`plugin.unused` is silent on a manifest read from an install tree with no source beside it,
and no finding cites a file of rosio itself as where it is true. No release had shipped, so no
stored id or waiver existed.

Inside `1.20` several things moved without a new key. `resolution.below_budget` keeps one
subject for the whole run, the first profile by id, else the first package, and names the
worst profiles in its text, because a subject that followed the worst profile moved the id
whenever a repair handed that place to another profile and un waived the finding.
`launch.unloadable` states one finding per profile and missing entry point however often the
profile names it. A launch exception row's `env` names the other variables by the sourced
environment the walk ran with, and names only the one the walk stopped at when that
environment could not be read, and it no longer names a variable read with `os.environ.get`
or `os.getenv`, which answer None rather than raise. A launch file builds its description
once as `ros2 launch` builds it, and only one that raised after a package lookup missed is
built again with the missing package answered, so a file that catches the miss itself keeps
its own decision and records no `PackageNotFoundError`. No release had shipped, so no stored
id or waiver existed.

Inside `1.21` several things moved without a new key. A tool message names a path inside a
root by its package (`bot/config/x.yaml`) rather than by the label of the root it is mounted
under (`@src/ws/bot/config/x.yaml`), which moved the ids of blind spots whose expression
quotes such a message and the discriminators of `launch.raised` findings over one. A file the
launch sandbox did not find outside `/tmp` is reported by name instead of as a sandbox
refusal, and a launch failure on a share file only a build puts in place is left to
`build.unbuilt`. A robot model a launch line expands from such a file is the blind spot
`file_after_build` rather than `xacro_failed`. A node a free function creates from a node
type with a literal name (`rclcpp::Node::make_shared("talker")`) names its launch site, so
those synthetic node ids resolved and moved. Finding prose states file ids and root labels as
paths, which moved the fingerprints of findings whose detail held one. A parameter key that
no declaration names while a declaration built at run time could is no longer `effective`,
and `param.set_undeclared` decides per key, so a file holding one such declaration no longer
silences every key of its node. No release had shipped, so no stored id or waiver existed.

Inside `1.22` several ids moved besides the new key. `resolution.below_budget` is about
`pkg:@repo`, the workspace, which extraction now writes into every atlas, instead of the
first profile by id, so its finding id moved once and no added profile or package moves it
again; an atlas without that entity keeps the old choice. A python declaration whose name is
built at run time is a `param_name` blind spot rather than an `other` one, which moved the id
of each such entry, and python declarations the reader followed now count under
`param_name`, which moved that rate. `build.underlay` names a tree outside the workspace as
`a tree outside this workspace`, with `that ros_setup sources` when a setup file holds it,
instead of its last directory, and states the build beside src as a build, which moved the
fingerprints of those findings. No release had shipped, so no stored id or waiver existed.

Inside `1.22` a process a launch site starts unnamed, whose source names each node it runs
with a literal of its own, became one node per name, which moved its synthetic node id and
the parameter ids under it to real ones and retired the `node.ambiguous_identity` finding the
join wrote for it. A block keyed on one of those names now carries the chain that node reads.
The fix of the runtime `node.ambiguous_identity` finding names the per node remap when one of
the candidates runs several nodes. No key was added and no release had shipped, so no stored
id or waiver existed.

Several finding texts moved in one change without a version bump, and they are named here
because a fingerprint covers the normalised detail while no finding id moved. `build.stale`
also fires on a launch file the walk ran from the install tree, which is the copy `ros2 launch`
runs once a package is built, and on a python module whose installed copy holds different
bytes, which reuses the attribute `install_copy_differs` on that file; its title and detail
say which tree the atlas shows. `extract.input_refused` names the line of a yaml syntax error
and lost a sentence about the rule's own history. `node.ambiguous_identity` over an
executable that runs several nodes in one process says so and no longer asks for a launch
name, which would rename all of them. `resolution.below_budget` reports at `info` until the
configuration states a budget, because under the default nothing in a user's code lifts it.
The declared `node` fact lost `gated` for the reason `conditions` went in `1.6`: one list of
composable nodes loaded into a new container or into an existing one wrote both values into
one slot. The gates stay on each `launchent` fact, and a site whose condition is false in a
profile no longer counts as a second instance of the node there.

The fifth first run trial moved several ids and texts inside `1.22` with no new key and no new
rule id. `param.type_changed` compares the winning value with the declared type only, so a
shadowed candidate of another type and a value that is a whole `$(...)` substitution no longer
fire it; its detail and say name the declaration, which moved its fingerprints and kept its
ids. `extract.input_refused` says `could not read it (<reason>)` where it said `recorded
<reason>`, names the launch line that built a missing path in its fix and its `where`, names a
package a xacro include misses, and is silent on a model a live launch line expands with
arguments. A refused file whose detail only restated the reason (`file_missing`) carries no
`refused_detail`, and a missing file a site gated off in the profile reads is no refusal at
all. A xacro refusal detail lost the `declared.urdf: xacro refused <file>: ` prefix and spells a
missing package in plain words, which moved the ids of those `frame_id` blind spots, whose
expression is the detail. `launch.raised` names an unknown launch tag, and it and
`launch.unloadable` cite the file without the phase word as locator. `param.set_undeclared`,
`resolution.below_budget` and the build fix of `build.unbuilt` and `build.underlay` changed
text. A topic blind spot the collector already owns at the symbol is no longer written a
second time under the node, so those node owned ids are gone. A time travelling
`lookupTransform` or `canTransform` reads its source from the third argument, which moved the
key of the `listens_tf` edges that named a time as a frame. A parameter default that is a cast
of a number or `std::string()` folds, and one that is a local variable has the blind spot
reason `local_variable`; the reason is not in the id. A target `ament_auto_add_executable`
or `ament_auto_add_library` adds is read as its plain spelling, and an executable takes the
name `OUTPUT_NAME` installs it under, so such an executable has a new `exe` id. No release
had shipped, so no stored id or waiver existed.

Its follow up removed findings and changed no id. `param.set_undeclared` is silent on a node
that ignores a line every node of the profile reads, a `/**` key or a launch `SetParameter`,
while another node of the profile declares the name, because that line is not deletable for
one node. The parameter stays `undeclared` with the note saying the node ignores it.
`extract.input_refused` is silent on a xacro model no profile loads whose expansion failed only
because an argument with an empty default named a package (`$(find ${vehicle_model}_description)`),
which is a template waiting for its argument rather than a broken file. `rosio init` keeps
offering a launch file other entry points include when one of them that nothing includes asks
for the same arguments, so a top level file its simulator includes is still a profile.

Inside `1.22` the coded records of a C++ class that is not a node moved, with no new key.
A class a node class hands itself to (`this`, `shared_from_this()`, or a member or local that
only ever holds a shared pointer built over `this`), by `std::make_unique`, `std::make_shared`
or `new`, binds its records to that node, and so do the methods it and its bases define in a
header of the package. In a process that runs several nodes, a record of a class only one
node inherits or hands itself to is that node's alone, where it stayed with every node. A
class the package constructs at one site takes the literal arguments of that site into its
constructor, so the topic and service names it was handed stop being blind spots. A record of
a class no node, component or plugin class reaches that way, in a source a build target
compiles, of a package whose executable the join followed, is the blind spot
`node_handle_untraced`, role `helper`, under the want `other`, one per method, where it used
to vanish. A node whose every launch site is off in a profile binds no code
there: a load into an existing container beside a new container, under a condition read
false, minted a second node that published what the first did. No id form moved, so no
stored id or waiver stopped matching; the endpoints, parameters and rates of such stacks did.

Inside `1.25` several ids and texts moved besides the new keys. A launch file of a package
the roots hold is read from the roots, its includes, parameters files and python modules with
it, so the nodes, parameters, values and blind spots of a workspace that holds a package a
sourced release also ships moved, and `param` and `diff` no longer tag a value as read from a
release. A configuration that states no `ignore` skips the `test` and `tests` directories,
which moved the root hashes, the run ids and every entity a test source held, and `rosio init`
leaves that default out of the file it writes. An explicit `profiles: []` is taken as written,
and the fallback offers no ROS 1 or test launch file. A ROS 2 XML launch file with a
`launch-prefix` is read rather than refused as ROS 1. A launch file that raised is
`launch.raised` alone, and the reason of `launch.unloadable` is the refusal note alone.
`build.underlay` is one finding about the workspace (`pkg:@repo`) instead of one per package.
`plugin.inert_by_design` is one finding per file, owner and block with the profiles it is inert
in, its discriminator the owner and block without the profile, and `param.restated_identical`
is one finding per restated line and value with its profiles, its discriminator a digest over
the line and the winning file. `plugin.unused` names only the profiles that start the node
holding the selector list. An ImportError message names the tree the module was read from and
the workspace copy when there is one, and a ModuleNotFoundError message says when the release
installs a package of that name without the module, which moved the discriminators of those
`launch.raised` findings. A file an unbuilt package's install rule puts in place and an include
misses is `file_after_build`, so `build.unbuilt` takes it from `launch.raised`. No release had
shipped, so no stored id or waiver existed.

Inside `1.25` the C++ reader closed eight gaps with no new key. A member a node class
initializes with itself in a constructor's initializer list (`fitter_(this)`) is a class it
hands itself to, typed from the field its header declares. A known wrapper member a header
declares and a source initializes (`diagnostics_(this)`) is a construction, so its endpoints
reach the node, and a C++ name that nests a wrapper's symbol in a namespace of its own
(`vendor::wrapper::diagnostic_updater::Updater`) is that wrapper. The wrapper table states the
parameters a library declares, which count for `param.set_undeclared` and are no atlas key. An
`.ipp` file is read as a C++ source of its package wherever a header is, and its content enters
the root hash, which moved the run ids of a tree that holds one. An all caps identifier between
`class` or `struct` and the class name (`class FOO_PUBLIC Controller`) is an export macro and
no part of the name when a base clause, `final` or a brace ending its line follows the name
(`struct POINT origin{0, 0};` declares a variable), and a function name a formatter wraps
after `::` is one name, which moved the `sym` ids of such functions. An unqualified class a node hands itself to is looked up in
the enclosing namespaces first, so a name several packages define no longer reaches all of
them. No release had shipped, so no stored id or waiver existed.

Later inside `1.25` several ids and texts moved with no new key. `ids.digest` never returns
a value made of digits alone: one short digest in about forty was, a finding or a launch step
whose discriminator is such a digest was refused as an ordinal, and the rule that wrote it
landed in `counters.drift.failed`. The first digit of such a value is now a letter, which
moved every id, edge key and discriminator that held one, and only those.
`param.restated_identical` is one finding per line whatever each profile restates there,
where two profiles restating different parameters or values on one line wrote two findings
under one id; its detail names what each profile restates when they differ, which moved
those fingerprints. A test directory a profile names a launch file through is left out of the
default `ignore`, which moved the root hashes and run ids of such a workspace. An ImportError
read from the workspace copy of a module names that copy as the one lacking the name instead
of naming it twice, which moved the discriminators of those `launch.raised` findings. The
site directories of the install roots go on the python path of the walk in the order of the
roots, and a CMake target lists its `.ipp` and `.hxx` files as sources. No release had
shipped, so no stored id or waiver existed.

Inside `1.26` several ids and texts moved besides the new keys. A component library binds the
helper classes, header members and bases of the class it registers, so a composed node states
the endpoints a helper makes, and `rclcpp_action::create_server` and `create_client`, and nav2's
`create_action_server` and `create_action_client` (the member, and the `nav2::interfaces` free
function that takes the node first) are action endpoints, so `action` entities and their `serves` and `calls`
edges exist where there were none. The wrapper table holds `nav2_util::TwistPublisher`, whose
second argument is the topic. A `std::string` parameter of a plugin's `configure` or
`initialize`, and `getName()`, of a class its package registers with
`PLUGINLIB_EXPORT_CLASS` are the plugin instance name, so `name + ".scale"` there is a
suffix of a `pparam` and not a `param_name` blind spot, and a helper the plugin hands that
name to declares suffixes of the same plugin; the parameter states and the `param_name` rate
of a plugin stack moved. A `param_name` blind spot whose name a local holds states the
local's initializer (`prefix + ".value"`) as its expression, which moved those entry ids.
`static_transform_publisher` arguments, flags or the positional form, are parameters of its
node (`frame_id`, `child_frame_id`, `translation.*`, `rotation.*`), its role digest covers
them, and a frame its source reads from a parameter is no longer minted from the default, so
the synthetic ids of static publishers moved and `frame:frame` and `frame:child` went. Two
broadcasts of one frame pair in one C++ symbol that state different content (a condition, a
broadcaster) are two `tf_parent` sites, where they raised; a C++ broadcast states no origin,
so it is not in the key. A node only a site whose condition is false in every
profile starts is not an entity and owns no blind spot: the launch entity stays, with its
`launches` edge, which removed synthetic nodes and `node_name` entries. `import a.b` binds
`a`, so a type spelled `sensor_msgs.msg.Image` is `type:sensor_msgs|msg|Image`. A python
`sendTransform` reads every transform of a list and the one a method of the node builds and
returns, and an element it cannot follow is a `frame_id` blind spot. `param.set_undeclared`
over a dict a launch file passes asks to remove the key from that dict rather than to delete
the line, and `rosio query` over an endpoint relation no edge answers names the blind spots of
that want. No release had shipped, so no stored id or waiver existed.

Inside `1.27` several ids and texts moved besides the new reason. Every coded edge the join
writes over a site (`publishes`, `subscribes`, `serves`, `calls`, `broadcasts_tf`,
`listens_tf`, `tf_parent`, `declares_param`, `reads_param`) and every C++ `tf_parent` edge
carries a digest of its own content in its key, so every such edge id moved once. A
`std::string` parameter of `configure` or `initialize`, `getName()` and `name_` name a plugin
instance only in a class its package registers with `PLUGINLIB_EXPORT_CLASS` (a template
through the alias the macro names) or in a helper a registered plugin hands its name. A file
that states the suffixes of a plugin another file of the package registers binds them to that
plugin, and a suffix no registration owns is a `param_name` blind spot with reason
`plugin_instance_prefix` instead of a node parameter named by the suffix, which moved
`pparam` entities, parameters, states and the `param_name` rate. A helper a plugin's setup method hands its name
argument directly declares suffixes of that plugin, as one handed the member already did. A
local assigned more than once (the arms of an `if`) states its spelling as the expression of
its `param_name` blind spot, which hides every key, where it stated the last assignment. A
python method that builds and returns a transform is read in its own scope and only from the
caller's class, so a frame no literal or member of that method states is a `frame_id` blind
spot. A namespace qualified `nav2::interfaces::create_action_server` or
`create_action_client` takes its name from the second argument whatever the node handle is
called, and the member form from the first. A python frame literal that names no frame is the
`frame_empty` blind spot, where it refused the whole file. A C++ file the atlas refuses keeps
none of the records it wrote before the refusal, and the pass goes on. A rotation a
`static_transform_publisher` reads from angles cites the flags written, or `positional`, where
it cited `--roll --pitch --yaw`. No release had shipped, so no stored id or waiver existed.

Inside `1.28` several ids and texts moved besides the new keys. `param.set_undeclared` is one
finding per line, listing the profiles and the nodes it is handed to, and its subject was the
parameter when the line set one parameter and the file otherwise (always the file from
`1.29`); its discriminator is a digest
over the file, the locator and the names, so every such id moved. A key a `SetParameter` hands
a group is asked to be stopped at that line only when no node of the group reads it, never
removed from a dictionary. The file list of a `set_undeclared` finding names the node's own
sources first. `remap.unresolved` is silent on a remap `remap.matches_nothing` reports, and the
remaps one file writes on the same nodes are one finding whose discriminator is a digest over
their sorted expressions, which moved those ids. A selector list of `filters` or
`path_handler_plugins`, a block a string value names (`motion_model: diff_drive`) and a plugin
block nested in a selected instance are live, so `plugin.inert_by_design` stopped firing on
them, and its title names the node that owns the block. `extract.input_refused` over a missing
parameters file cites the `DeclareLaunchArgument` default that names it, and its fix names a
file beside it whose name is close. `qos.differ`, `type.differ` and `rate.below_declared` compare the edges of one endpoint as one
when each layer states it on one edge at most, and a layer that states it from two sites keeps
every edge apart, so such an endpoint is not compared with the observed one; and `endpoint.observed_only` is silent on an observed
endpoint another site of the same node states in source. An rclpy `QoSProfile` built with only
a depth states the reliable and volatile defaults rclpy gives it. `topic.no_publisher` over an
observed topic names the remap that moved its publisher away. The observed `executable` of a
python node is its script, a Zenoh router process is never a profile command, and `rosio
probe` counts the frames observed `tf_parent` edges name. `build.underlay` titles say the
count is of the packages the walk looked up. No release had shipped, so no stored id or waiver
existed.

Later inside `1.28` the meaning of one edge direction changed with no new key, which would be
a major bump once a release has shipped. A `tf_parent` edge the C++ collector or the join
writes from a broadcast ran from the parent to the child while a urdf joint and an observed
transform ran from the child to the parent, so one pair stated by two layers was two pairs and
no layer could be compared with another. Every `tf_parent` edge now runs from the child to the
parent (section 5), which moved the id of every coded `tf_parent` edge, and a reader takes the
child from `src` instead of from the source of the fact. A xacro model's joints count as model
joints for `frame.parent_differs` and the published frame set, where only a `.urdf` did.
`frame.not_in_urdf` reports, at `warn`, a frame the code broadcasts by a literal name under a
link of a model a profile loads when no model declares it and that model places a child of the
same link under a close name, naming both frames, both lines and the profiles. `rosio path`
between two frames walks the transform tree first and states `tree_route`. The wrapper table
holds `nav2::TransformBroadcaster`, `nav2::StaticTransformBroadcaster` and
`nav2::TransformListener`, also when nav2's `create_*` factories build them. A C++ string local
that `append` extends is extended the way `+=` extends it, and a local whose last value was
written inside a branch the read is outside of is a blind spot spelled by its name rather than
the last literal, so a name two branches assign declares neither. A name built by extending a
literal states the literal in its blind spot expression (`"sensor." + id`), which moved those
entry ids. No release had shipped, so no stored id or waiver existed.

Inside `1.29` several ids and texts moved besides the new keys. `param.set_undeclared` is
always about the file that holds the line, so a profile that adds a node no longer moves its
id, and it cites every line it groups. A yaml line is still named by its key, and a launch
action that hands keys to nodes (a `SetParameter`, an inline dict, a `-p`) by the nodes it
hands them to, so the keys of one inline dict are one finding and a key added to the action
moves no id; a line a node reads silences only itself, where a `SetParameter` read in one
group silenced the same name in every group of the file. `remap.unresolved` is discriminated
by the nodes the file itself writes the remaps on and no longer by the remaps, so neither a
remap added beside another nor an included file writing the same rule moves its id, and every
such id moved once. A coded site edge whose values hold a
mapping or a list (a qos block) digests them in the pinned encoding rather than in the order
they were built, so those edge ids moved once. A wrapper endpoint a header builds and its
constructor owns no longer carries the header's line beside the source file it cites. An rclpy
`create_lifecycle_publisher` and an rclcpp `create_generic_publisher` or
`create_generic_subscription` are endpoints, and a C++ class row no longer lists the API calls
this reader reads (`create_publisher()`, `bind()`) among its `node_handoffs`, while it now
lists the free functions and classes handed a node interface (`get_node_topics_interface()`).
No release had shipped, so no stored id or waiver existed.

Later inside `1.29` several ids and texts moved with no new key. A `SetParameter` or
`SetParametersFromFile` site is located by the scope it pushes into,
`SetParameter(<digest>)`, a digest over the processes and includes of its group as the launch
file writes them, and an inline dict a launch action hands its node by that action,
`Node(<name>).parameters`, where both were a bare word told apart by the line alone. Every
candidate, blind spot and finding that cites such a site moved. `param.set_undeclared` names
a handed line by that locator alone, no longer by the nodes it reaches, so a profile that
starts one more node of the group, or the same node in another namespace, moves no id, and
whether another node needs the line is decided by file and locator without the line hint.
The renamed variant of `frame.not_in_urdf` fires only when the two names share a stem and
differ by a `_link` or `_frame` suffix, so `imu_link` beside `gps_link` and `camera_link`
beside `camera_link_optical` are no longer one part. A C++ name a `switch` arm or a loop body
assigns is a blind spot outside that arm or body, as one an `if` assigns is, which moved
parameter records and `param_name` entries. `name_literals` leaves out a path under a
filesystem root (`/tmp/...`, `/dev/...`, a home directory) and, in C++, the text of comments and
character literals. An rclpy `ActionClient` or `ActionServer` given its node, type and name by
keyword is an action endpoint, where it was a node handoff. `remap.matches_nothing` reads a
remap scoped to one node of a site that starts several for that node and skips the others,
where it gave up on the remap. No release had shipped, so no stored id or waiver existed.

Inside `1.30` several ids and texts moved besides the new keys. `qos.incompatible` is one
finding per topic and profile, told from the first layer of coded, declared and observed that
holds the pair and listing every layer that does, so its discriminator lost the layer and its
id moved; a mismatch the probe confirms no longer adds a second finding with no line to edit.
An rclpy `ActionClient` or `ActionServer` is one action endpoint with its type, as the C++
reader reads `rclcpp_action`, where it was five endpoints under `_action/`, so those edge ids
and the topics and services they named went. A C++ endpoint type spelled through a `using`
alias of the file or of the class is stated as the type it names, which moved the values of
those edges. A `message_filters::Subscriber` built in a constructor's member initializer is an
endpoint. `launch.argument_undeclared` and the component case of `plugin.unresolved` are
about the `launchfile` entity when one exists, which moved those finding ids. An unnamed
`ros2_control_node` is `/controller_manager`, which moved its synthetic node id, and
`param.orphan_block` is silent on the block of a controller that manager loads by a
`<name>.type` key and on a parameters file only sites switched off in the profile hand. `rosio
init` comments a profile out only when a file of the same package includes it with no
condition and no argument, read by package and path rather than by file name. `rosio
probe` without `--profile` over several profiles files the observation against the one
profile whose `ros2 launch` commands run, with their arguments, and asks for `--profile` when
the running launch commands match none or several. A path inside a package enters an id or
compared data as `<package>/<path inside it>`, read from the nearest `package.xml`, so the
source copy and the install copy of one file are one value and a `colcon build` moves no
launch entity, synthetic node or finding id; the ids that held a root relative path moved
once. `topic.no_publisher` and `topic.no_subscriber` skip the topics behind an
action, `endpoint.observed_only` skips a `/clock` subscription, which the time source of a
node run with `use_sim_time` makes, and the graph states no missing end on
`/parameter_events`, `/rosout` or `/clock`. `frame.orphan_root` reads a tree repeated under another frame prefix
(`robot2/odom` beside `robot1/odom`), or one a static publisher feeds through a remapped
`/tf`, as another robot rather than a second root. The title of `qos.incompatible` names the profile, and
`param.type_changed` spells types as ROS does (`double`, `integer`, `string`), which moved
their fingerprints. `rosio extract` says when it replaces an atlas that carried the observed
layer. `lifecycle.unmanaged` names the profile in its title, `param.shadowed` over a node
key a later `/**` block of the same file overrides says to move or trim that block, and a bare
yaml key that is the last segment of a namespaced node's name is `param.owner_name_mismatch`
rather than `param.orphan_block`. `param.sim_time_mixed` cites the line that writes a value an
include hands over, when the including file writes it once beside the argument name. No
release had shipped, so no stored id or waiver existed.

Inside `1.30`, with no new key, an all zero header stamp series produces only `time.zero`
at `info` and takes no part in other stamp checks or their reference medians. Types whose
zero stamp means start now produce no finding, mixed zero stamps keep their error, and each
`time.*` fix names only its own check.

Inside `1.31`, with no new key, an action of an XML or YAML launch file cites the line of its
element (`<node` or `- node:`), which the frontends' own parsers do not keep, so the walk
parses such a file a second time with expat or a marking yaml loader; a line hint is advisory
and outside compared data. An action of an included file is named in prose by the file it is
written in (`sensors.launch.yaml Node(camera)`) and no longer by the entry point, which moved
the fingerprints of findings that name one. `rosio param` says when the code declares
another type than the winning value, and `rosio path` takes no hop from a publisher's topic to
a subscription whose QoS it can never match, and says so when that leaves no data route. A
`static_transform_publisher` a profile starts with `ros2 run` states its frames, its
`broadcasts_tf` edge and its `/tf_static` publisher the way a launched one does, keyed by the
entry point. A python action name the reader could not follow is a blind spot under the want
`action_name`, as the C++ reader files it, where it was `other`, which moved the ids of those
entries and lets a missing action server say it was not found rather than absent.

Later inside `1.31`, with no new key, `rate.below_declared` takes a recorded bagtopic rate
through its `samples` edge when the publisher has no observed rate in that profile. Each
recording keeps its own finding, discriminated by bagtopic, and the text names the measuring
layer. An observed rate still wins, including zero. The C++ reader states timer callbacks
for `std::bind(&Class::method, this)` and inline lambdas, literal millisecond durations and
seconds durations read from a known numeric parameter member, and the publisher member with
the symbols that call its `publish`. These reuse the python row keys in section 2.2, so the
join uses the same period inference. A lambda symbol uses a content digest and no line.
New period values move the keys of the coded publisher edges that gain them, and finding
text changes move fingerprints. No release had shipped, so no stored id or waiver existed.
The report no longer says this rate rule needs the observed layer when the recorded layer
is present. Publish calls in another source of the same package and class also count, so a
header and its source keep one publisher history. Parameter expressions outside a direct
seconds value or a literal numerator over a member keep no period form, and chrono durations
with an explicit ratio remain unresolved.

Inside `1.31` a recorded bagtopic's `stamps` gained `offset_s`, the median of receive time
minus stamp over its stamped messages. `time.topic_skew` takes its reference from those
offsets with one vote per publishing node where the coded or declared layer names the one
node that publishes the topic, so a node with two late topics no longer outvotes a punctual
one and has the punctual topic blamed. A topic no node is known to publish keeps its own vote,
and a recording where no topic has a known publisher keeps the per topic result the stamps
check stored. The grouping is done by the rule, because the stamps collector reads no other
layer. The standalone `rosio stamps` command has no atlas and votes per topic. The stored
check results and the bag verdict are unchanged.

The tenth first run trial moved several findings inside `1.31` with no new key. The C++
reader writes the `parameter_reads` record block of section 2.2, which only the python reader
wrote, so `param.read_undeclared` reaches a C++ `get_parameter` it never saw; a C++
`get_parameter_or(name, default)` states that api where it stated `get_parameter`. A CMake
`foreach` over literal names is unrolled, so `add_executable(${node} src/${node}.cpp)` in such
a loop binds each executable to its source. A node class handing itself to a known wrapper
(`tf2_ros::TransformBroadcaster`) is still read whole, because the wrapper creates only what
its row states. `known_names.SERVICE_SCOPE` states the services an unread
`robot_state_publisher`, `joint_state_publisher`, `spawner`, `ros2_control_node` or component
container can serve, so `service.no_server` no longer abstains for a profile that starts one.
`controller.undefined` counts a `<name>.type` key a file the profile reads sets whether or not
it wins, is one finding per launch file and controller, and suggests only a controller sharing
a word of its name. An `rclcpp::QoS(10)` with only the modifiers the reader spells is the
default profile at that depth, a `message_filters::Subscriber` handed a QoS states it, and
`qos.incompatible` compares a wrapper end whose QoS the source states. The renamed variant of
`frame.not_in_urdf` also matches the same words in another order
(`camera_depth_optical_frame` beside `depth_camera_optical_frame`). `rosio init` writes a
commented profile for the other value of a simulation switch an entry point declares. No
release had shipped, so no stored id or waiver existed.

The eleventh first run trial moved several findings inside `1.31` with no new key. The python
reader follows a declare wrapper the way the C++ reader does: a module level function that
calls `declare_parameter` on a node and a name it is handed declares, at each call, the name
its caller passes, also from another module of the workspace, so `param.set_undeclared` no
longer fires on every key such a helper declares. Endpoints built inside a dict comprehension
over a literal sequence are read, where they were dropped without a blind spot. A frame spelled
as a parameter plus a literal (`self.prefix + 'odom'`) states `<role>_parameter_suffix`, so the
join adds the literal to the profile's value instead of replacing the whole frame with it, and a
`TransformStamped` built with `header=` and `child_frame_id=` keywords states its frames.
`frame.broadcast_twice` counts a broadcast whose parent a profile's parameter chain resolves.
No release had shipped, so no stored id or waiver existed.

* in `1.32`, the rule id `topic.namespace_escape`, an executable started in two namespaces
  whose code publishes one absolute name from every copy. No key and no id form changed.

Inside `1.32`, with no new key, the declared fact of a composable node whose `plugin=` is
written on a line below the start of its launch site carries a second evidence row on that
line, its locator the site's with `.plugin` appended, and the component case of
`plugin.unresolved` cites it, which is the line an engineer edits. It adds a row and moves
no id.

Inside `1.32`, with no new key, Autoware's build and handoff idioms are read.
`autoware_agnocast_wrapper_register_node` registers an executable per class the way
`rclcpp_components_register_node` does, so those executables bind their sources and their
remaps stop being blind spots. A target that lists `DIRECTORY src` holds the sources under that
directory, so a generated parameter library it links reaches them. A node class that hands
itself to a static method (`NodeParam::init(*this)`) or to a method of a field
(`smoother_.initialize(this)`) reaches that class, and another package a node reaches counts
the generated parameter libraries its sources link. Entities, edges and finding ids that
depended on those executables moved; a stored id of a finding about an unbound node stops
matching because the finding is gone.

Inside `1.32`, with no new key, a launch name built by `AnonName` is `runtime_randomised`:
the walk performed it, which appended the host name, the process id and a random number, so
the atlas held a node id that changed on every run and named the machine. The node is
synthetic now and its launch step is named by the stem `AnonName` was given. A free function
a node class hands its node to (`build_processors(node_ptr, names)`) hands it on to the
classes it builds, and is a `classes` row named `<function>()` holding only `node_handoffs`,
so the declarations of those classes reach the node. A node handed on through
`std::dynamic_pointer_cast<Self>(shared_from_this())` is still the node, and `node_ptr` is a
name a node handle goes by. No release had shipped, so no stored id or waiver existed.

* in `1.33`, the record key `parameter_writes` and the candidate mechanism `code_set` (section
  7), and the blind spot reason `set_in_code`.
* in `1.33`, the coded `publishes` and `subscribes` edge value `qos_overridden`, the QoS
  policies a parameters file overrides for that endpoint through
  `qos_overrides.<topic>.<publisher|subscription>.<policy>`. rclcpp and rclpy take such a key
  only when the endpoint was built with overriding options, which the reader does not follow,
  so the source value of each named policy is dropped from `qos` rather than stated as what
  runs, and `qos.incompatible` no longer fires on a policy the file changes.

Inside `1.33`, with no new key, four calls the readers did not know are read, found by
comparing the public API of the installed rclcpp and rclpy with what the readers spell. A
C++ `create_sub_node("ns")` handle prefixes its namespace onto every relative topic and
service name it creates, as `extend_name_with_sub_namespace` does, and one the reader cannot
follow leaves the name a blind spot. `get_parameters_by_prefix(prefix)` reads back every
parameter under the prefix in both readers, so `param.declared_unread` no longer fires on
them. C++ `waitForTransform` and python `lookup_transform_full`, `can_transform_full` and
the `_async` forms are transform reads, the `_full` forms taking the source from the third
argument. C++ `create_generic_client` is a service client whose type is the string second
argument. `create_generic_service`, which kilted added, is the server of the same
shape. No release had shipped, so no stored id or waiver existed.

Inside `1.33`, with no new key, two python reads moved after a generated workspace ran on
jazzy and its live graph disagreed with the atlas. An f-string that starts with a parameter
(`f"{prefix}/scan"`) states the literal rest as `parameter_suffix`, so the join appends it to
the value each profile gives the parameter, where it named the endpoint by the parameter
value alone; a parameter anywhere else in the string is the blind spot
`parameter_inside_name`. A bare integer depth states the reliable and volatile defaults rclpy
gives `QoSProfile(depth=n)`, as `rclcpp::QoS(n)` already did, which the running node reports.

Inside `1.33`, with no new key, the chain a process leaves unnamed follows under a name its
source states keeps only that process's launch rows. Every unnamed process of one namespace
followed a block keyed on that name, so the chain held the inline dictionary of each of them
and the file rows once per process, and the first process's value won for every node: a
generated launch file with six unnamed nodes in `/` handed `/coded_2` the dictionary of
`node_4`. A row an action states (`Node(x).parameters`, `Node(x).arguments`) stays only when
that action starts the process, and a file row is kept once.

Inside `1.33`, with no new key, the C++ generated workspaces of the conformance harness moved
four readings. A call on a sub node held in a member (`subnode_.get()->create_client(...)`)
takes the sub namespace, also when the member is made by a method the constructor's
initializer list calls, which is read after the call. `get_parameter_or(name, member_,
default)` hands `member_` the parameter, so a name built from it is the profile's value and no
longer the literal the member held before. A python helper handed the node that sets
`node.channel` and reads it back names that endpoint. `param.set_undeclared` reports a key of
a block launch never names whose node the join found in the source and which ignores it, where
the rule took that block for dead configuration.

Inside `1.33`, with no new key, a transform workspace generated and run on jazzy moved three
readings. A robot model a `robot_state_publisher` loads with a `frame_prefix` in its inline
parameters is stated in that profile's slot with the prefix on every link and joint, which is
what the node publishes; the `*` statement stays the file with its defaults. The `/tf_static`
publisher of a launch `static_transform_publisher` follows the first remap of `/tf_static`, as
every other endpoint does. A python node that fills `self.transform.header.frame_id` and
`self.transform.child_frame_id` in one method and sends `self.transform` from another, a timer
among them, states that pair where it was a `call_result` blind spot.

Inside `1.33`, with no new key, a pluginlib workspace generated and run on jazzy moved five
readings. A plugin reading a suffix of its own instance (`get_parameter(name + ".topic")`) made
a `reads_param` edge to a `pparam`, which the join and `param.read_undeclared` took for a node
parameter and stopped on. A selector list names its base class by the class it selects, from
the manifest that exports that lookup name, and `SELECTOR_BASE_CLASS` answers only for a name
no manifest of the roots exports, because a node of another stack keys its plugins on
`plugins` too and was read as costmap layers. A C++ file that registers several plugin classes
binds each class's suffixes to its own registration, where every suffix went to the first one
or became a `plugin_instance_prefix` blind spot. A suffix a selected instance's class declares
with a known default and no file sets is `<instance>.<suffix>` on the node, won by that
default, which is what the node runs with.

* in `1.34`, the finding key `edits`, the exact replacements a fix makes: rows of `file`,
  `root`, `line_hint`, `find` and `replace`. A reader applies a row only where the cited line
  holds `find` once as a whole word, so an edited or moved line is left alone; the language
  server offers the rows as one quick fix. `param.set_undeclared` states the rename of a key one
  slip from a declared name, and `frame.not_in_urdf` the frame name the model has. `rosio drift`
  prints the line to edit first, as `path:line`, and no longer lists a severity with no finding.

A reader pinned to `1.0` still loads a `1.34` file: `Atlas.from_json` compares the major
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
param.sim_time_mixed        launch.file_not_installed     model.mesh_missing
build.unbuilt               build.underlay                remap.matches_nothing
type.incompatible           service.no_server             launch.argument_undeclared
frame.broadcast_twice       param.read_undeclared         controller.undefined
controller.joint_missing     topic.namespace_escape
```

`launch.file_not_installed` is `error`. It fires once per file a launch file reads
from a package share directory that the install tree does not hold while the package source
holds the same package relative path, which is a build file missing its install line. The
parameter collector records it as the refusal `file_not_installed` with the launch site as
evidence, and `extract.input_refused` leaves that reason to this rule. The fix names the
`install(DIRECTORY ...)` line of `CMakeLists.txt` or the `data_files` entry of `setup.py`, by
the package's build type. A path absent from the source as well is a wrong path and stays a
refusal, and a package with no install tree is not built, so neither fires it.

`model.mesh_missing` fires once per robot model and mesh, never once per link, for a
`package://` mesh the model names and the workspace does not hold, and only for a model some
profile loads, which is a launch entity `file_reads` row with role `robot_model` at a live
site. rviz and gazebo draw nothing for those links and say so only in a log. It is `error`
when the package source holds the mesh at the same package relative path and the install tree
does not, read the way `launch.file_not_installed` reads it, and the fix names the same
install line by build type. It is `warn` when no tree holds the mesh, a wrong path or a file
never committed, and the fix names the filename in the model. A model no profile loads is not
reported, and a mesh that is not a package name stays the blind spot the urdf collector left.

`build.unbuilt` is `warn`. It fires once per atlas when launch files or robot models read
files from the share directory of a package no install root holds, the files are missing at
that path and the package source holds a file of the same name under another directory, which
is an install rule a build has not applied. It names each package, each file and where the
source holds it, its subject is the first such package by name, and the fix is `colcon build`
and the install tree as a root. A missing file the source holds nowhere stays
`extract.input_refused`, because a wrong path and a generated file read alike.

`build.underlay` is `warn`. It fires once for the workspace, naming every package the roots
hold whose share directory a launch walk looked up and the ament index answered from another
tree: the release a sourced setup file provides, for a package only a src root holds, or for
one an install root holds whose setup file is not sourced, or the build beside src that no
root names. The walk reads the workspace copy, as section 9 point 9 states, and the finding
names each package with the tree `ros2 launch` runs it from and the profiles, and says an
edit in the workspace takes effect only once it is built and sourced. Its fix names the setup
file to source, and an install tree beside src that no root names when there is one.

`remap.matches_nothing` is `warn`. It fires once per remap line whose source name no endpoint
of the node it is written on carries, in every profile that starts the node there, and only
when the join read that node's whole source and found no topic, service or action name in it
that the reader could not follow. It stays silent when the node states no endpoint at all,
when the node's code hands the node (`this`, `self`, a node interface) to a class or a free
function no source of the read set defines, or builds it on a python base another module
defines, when a package of the node holds a source the reader refused, when the name is a
sub topic of one of the node's endpoints (image_transport and cuda_blackboard add those) or
one every node or every lifecycle node has, when any source of the read set spells the name
as an endpoint, because a helper the join did not bind to the node may create it, and when a
source of the node spells the name, or a base of it, as any string literal. A remap scoped to
a node (`ctrl:status`) is read for the node it names and no other. It names the node's real endpoint names, the closest one
when one is near, and the line to edit. It is a rule of its own and not a case of
`remap.unresolved`: that rule is `info` and says nothing states the kind of a name, which is a
gap in what rosio read, while this one says the code was read and the rule is dead, which is a
defect in the launch file (a typo, or a name the node renamed). Folding the two would grade a
dead remap at `info` or every unread package at `warn`. A remap this rule reports is not also
`remap.unresolved`. The observed half is in `topic.no_publisher`, which names a remap that
moved the only publisher of a topic away from it.

`type.incompatible` is `error`. It compares the ends of one topic, service or action in one
layer and one profile, and fires when a producing end (a publisher or a server) and a
consuming end of another node state two types, both spelled with a package. The type the
producers state most is the reference, and the finding names the line of the odd end. A node
that states two types on one name holds alternatives a parameter chooses between and is left
out, and so is an end under a condition. One line read under several namespaces is one
finding. It is a rule of its own and not a case of `type.differ`, for the reason
`qos.incompatible` is not a case of `qos.differ`: `type.differ` is two layers disagreeing about
one endpoint, a gap between what was written and what runs, while this is two endpoints that
agree with their own layers and can never connect, which is a defect at `error`.

`service.no_server` is `warn`. It fires when a node calls a service or an action that no node
of the profile serves, and only when the caller's source was read whole the way
`remap.matches_nothing` reads it, no node of the profile holds a name of the same kind the
reader could not follow (an action name nobody followed makes only services under `_action/`,
so it hides no plain service), every declared node in that profile was read whole, no remap target
ends in the name, and a node whose whole source was
read serves another name of the same type. It names that name, in the caller's namespace
first and then the closest by spelling, because a client of a name nobody serves is nearly
always a typo or a rename. A service with no server of its type anywhere stays a question of
coverage, and the graph keeps saying no server was found. It is not a case of
`topic.no_publisher`, which asks the observed layer and fires on a topic, while this one reads
the code and fires on a service or action name.

`launch.argument_undeclared` is `warn`. It fires once per including file and argument name
when an include passes an argument that no file of the included tree declares or reads, and
names the closest argument the tree declares that the include does not already pass. The walk
decides it only when every file below the include resolved, raised nothing and reads its
configurations by literal names, because a configuration read by a built name could be the
argument. Parameter files read with substitutions enabled join the read texts, and the rule
is undecided when their paths do not resolve or their files cannot be opened.
An import outside the standard library and `launch`, `launch_ros`, `launch_xml`, `launch_yaml`
and `ament_index_python` makes the rule undecided unless its module text can be read beside
the launch file or on the walk's python path.
Jazzy `IncludeLaunchDescription.execute` returns unscoped `SetLaunchConfiguration` actions,
so a passed argument is also visible to later actions of the including description, and the
rule also requires that no later action of the including file reads the name.
ROS 2 accepts such an argument without a word and the included file runs on its own
default, so the value goes nowhere. It is not `launch.raised`: nothing raises.
An argument a launch line hands `xacro` for a robot model is the same question: the
`file_reads` row of the model carries `args_unread` when no file of the model or its
includes declares or reads the name, and nothing is decided when an include does not
resolve.

`frame.broadcast_twice` is `warn`. It fires once per child frame and profile that two nodes
broadcast, from declared or coded facts that state the child as a literal and a parent, a
literal or a parameter the profile resolves, and sit under no condition. tf2 holds one transform per child, so the frame takes whichever arrived last, which
is what two robots without a frame prefix, or one frame published with two parents, look like.
Two sites of one node are one broadcaster (section 5 cites one node broadcasting a pair from
two methods). Two nodes whose remaps send `/tf` and `/tf_static` to different topics write
two trees, the way two robots in their own namespaces do, so one child frame in each is no
clash. It is not `frame.parent_differs`, which compares the robot models of the
profiles with each other.

`topic.namespace_escape` is `warn`. It fires once per profile, topic and executable when one
executable the launch starts in two or more namespaces publishes a name its code spells with a
leading slash, so every copy writes one topic: two robots on one `/battery_state`, which no
subscriber can tell apart. Only a coded publisher with no remap and no condition counts, and the
names a library shares on purpose (`/tf`, `/tf_static`, `/rosout`, `/diagnostics`,
`/parameter_events`, `/clock`) are left out. It names the line and asks to drop the slash or
remap the name per namespace. It is not `type.incompatible` or `qos.incompatible`, which are two
ends that cannot connect, while here the ends connect and should not.

`param.read_undeclared` is `warn`. It fires once per parameter of a node whose code reads it
with `get_parameter(name)`, in rclcpp or rclpy, when no code the node runs declares the name.
Both libraries raise `ParameterNotDeclaredException` on that read, so the node stops the first
time the line runs, whatever a file sets. It is decided the way `param.set_undeclared` decides
a key (`join.reading`), with the read itself not counted as a declaration: silent on a node
that takes undeclared parameters, on one whose code was not read whole, where a name built at
run time could be this one, and on `get_parameter_or` and the out parameter form, which answer
a default. It names a declared name close to the one read. It is not `param.set_undeclared`,
which is a file setting a value nobody reads, a line that does nothing, while this one is code
that crashes.

`controller.undefined` is `error`. It fires once per spawner and controller name when a
`controller_manager` `spawner` asks the profile's one `ros2_control_node` for a controller no
parameters file the manager is handed defines by a `<name>.type` key, so the spawner fails and
the controller never runs. It names a defined controller close to the one asked for. It is
silent where the profile starts no manager or several, where the manager's configuration
defines no controller at all (an unread or unbuilt file), and on a spawner that hands the
controller its own type or parameters file (`-t`, `-p`).

`controller.joint_missing` is `error`. It fires once per controller parameter and joint when a
controller the manager defines lists, under `joints` or `joint`, a joint the one robot model
the profile loads does not declare, so the hardware exports no interface for it and the
controller fails to configure. The value is the candidate ranked first in the profile, because
the manager hands the block to a controller no launch line names and no candidate wins. It is
silent unless the profile starts one manager and loads one model whose joints were read.

A composable node whose `plugin=` names a class that no `rclcpp_components` registration or
`RCLCPP_COMPONENTS_REGISTER_NODE` of the workspace names is a case of `plugin.unresolved` and
not a rule of its own: it is the same question (a class name nothing provides) asked of the
component registry rather than of a pluginlib manifest. It is decided only for a package whose
C++ sources were all read, leaves the blind spot `component_class_unregistered`, is about the
launch file, and names the closest registered class.

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

A rule that raises inside the engine, or produces two findings under one id, writes no
finding and is named in `counters.drift.failed` instead. `rosio drift` prints that list and
grades it as `error` under every `--fail-on` but `never`: whatever the rule would have found
is missing, and a gate that passed over it would read exactly like a clean stack. It is not a
rule id, because no rule can report that it did not run.

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
* **A finding that groups members.** The remaps one file writes on one node, or the keys one
  launch action hands its nodes, are one finding named by the place and not by its members:
  a member added beside them keeps the id and moves the fingerprint. A waiver keyed on the id
  accepts the place, and one keyed on the fingerprint accepts the members it was written
  against and stops matching when another arrives.
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
| `rosio/drift/controllers.py` | M3 | ros2_control: spawned controllers no configuration defines, controller joints the model lacks |
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
| `rosio/serve/tools.py` | M6 | the twelve tools as plain functions, eleven over a loaded atlas and `migrate` over the sources, no mcp involved |
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

   A site in a file no root holds is still a claim. A node or an include that a launch file
   from the ROS installation states (turtlebot3 starts every nav2 node through
   `nav2_bringup`) cites the deepest include on its action trail whose line a root holds,
   which is the line an engineer edits, and its locator is the trail, which names every
   outside file on the way. Failing to cite it once dropped every node the walk started.

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
