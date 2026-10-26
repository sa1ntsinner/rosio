# rosio-lsp

`rosio-lsp` reads a rosio atlas and serves diagnostics, Markdown hover, definitions,
references, parameter inlay hints and workspace symbols over LSP stdio. Extraction remains
in the Python engine.
It starts without an atlas and keeps the last usable atlas if a reload fails.

Run `cargo run -p rosio-lsp --release`, or build with `cargo build -p rosio-lsp --release`
and configure your editor to launch `target/release/rosio-lsp` (on Windows, add `.exe`).
The first workspace folder is used, with `rootUri` as a fallback.

## Initialization options

| Option | Default | Meaning |
| --- | --- | --- |
| `atlasPath` | `rosio-out/atlas.json` | Atlas path relative to the workspace folder |
| `roots` | `{}` | Root label to local directory overrides, absolute or workspace relative |
| `extractCommand` | `["rosio", "extract", "."]` | Program and arguments, executed without a shell in the workspace folder |
| `codeLens` | `true` | Show endpoint peer counts and QoS incompatibilities |
| `extractOnSave` | `false` | Extract when a cited file is saved, debounced by 300 ms |

Root overrides take precedence over `generated.roots[].path`. Package locations come
from `generated.packages`. For older atlases without this map, package directories are
found through `package.xml` under the configured roots. `@repo` paths use the workspace
folder. Locations use zero based lines. Locator searches use the first whole word match
and are marked approximate, including a line zero fallback when no symbol matches.

The server registers an atlas watcher when the client supports dynamic watched file
registration. Clients can also send `workspace/didChangeWatchedFiles` directly.
`workspace/executeCommand` accepts `rosio-lsp.reload` and `rosio-lsp.extract`, named apart from any command a client registers for itself. Extraction runs on
one worker, reports progress and logs stderr. Requests received during extraction queue
one rerun. Standard output is reserved for LSP messages.

Parameter inlay hints show which values win, where overrides come from, and inactive
configuration. Each hint includes the candidate chain for its profile. Atlas reloads
refresh hints when the client supports `workspace/inlayHint/refresh`.

## Requests

| Method | Parameters | Result |
| --- | --- | --- |
| `textDocument/codeLens` | `{textDocument}` | Endpoint peers and QoS warnings, one per line; opens the first endpoint card in edge id order |
| `textDocument/inlayHint` | `{textDocument, range}` | Hints at candidate source line ends, one per line and profile |

### Custom requests

| Method | Parameters | Result |
| --- | --- | --- |
| `rosio/summary` | `{}` | `{schemaVersion, counters, layers, profiles, roots, findings: {error, warn, info}, atlasPath, loaded, error?}` |
| `rosio/findings` | `{minSeverity?}` | `[{id, rule, severity, title, say, fix, locations: [{uri, line}]}]` in atlas order |
| `rosio/entity` | `{id}` | Raw entity with `sites: [{uri, line, approximate}]` and `findings: [ids]`, or `null` |
| `rosio/search` | `{query, kinds?, limit?}` | `[{id, kind, label, layers}]`, case insensitive, exact label then prefix then contains |
| `rosio/children` | `{kind?, profile?}` | Object mapping kind to sorted entity ids, using `indexes.by_kind` when present |
| `rosio/launchTree` | `{profile?}` | Launch profiles with nested launch entities and source locations |
| `rosio/frameTree` | `{profile?}` | TF forest with geometry, broadcasters, listeners and findings |
| `rosio/paramTree` | `{profile?}` | Parameters by owning node, with the winning value and the candidate chain |
| `rosio/graph` | `{profile?, focus?, depth?}` | Nodes, topics, services and actions with the endpoint edges between them |
| `rosio/robotModel` | `{model?, profile?}` | One urdf or xacro model: links, joints, roots and the TF broadcasts into it |
| `rosio/unresolved` | `{}` | `[{id, want, expr, reason, owner, resolvableBy, locations: [{uri, line}]}]` in atlas order, `owner` as the owning entity's label, or its id when no such entity exists |
| `rosio/mesh` | `{file}` | `{file, format, triangles, truncated, triangleCount, bounds}` or `{file, format, refused}` |

`minSeverity` is `error`, `warn` or `info` (default `info`), and includes all more severe
findings. Search defaults to 100 results. Profile filtering accepts a slug or profile id.
Without a loaded atlas, collection requests return empty results, entity and hover return
`null`, and summary includes a short error. Failed reloads retain the previous answers
and add the latest error to summary. Schema major 1 is supported, including newer minors.

### Launch and frame trees

`rosio/launchTree` returns an array of profile rows:

```text
[{id, label, command, entries: [
  {id, label, arg_declarations, args, location?, children: [
    {id, kind, label, nodes: [{id, label}], package, executable, namespace,
     conditions, gates, parameter_inputs, remaps, location?, children: [...]}
  ]}
]}]
```

`entries` contains the profile's entry launch files. Each file's `contains` edges supply
its entities. Action segments in the entity id determine nesting under the nearest
existing ancestor. Includes with the same basename in different packages carry a content
discriminator while existing descendant ids stay unchanged. Their original `entity_path`
and `included_file` facts, matched against descendant evidence, resolve the parent when
that match is unique. Labels use the decoded final action segment. `kind` is `include`,
`node`, `composable`, `container`, `group` or `other`. Outgoing `launches` edges supply
`nodes`. `package` comes from the fact's `pkg`. Arrays retain their atlas values, including
parameter input and remap order. Missing scalar values are `null`, and missing arrays are
empty. `command` uses the profile's stated command, or its entry point descriptions joined
by newlines, and is `null` when neither exists. It is display text, not a shell invocation.

`rosio/frameTree` returns an array of root frame rows:

```text
[{id, label, parent?, joint, joint_type, model, xyz?, rpy?,
  broadcasters: [{id, label, location?}], listeners: [{id, label, location?}],
  findings: [finding ids], location?, children: [...]}]
```

A `tf_parent` edge points from child to parent. Its values supply `joint`, `joint_type`,
`model`, `xyz` and `rpy`. Unknown joint fields are `null`; absent geometry is omitted.
Joint evidence supplies the child location, with frame evidence as fallback. Broadcaster
and listener locations use edge evidence, then node evidence. Nodes are deduplicated by id.
Frames without a parent remain roots, including isolated frames used only by lookups.
`findings` names findings directly about the frame, filtered to the requested profile.
Alternative parent edges retain separate child rows. Cycles stop before repeating a frame
on the same path, and components with no natural root start at their first frame id.

`rosio/paramTree` returns `{profile, nodes: [{id, label, params: [...]}]}`. A parameter row is
`{id, name, state, layers, findings, value?, valueType?, source?, candidates}`, and each candidate
is `{rank, wins, value, valueType, mechanism, shadowKind, source}`. A `source` is `{file, locator,
at?}`, with `file` as the package path and `at` as `{uri, line, approximate}`. `value` is absent
when no candidate wins in the profile. Without a profile, candidates of every profile are listed
and the first profile's effective value is shown.

`rosio/graph` returns the system graph:

```text
{profile, truncated,
 nodes: [{id, label, kind: "node", layers, findings, nameSource, at?}],
 channels: [{id, label, kind: "topic" | "service" | "action", type?, layers, findings, at?}],
 links: [{id, src, dst, kind: "publishes" | "subscribes" | "serves" | "calls", layers,
          agreement, conflicts, qos?, wrapper, rawName?, at?}]}
```

Every link points from the node to the channel, whatever its kind: a `subscribes` link has
the subscribing node as `src` and the topic as `dst`, as the atlas stores it, so a view draws
data flow by reversing it. A node or channel is kept when its `profiles` or its facts name the
profile or `*`, and a link when it is in the profile and both ends are kept. Synthetic node ids
are returned as they are; `label` is always the entity label. `findings` counts findings about
the entity in the profile. `type` is set when the channel's `uses_type` edges name exactly one
type. `qos` holds `reliability`, `durability`, `history`, `depth` and `preset` from the link's
effective values in the profile, each only when stated, and is absent when none is. `wrapper`
is true when a fact in the profile or the edge attributes mark it `via_known_wrapper`.
`rawName` is the name the source spelled. `at` is `{uri, line, approximate}` from the first
evidence row of the profile's facts that maps to a local file. With `focus`, only entities
within `depth` hops (default 1) of that id over the kept links are returned, so depth 1 from a
node is its channels and depth 2 adds their other nodes; an unknown focus returns empty lists.
At most 400 nodes and channels are returned. Beyond that `truncated` is true and the kept set
is the focus first, then entities with findings, then the rest, each in id order. Rows are
sorted by id within each list.

`rosio/robotModel` returns one robot model and the list to choose from:

```text
{profile, model, truncated, limit,
 models: [{id, label, robotName, strategy, profiles, missingMeshes?, at?}],
 links: [{id, label, layers, findings, worst?, at?}],
 joints: [{id, name, type, parent, child, xyz?, rpy?, axis?, limit?, mimic?, unreadable?, at?}],
 roots: [frame ids],
 broadcasts: [{id, node, nodeLabel, child, parent?, parentName?, layers, condition?, static?, at?}]}
```

A model is a `file` entity the urdf collector wrote, so `model` is a file id. `profiles` are the
profiles whose launch entities read the file with role `robot_model`, plus any profile a urdf
fact of the model is stated for. Without `model`, the first model a profile loads (the requested
profile when one is given) is chosen, else the first model; an unknown `model` returns `model:
null` and empty lists. `links` are the frames whose `urdf_models` name the model, and `at` is the
frame's line in that file. `joints` are the `tf_parent` edges whose values name the model, one
per joint name; `parent` and `child` are frame ids and the geometry keys are copied only when
the atlas states them, so an unreadable origin is absent and never zero. `roots` are the links
no joint has as child. `broadcasts` are the `broadcasts_tf` edges in the profile whose child or
stated parent is a link of the model; `parentName` is the parent as the source states it and
`parent` its frame id when that frame exists. `worst` is the most severe finding about the
frame in the profile. Model `at` is the start of the file. At most 1000 links and 1000
broadcasts are returned, in id order; beyond that `truncated` is true. Joints are sorted by
name and every other list by id.

`rosio/mesh` reads the mesh a file id names, such as the `file` of a mesh row in a link's
geometry, resolved the way evidence rows are. Binary and ASCII STL and Wavefront OBJ are read,
OBJ polygons as fans from their first corner. `format` is the lowercase extension.
`triangles` is flat `x y z` per corner, nine numbers per triangle, in the mesh's own units.
Beyond 20000 triangles the mesh is decimated and `truncated` is true: every corner snaps to a
uniform grid of cubic cells over the bounds and moves to the mean of the corners in its cell,
triangles with two corners in one cell and repeats of a kept triangle are dropped, and the rest
keep their orientation and file order. The grid starts at 1024 cells along the longest side and
coarsens by a fifth per pass until at most 20000 triangles remain, so one file always gives the
same answer. `triangleCount` stays the true count and `bounds` covers every original triangle,
and is `null` for a mesh with none. `refused` is `file_missing` when the file id resolves to no local file,
`format_not_read` for any other format (Collada included) and `mesh_unreadable` for a file
that is cut short or does not parse.

Both tree requests accept a profile slug or id. Launch results always separate profiles. Without
`profile`, frame results include all profiles. Facts are restricted to the requested profile
and `*`; specific facts supply fields before wildcard facts. Repeated fields use the first
profile and layer in lexical order. Locations use the same filtered facts and zero based
lines. Optional locations are omitted when their evidence cannot be mapped to a local URI.
Profile rows, roots, children, nodes and finding ids are sorted by id; parallel TF edges
with the same child retain edge id order. Both requests return `[]` without a loaded atlas.

Run `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings` and `cargo test`.

A rosio diagnostic offers one code action, `Waive this finding in rosio-waivers.yaml`, which
appends the finding id with a reason to fill in, creating the file at the workspace root when
there is none. `rosio drift --waivers rosio-waivers.yaml` then stops grading it.

The server sends `rosio/atlasChanged` after every successful load, including an atlas
without findings. Its payload has `atlasPath`, `counters` and `findings` counts by severity,
plus the fields of `rosio/summary`. Failed reloads do not send it. Diagnostics retain their
own publication path. Clients should refresh atlas views on this notification.

Endpoint peers require usable facts in the same profile. A `*` fact applies in every
profile. Lenses name each profile when its peer set differs. Waiver actions accept block
list headers with comments and disable unsafe edits with a reason, including flow lists,
anchors and non-list values. Entity cards show a ledger per profile, with `*` as fallback,
and identify each parameter candidate's profile.

## Release migration

Execute `rosio-lsp.migratePlan` with `arguments: [{"from":"humble","to":"lyrical"}]`.
`from` is optional and uses `ROS_DISTRO` from the configured engine environment. The
engine command is the prefix of `extractCommand` before `extract`. The server runs
`migrate plan . --json` in the workspace and resolves returned paths against the git
root, or the workspace when there is no git root. Engine wrappers can source ROS.

`rosio/migrationPlan` carries `{from, to, summary, changes}` after each successful plan.
Migration diagnostics use source `rosio migrate` and coexist with atlas findings.
Full document synchronization removes stale matches after edits. Quick fixes use exact
UTF-16 ranges, and `source.fixAll.rosioMigration` applies automatic changes in one file.
`rosio-lsp.migrateClear` clears the plan, diagnostics and sends a null plan notification.
`rosio-lsp.migrateApply` accepts the same arguments, runs apply without `--force`, then
plans again. Its response also includes `applied.diff`, the engine's diff path.
Newer requests supersede older plan results. Engine failures retain the previous plan.
`rosio-lsp.migrateVerify` accepts the same arguments and answers with the data of
`migrate verify --json`, including when the build failed. It leaves the plan as it is.
