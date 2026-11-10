# rosio architecture

rosio reads a ROS 2 workspace, a live system and a recording, and writes one atlas plus
the drift between its layers. The identity and schema contract is in AGENTS.md, which every
module obeys. This file is the map: what each module is, how data moves, and why no language
model appears anywhere in the core.

Every module of the package is in the table below, and nothing that is not a module of the
package is. That is not kept by hand: `tests/test_architecture.py` walks `rosio/`, reads
the module names out of this file, and fails when the two disagree in either direction. A
tool whose whole purpose is reporting that a written description and a real system have
drifted apart cannot ship a written description that has drifted from its own tree.

## Shape in one paragraph

Collectors are independent producers. Each one reads one kind of input and emits claims into
an `AtlasBuilder` as facts keyed by `(layer, profile)`, edges keyed by a producing site, and
explicit entries for what it could not resolve. Nothing is merged while collecting. One join
phase runs afterwards, resolves synthetic node identities to real names, and recomputes every
derived answer. Drift rules then read the joined atlas and emit findings. Renderers, the
query surface and the agent tools read the same atlas and never re extract. The atlas is the
only interface between those halves, so any of them can be replaced without touching the
others.

## Modules

### Core, milestone M0

| module | one line |
| --- | --- |
| `rosio/__init__.py` | the version and the layer name tuple |
| `rosio/__main__.py` | `python -m rosio`, the same entry point as the command |
| `rosio/errors.py` | the exception hierarchy and the process exit code each class maps to |
| `rosio/ids.py` | every canonical id: build, parse, escape, plus the one ROS name and parameter name normaliser |
| `rosio/model.py` | the atlas dataclasses, the builder that enforces the contract, and the pinned deterministic serialisation, which is the only place the json encoder is reached |
| `rosio/schema.py` | the atlas JSON Schema as a python dict, and a hand written validator with no third party dependency |
| `rosio/cli.py` | argparse entry point, one subcommand per verb, no CLI framework |
| `rosio/scaffold.py` | `rosio init`: the fallback configuration written out as a `rosio.yaml` to edit |
| `rosio/bundle.py` | portable archives with an atlas, reports, cited files and revision evidence |

### Release migration

`rosio migrate` reads the workspace's source packages and two data files, never an atlas.
`data/distros.json` is measured by `corpus/distros/collect.py` from rosdistro and the official
`ros:<release>-ros-base` images; `data/rules.yaml` is written by hand, one source per rule.

| module | one line |
| --- | --- |
| `rosio/migrate/__init__.py` | the source packages of a discovered workspace, `plan`, and `apply`, which rewrites exact changes only and returns the diff |
| `rosio/migrate/distros.py` | release order and what each release ships: released packages, installed headers, toolchain versions |
| `rosio/migrate/rules.py` | the hand written rules, successors and removed python modules, refused at load when a source or a release is missing |
| `rosio/migrate/scan.py` | one matcher per file kind: package.xml tags, CMake invocations, includes and identifiers with comments blanked, python imports through `ast` |
| `rosio/migrate/cmake.py` | CMake command invocations with exact argument spans, and plain `set()` lists |
| `rosio/migrate/report.py` | the plan as lines for a person and as data for an agent or an editor |
| `rosio/migrate/verify.py` | build copied sources in a target release container, parse package diagnostics and match their locations to the plan |

### Static layers, milestone M1

| module | one line |
| --- | --- |
| `rosio/extract.py` | the orchestration behind `rosio extract`: the committed collector order, one run per layer per profile, one pass row per collector, recompute once, validate, write atlas.json |
| `rosio/config.py` | run configuration: which roots, which profiles, which layers, which collectors are opted into |
| `rosio/workspace.py` | package discovery, ament index and install tree lookups, and a content hash per root that survives a move of the checkout |
| `rosio/collect/__init__.py` | the collector registry and the context every collector is handed, so a new collector is one entry |
| `rosio/collect/launch_declared.py` | launch files, executed in a sandbox because a static parse gets a wrong answer on real launch code, not a partial one |
| `rosio/collect/params.py` | yaml parameter profiles, the five key shapes, and the ordered candidate chain per parameter |
| `rosio/collect/urdf.py` | urdf and xacro: links, joints, frames, sensor frame names, and gazebo plugin remaps |
| `rosio/collect/interfaces.py` | msg, srv and action definitions, read from the rosidl argument list rather than a filesystem glob |
| `rosio/collect/plugins.py` | pluginlib manifests, unioning the package.xml exports with the CMake exports, and the executable to source mapping the join needs |
| `rosio/collect/entrypoints.py` | the non launch composers: desktop launchers and scripts that build `ros2 launch` and `ros2 run` command strings |
| `rosio/resolve/__init__.py` | the resolve package surface, holding no atlas id anywhere under it |
| `rosio/resolve/records.py` | the intermediate records a source produces before identity is decided, so the launch resolver and the parameter resolver share one contract |
| `rosio/resolve/substitutions.py` | substitution evaluation, needed because remap targets, timer periods and argv elements are substitutions |
| `rosio/resolve/launch_exec.py` | sandboxed `generate_launch_description`, OpaqueFunction performance, event handler payloads, per file exception capture |
| `rosio/resolve/_launch_child.py` | the child process that imports launch, builds the description and walks it, talking to the parent over one json object each way |
| `rosio/resolve/yaml_lines.py` | yaml loaded with the line of every key kept, so a parameter candidate can cite the line it was written on |
| `rosio/resolve/rewritten_yaml.py` | recognition of the nav2_common RewrittenYaml and ParameterFile behaviour, by duck typing, including that it rewrites only keys that already exist |
| `rosio/resolve/py_pipeline.py` | the parameter document a launcher assembles in python and writes to a temp file, which is what the running system is configured by and what no repository holds |
| `rosio/resolve/merge.py` | the one deep merge rule, recording which merge site produced a value |
| `rosio/resolve/ros_args.py` | the argv parser for `--ros-args`, `-p`, `-r`, `--params-file`, `__node` and `__ns` |
| `rosio/resolve/qos_presets.py` | what a QoS written as a preset or a bare depth means, read from the client libraries, so a rule can compare reliability and durability across forms |
| `rosio/resolve/known_names.py` | the committed node name table for third party executables whose source is not in the workspace |
| `rosio/resolve/identity.py` | role content, role digest and the one node id rule every collector asks, so an unnamed node cannot become two entities with two halves of its edge set |

### Code layer, milestone M2

| module | one line |
| --- | --- |
| `rosio/collect/py_code.py` | rclpy sources read with the stdlib ast: endpoints, parameter declarations and reads, TF, node names, with one function wide dataflow pass |
| `rosio/collect/cpp_code.py` | C++ sources, tree-sitter behind the `cpp` extra, with the same dataflow pass for QoS variables and parameter derived names |
| `rosio/resolve/wrappers.py` | the known wrapper table, so a publisher hidden inside a library helper is not reported as an observed only endpoint |
| `rosio/join.py` | the single join phase: identity resolution, supersession, then `recompute`, idempotent by contract |

### Drift, milestone M3

| module | one line |
| --- | --- |
| `rosio/drift/__init__.py` | the drift package surface, re exporting the rule registry |
| `rosio/drift/rules.py` | the closed rule vocabulary, the deprecation alias table that keeps waivers valid, and the severity per rule |
| `rosio/drift/engine.py` | run order, isolation of a rule that raises, vocabulary check, deduplication by fingerprint, waivers, filtering and the counters |
| `rosio/drift/params.py` | shadowed, restated identical, set but never declared, declared but never read, orphan blocks |
| `rosio/drift/time.py` | recorded backward steps, clock offsets, skew, frozen and zero stamps, gaps and drift |
| `rosio/drift/topics.py` | publishers with no subscriber, subscribers with no publisher, QoS and type disagreement, rate below the declared expectation, observed only endpoints |
| `rosio/drift/nodes.py` | node identity: declared and not observed, observed only, ambiguous, regressed, unnamed, plus the profile scoping the graph rules share |
| `rosio/drift/middleware.py` | observed middleware groups and missing Zenoh routers, scoped by profile |
| `rosio/drift/frames.py` | frames with no broadcaster, orphan roots, config frames absent from every urdf, parent geometry disagreement, remaps and lifecycle |
| `rosio/drift/plugins.py` | what a selector list loads, what it leaves behind, and what it names that no manifest exports, which is what gates the parameter family |
| `rosio/drift/controllers.py` | ros2_control: a spawned controller no configuration defines, a controller joint the robot model lacks |
| `rosio/drift/build.py` | build staleness, unloadable launch files, resolution rate below budget, and the completeness of the extraction itself |
| `rosio/query.py` | the small context surface behind the CLI slices: explain, param, query, path, diff, unresolved, drift |

### Output, milestone M4

| module | one line |
| --- | --- |
| `rosio/render/__init__.py` | the render package surface, pure functions of a loaded atlas |
| `rosio/render/report.py` | ATLAS_REPORT.md, what the system is |
| `rosio/render/drift_md.py` | DRIFT.md, findings grouped by severity and rule family, each with its quotable sentence |
| `rosio/render/html.py` | a single file atlas.html, findings first, then per entity panes showing written, built and running side by side with only conflicting fields tinted, everything inlined so nothing is fetched at run time |

### Runtime layers, milestone M5

| module | one line |
| --- | --- |
| `rosio/collect/probe.py` | a short rclpy probe: nodes, endpoints with QoS, effective parameters, TF, rates, lifecycle state |
| `rosio/collect/_probe_child.py` | the child that imports rclpy, one request over stdin, so a stuck discovery costs one probe and not the run |
| `rosio/collect/proc.py` | the process table half of the observed layer: argv, parameters files, remaps, node name assignments, middleware and router presence |
| `rosio/collect/stamps.py` | recorded header stamp checks and clock verdicts |
| `rosio/collect/bag.py` | rosbag2 metadata and per topic statistics, into the reserved bag entity kinds |

### Header stamp checks

| module | one line |
| --- | --- |
| `rosio/stamps/__init__.py` | the public stamp check helpers and thresholds |
| `rosio/stamps/__main__.py` | the standalone header stamp command entry point |
| `rosio/stamps/checks.py` | the registry of stream and clock checks |
| `rosio/stamps/cli.py` | the stamps command arguments and rendered verdict |
| `rosio/stamps/clock.py` | epoch offsets, clock drift and skew between topics |
| `rosio/stamps/fallback.py` | tolerant readers for recordings with damaged definitions |
| `rosio/stamps/model.py` | stamp series and individual check result records |
| `rosio/stamps/mutate.py` | controlled timestamp mutations for recorded fixtures |
| `rosio/stamps/read.py` | header stamp series and reader degradation notices |
| `rosio/stamps/stream.py` | backward steps, frozen and zero stamps, gaps and rates |
| `rosio/stamps/verdict.py` | clock diagnosis from the combined check results |

### Agent integration, milestone M6

The agent surface is two packages, not one module. `rosio/agent` writes files and imports
nothing optional, `rosio/serve` answers questions and is the only place the `mcp` extra is
reached. Nothing in `rosio/agent` may import `rosio/serve` at module import time, or an
install fails on a machine that never wanted the server.

| module | one line |
| --- | --- |
| `rosio/agent/__init__.py` | the agent package surface, which resolves `serve` lazily so an install never pulls in `mcp` |
| `rosio/agent/platforms.py` | the table of assistants, one row per file, with the verified flag |
| `rosio/agent/install.py` | the install and uninstall writers, idempotent, with no silent overwrite |
| `rosio/agent/coverage.py` | the Claude Code Stop hook: a diff review against coding.md, or a nudge toward unopened source files |
| `rosio/serve/__init__.py` | the serve package surface |
| `rosio/serve/tools.py` | the thirteen tools as plain functions, eleven over a loaded atlas, `migrate` over the sources and `check`, which extracts the served workspace again, no mcp involved, every answer capped and every refusal capped with it |
| `rosio/serve/server.py` | the mcp server, both transports, the key check, and the one place each optional range is written |
| `rosio/serve/__main__.py` | `python -m rosio.serve`, the same options as `rosio serve` |

The four files the installers copy into a user's repository are data and not modules, and
they live in `rosio/agent/templates/`.

### Staying current, milestone M7

| module | one line |
| --- | --- |
| `rosio/history.py` | the git hooks, the rebuild they run, the build cache behind it and the two revision mode of `diff` |
| `rosio/mergetool.py` | the `atlas.json` merge driver, union with every disagreement stated |

## Data flow

```
workspace src        launch, yaml, urdf, msg, pluginlib, entry points
      |                              |
      |                              v
      |            collect/{launch_declared,params,urdf,interfaces,plugins,entrypoints}
      |                    over resolve/*            (declared) ---+
      |                                                            |
C++ and python sources -> collect/{py_code,cpp_code}   (coded) -----+
                                                                   |
live system  -> collect/{probe,proc}                (observed) ----+--> AtlasBuilder
                                                                   |     facts keyed by
rosbag2 dir  -> collect/{bag,stamps}                (recorded) ----+     (layer, profile)
                                                                         edges keyed by
                                                                         producing site
                                                                         unresolved entries
                                                              |
                                                              v
                                                        join.py  (identity, supersession)
                                                              |
                                                              v
                                                     builder.recompute()
                                             conflicts, agreement, effective, counters, indexes
                                                              |
                                                              v
                                                         atlas.json
                                                              |
                    +----------------+----------------+-------+--------+----------------+
                    v                v                v                v                v
              drift/* rules      render/*         query.py        serve/tools.py    history.py,
                    |                |                |                |            mergetool.py
                    v                v                v                v                v
            findings in         ATLAS_REPORT.md,  rosio get,   the thirteen tools rebuild on a
            atlas.json          DRIFT.md,         param, diff,   an assistant       hook, merge
                                atlas.html        unresolved     calls              two atlases
```

Three properties hold across the whole flow.

* Collectors never read each other. Running them in a different order gives the same atlas.
* The join phase is the only place that changes an identity, and it is a fixpoint.
* Everything after the join is a pure read. DRIFT.md, atlas.html, every CLI answer and every
  answer the server gives are renderings of atlas.json with no re extraction, so a finding
  shown to a human and a finding read by an agent can never disagree.

## Why the core is free of language models

1. **The output is evidence, and evidence must be reproducible.** Every fact cites a file, a
   locator and a root. Two runs over unchanged inputs produce identical canonical bytes, and
   CI asserts it. A sampled model breaks that on the first run.
2. **Drift is a comparison, not a judgement.** The whole value is that a reported mismatch is
   a fact about two layers of a real system. A guessed mismatch is worse than no report,
   because a report that is wrong once stops being read.
3. **The consumer is already a model.** rosio exists to give a coding agent a ground truth
   it cannot get by reading files one at a time. Putting a model inside the extractor would
   hand the agent its own uncertainty back, laundered as data.
4. **It must run where the robot is.** On a Jetson, in CI, in a container with no network and
   no key. A deterministic extractor with pyyaml as its only core dependency runs
   everywhere, offline, in seconds, at no cost per run.
5. **Explicit ignorance beats a plausible guess.** Unresolved sites are counted and gated on,
   so the honest answer is measurable and improvable. A model would fill those gaps with
   something that looks right, which removes both the measurement and the pressure to fix
   the extractor.

Language models are welcome around the edges, outside the core: writing a summary from
atlas.json, proposing a fix for a finding, or driving the CLI. None of that is allowed to
write into the atlas.
