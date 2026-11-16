# Changelog

## 0.2.1

Atlas schema `1.35`, additive: a reader of a `1.34` file loads it.

### Continuous integration

* `rosio drift --format github` prints one workflow command per finding, so GitHub pins each
  one to its line of the pull request. `--format sarif` writes SARIF 2.1.0 for code scanning,
  with the finding fingerprint under `partialFingerprints` and a waived finding as an external
  suppression.
* `rosio drift --baseline FILE` leaves out every finding whose id the baseline atlas holds, so
  the gate grades only what a change adds. A waiver whose finding the baseline holds is not
  reported as a waiver that matched nothing.
* `rosio drift --format markdown` writes a short summary for a pull request: the counts by
  severity and one line per error and warn finding with the line to edit.
* A GitHub Action at the repository root, `uses: sa1ntsinner/rosio@main`. It annotates the
  pull request, writes the job summary, uploads SARIF, and with `base-ref` grades only the
  findings the pull request adds. See [docs/ci.md](docs/ci.md).
* The repository is a pre-commit hook source, hook `rosio-drift`.
* Tested on ROS 2 Lyrical and Kilted and on Python 3.14.

### VS Code

* The hover says what a name was read from (config, code, the live system, a bag).

### Assistants

* `rosio install claude-code` registers a SessionStart hook, `rosio hook brief`, which states
  the profiles, the finding counts and the worst findings of the atlas, and says when a source
  changed after it was extracted.
* A UserPromptSubmit hook, `rosio hook prompt`, states what the atlas holds about the nodes,
  topics, parameters, frames, files and rules a prompt names, from a name index cached outside
  the workspace.
* The Stop hook extracts a workspace the session edited again into a temporary directory and
  stops the session once with the findings the edit added. `rosio-out` is left as it was.
  When the atlas was already older than the sources as the session began, it says so and
  calls those findings not in the atlas rather than added by the edits.
* `rosio uninstall claude-code` removes all three hooks.

### Reading

* The fallback configuration no longer offers a launch file that a CMake test command of its
  package runs, nor, in `rosio extract` as in `rosio init`, one only an including launch file
  can start because it passes the arguments the file needs. A `LaunchConfiguration` read with
  no default that the file declares nowhere counts as such an argument.
* A xacro file whose root element is not `robot`, or that only defines macros, is read as a
  file other models include and not as a model of its own.
* A file the package's CMake writes into the build directory and installs into its share is
  reported once by `build.unbuilt` instead of `launch.raised` per launch file.
* A xacro include path that an empty argument spells is no longer called an unbuilt package:
  the refusal names the empty argument, and only a profile that loads the model reports it.
* A process a launch site starts inside an `OpaqueFunction` or an include keeps its own inline
  launch values.
* `param.set_undeclared` is silent on a key of a python dict several launch actions are handed
  when another of those nodes declares the key or builds a name that could be it.
* `launch.raised` names the command a launch file runs while its description is built and the
  message a file exits with, asks for an argument whose empty default is no valid choice, and
  asks for a code fix when the release rejects a call.
* `extract.input_refused` over a missing parameters file cites the `DeclareLaunchArgument`
  whose default spells only the file name.
* A mesh under a directory the reader may not enter is a missing mesh and no longer ends the
  robot model pass.
* A fallback profile for a launch file outside every package loads.
* `frame.orphan_root` is silent on two transform trees no profile runs together, and where
  every profile that runs them starts a node whose code was not read or stopped its launch
  walk, as `frame.no_publisher` is.
* A C++ parameter whose default is a member initialized with an integer literal takes the
  type its class declares the member with, so a `double` member no longer reads as an integer
  parameter and `param.type_changed` stays silent on it.

## 0.2.0

Atlas schema `1.34` (`0.1.0` wrote `1.32`). Both steps are additive: a reader of a `1.32`
file loads a `1.34` file.

### Fixes you can apply

* A finding whose fix is one exact replacement states it under `edits`: a parameter key one
  slip from a declared name, a frame name the model spells otherwise, a remap source one slip
  from an endpoint name.
* `rosio drift --fix` writes those replacements. It edits a line only where the text to
  replace stands once as a whole word, so a line somebody already changed is left alone.
* VS Code offers each one as a quick fix and applies all of them in one step
  (`Ctrl+Alt+.`), then extracts again.
* The MCP `drift` answers carry the edits, and `check` lists the findings an edit removed and
  the ones it added.

### Command line

* `rosio drift` prints the line to edit first, as `path:line`, and leaves out a severity it
  has no finding of.
* `rosio extract` ends on what to do next and warns when findings went because it read less.
* `rosio explain` takes a file and line and says whether what that line sets wins.
* The layers read as config, code, live and bag in output. Atlas keys, ids and finding text
  keep `declared`, `coded`, `observed` and `recorded`.

### VS Code

* Findings are grouped by the file that holds the line to edit and open that line.
* A failed extract says why in one message with one action, a save that made the atlas stale
  shows in the status bar, and a missing live layer is one click to `rosio probe`.
* The atlas panel is one column and folds a repeated rule into one row.

### atlas.html

* Opens on the findings. The graph draws what runs, left to right by data flow, and a cited
  line copies as `file:line`.
* Neutral greys, no accent colour. Layer colours stay apart for colour blind readers.

### Reading

* rclcpp and rclpy calls that were not read: sub nodes, `get_parameters_by_prefix`, QoS
  overrides from a parameters file, generic clients and services, the `_full` and `_async`
  transform reads.
* A parameter declared through a macro over a helper function, or through a helper that
  takes the name as `const char *`, is read as declared.
* A node that sets its own parameter is a candidate of that parameter (`code_set`).
* Plugin instances: several plugin classes in one file, suffix reads, defaults of a selected
  instance.
* `frame_prefix` on a robot model, a remapped static transform publisher, python transforms
  held in a member.
* Autoware node registration and handoff idioms.
* A repository holding `package.xml` at its top is read as one package.
* A launch file that makes launch_ros start its node (`SetUseSimTime` among others) is read.
  Its walk used to finish and then wait out the timeout, so the profile came back empty.
* `param.orphan_block` is a note in a profile that starts an unnamed process whose code was
  not read, since the block may belong to that process.

### Assistants

* `rosio install` also writes `coding.md` and `review.md`, the habits for writing, fixing and
  reviewing ROS 2 code.
* `rosio install claude-code` registers a Stop hook: a session that changed code reviews its
  diff once, and a review that opened few files is pointed at the ones it skipped.

### Waivers

`0.1.0` was published, so this matters to a repository that keeps `rosio-waivers.yaml`. The
readings above bind nodes, endpoints and parameters that were not bound before, which moved
the ids of findings about them and removed findings that were false. A waiver keyed on such
an id or fingerprint is listed by `rosio drift` as a key that matched nothing. Delete it, and
waive the new finding if it still applies. No rule id was renamed or removed.

## 0.1.0

First release.
