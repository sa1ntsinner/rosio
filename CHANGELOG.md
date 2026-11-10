# Changelog

## Unreleased

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
