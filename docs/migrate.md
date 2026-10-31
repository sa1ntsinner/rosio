# Moving to another ROS 2 release

Humble ends in May 2027. `rosio migrate` lists what moving a workspace from one release to another
asks of its own packages, with the file and line of each change and the source that says so:

```bash
rosio migrate plan --from humble --to lyrical     # every change, edits nothing
rosio migrate apply --from humble --to lyrical    # the exact rewrites, and the diff in rosio-out/migrate/
rosio migrate verify --from humble --to lyrical   # build in the target and match errors to the plan
```

What it knows comes from two kinds of source. Measured facts: which packages rosdistro releases for each
release, and which headers the official `ros:<release>` images install, with common ROS
packages (ros2_control, nav2, cv_bridge and BehaviorTree.CPP) and system development libraries
(Ceres, OpenCV and PCL) added, and the parameters nav2 nodes declare in
each release, read from the nav2 sources, so a parameter file learns which keys a node now ignores
and which unset keys changed their default. A short table of hand written rules
for what a measurement cannot see: a renamed API, a CMake function that is gone, a remapping that
silently stops applying, each with its upstream source. Nothing is recalled from release notes.

A change is either an exact rewrite (a header's new spelling or a renamed class) or one sentence for the person making it.
`apply` performs only the first kind, refuses files with uncommitted edits, and states for each
rewrite the oldest release where the new form builds, so a package that supports both ends knows
whether it can take the change today.

Finish with `verify`. It copies the source packages into a disposable Docker container,
installs dependencies with rosdep and builds with colcon. It uses a local
`rosio-facts:<release>` image when present, otherwise `ros:<release>-ros-base`;
`--image NAME` selects another image. Docker must be installed and running. Nothing is
built in the workspace. The report lists finished, failed and skipped packages, each
error as `listed` with the plan's rule and sentence or `not_listed`, and the plan changes
still open. Matching uses the same package, file and the lines the planned expression
spans, so it states location correspondence rather than proving the cause of an error.
`--json` returns the same report as data. A complete build exits 0, a failed or incomplete
build exits 1, and unavailable Docker exits 2.

In VS Code the plan appears as diagnostics with quick fixes and in a Migration view, by hand
changes first. Verify build runs `migrate verify` and lists each package that fails in the target,
errors the plan does not cover first. Agents get the same plan from the `migrate` tool, which hands the exact rewrites to
`apply` instead of asking a model to retype them.
