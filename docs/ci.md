# rosio in CI

`rosio drift` grades the findings of an atlas with `--fail-on`, and `--format` decides how they
are printed. `github` and `sarif` put each finding on the line of the pull request it is about,
so nobody has to read the job log.

## GitHub Action

```yaml
on: pull_request
permissions:
  contents: read
  security-events: write   # only for the SARIF upload
jobs:
  rosio:
    runs-on: ubuntu-latest
    container: ros:jazzy-ros-base   # so the launch files are walked
    steps:
      - uses: actions/checkout@v4
      - uses: sa1ntsinner/rosio@main
```

Inputs, all optional: `path` (the workspace, default `.`), `fail-on` (default `error`), `version`
(the rosio release to install, default `latest`), `sarif` (default `true`, uploads the findings to
code scanning), `args` (more arguments for `rosio extract`), `baseline` (an atlas file whose
findings are left out) and `base-ref` (a git ref to extract as the baseline). The action installs
`rosio[cpp]`, runs `rosio extract`, uploads the SARIF file, writes the job summary with
`rosio drift --format markdown`, then runs `rosio drift --format github --fail-on <fail-on>`,
which fails the step. Set `sarif: false` when code scanning is not enabled for the repository.

The summary sits on the run page of the pull request: the counts by severity, then one line per
error and warn finding with the file and line to edit, the rule and the finding's sentence, thirty
at most. Waived findings and findings the baseline holds are left out and counted. The same text
is kept in `$RUNNER_TEMP/rosio-summary.md` for a later step that posts it as a comment.

The action sources the setup file under `/opt/ros` when the job has one, which is what the
`container` line gives it, and installs the packages your launch files import the way your build
job does. Without a ROS installation it still reads C++ and Python sources, parameter files, robot
models, plugin manifests and interfaces, but no launch file is walked: every launch entry point is
a `launch.unloadable` finding at error, and the nodes and parameters a launch file composes are
missing from the atlas. Use the container.

## The formats

```bash
rosio drift --format github --fail-on error
rosio drift --format sarif > rosio.sarif
rosio drift --format markdown >> "$GITHUB_STEP_SUMMARY"
```

`github` prints one workflow command per finding, `::error`, `::warning` or `::notice` by
severity, with the file relative to the repository, the line, the rule as title, and the finding's
sentence and fix as the message. A waived finding is left out. `sarif` writes SARIF 2.1.0: one
result per finding with its rule, level, location and its fingerprint under `partialFingerprints`,
so code scanning follows one finding across commits. A waived finding stays in, with an external
suppression carrying the waiver reason. A finding tied to no file is printed without a location by
`github` and left out of `sarif`, because code scanning refuses a result with no location. `--fail-on` grades the same findings whatever the format.

## Only the findings a change adds

```bash
rosio drift --baseline main-atlas.json --fail-on error
```

`--baseline` takes another atlas, the target branch's for instance, and leaves out every finding
whose id it holds, so the gate grades only what is new. The text output says how many it held
back. `rosio diff` compares two atlases in full, and `rosio diff --rev main --rev HEAD` builds both
from git revisions, but it keeps neither atlas, so a baseline file is one more extract of the base.

In the action, `base-ref` does that: it fetches the ref, checks it out beside the pull request with
`git worktree add`, extracts it, and passes the atlas as `--baseline` to the annotations, the
summary and the gate. The SARIF upload keeps every finding, because code scanning tracks which
alerts a pull request opens by itself.

```yaml
      - uses: sa1ntsinner/rosio@main
        with:
          base-ref: ${{ github.base_ref }}
```

Finding ids carry no path, so an atlas of the base extracted in another directory holds the same
ids for the same findings. Elsewhere it is two extracts:

```bash
git worktree add --detach ../base origin/main
rosio extract ../base --out ../base-out
rosio extract .
rosio drift --baseline ../base-out/atlas.json --fail-on error
```

## Before committing

The repository is a [pre-commit](https://pre-commit.com) hook source. The hook `rosio-drift`
extracts the repository into `rosio-out` and runs `rosio drift --fail-on error`, printing only
the errors, when a commit touches a launch file, parameters, a robot model or a C++ or python
source.

```yaml
repos:
  - repo: https://github.com/sa1ntsinner/rosio
    rev: main
    hooks:
      - id: rosio-drift
```

Launch files are walked only where ROS is sourced, so run `git commit` from a shell that sourced
`/opt/ros/<release>/setup.bash`. Without it every launch entry point is a `launch.unloadable`
finding at error and the hook fails. The hook installs rosio in an environment of its own, which
cannot import the ROS python packages, so it takes its own directory off `PATH` before the walk:
the launch files then run under the `python3` the setup file put there, as `ros2 launch` runs
them. An activated virtual environment ahead of it on `PATH` hides that `python3` the same way,
so start `git commit` from a shell that has none.

## Waivers

A finding the repository has decided about is waived by its id or fingerprint, with a reason, in
the file the `waivers` key of `rosio.yaml` names, and the action reads that key from the
`rosio.yaml` in `path`. See "Waiving a finding you have already decided about" in
[commands.md](commands.md).

## GitLab CI

```yaml
rosio:
  image: python:3.12
  script:
    - pip install "rosio[cpp]"
    - rosio extract .
    - rosio drift --format sarif > gl-rosio.sarif
    - rosio drift --fail-on error
  artifacts:
    when: always
    paths: [gl-rosio.sarif]
```

The plain text output names the file and line of every finding in the job log, and the SARIF file
is kept as an artifact for any tool that reads it.
