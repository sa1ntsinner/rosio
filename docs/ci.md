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
    steps:
      - uses: actions/checkout@v4
      - uses: sa1ntsinner/rosio@main
```

Inputs, all optional: `path` (the workspace, default `.`), `fail-on` (default `error`), `version`
(the rosio release to install, default `latest`), `sarif` (default `true`, uploads the findings to
code scanning) and `args` (more arguments for `rosio extract`). The action installs
`rosio[cpp]`, runs `rosio extract`, uploads the SARIF file, then runs
`rosio drift --format github --fail-on <fail-on>`, which fails the step. Set `sarif: false` when
code scanning is not enabled for the repository.

The runner has no ROS installation, so the action reads the layers that need none: C++ and Python
sources, parameter files, robot models, plugin manifests and interfaces. Launch files are not
executed: the run summary says the launch layer did not happen, and every launch entry point is a
`launch.unloadable` finding at error, so with the default `fail-on` the step fails until those
findings are waived (below). The nodes and parameters a launch file composes are missing from that
atlas. To read them, run `rosio extract` yourself in a job where ROS 2 is installed and sourced.

## The two formats

```bash
rosio drift --format github --fail-on error
rosio drift --format sarif > rosio.sarif
```

`github` prints one workflow command per finding, `::error`, `::warning` or `::notice` by
severity, with the file relative to the repository, the line, the rule as title, and the finding's
sentence and fix as the message. A waived finding is left out. `sarif` writes SARIF 2.1.0: one
result per finding with its rule, level, location and its fingerprint under `partialFingerprints`,
so code scanning follows one finding across commits. A waived finding stays in, with an external
suppression carrying the waiver reason. A finding tied to no file is printed without a location in
both. `--fail-on` grades the same findings whatever the format.

## Only the findings a change adds

```bash
rosio drift --baseline main-atlas.json --fail-on error
```

`--baseline` takes another atlas, the target branch's for instance, and leaves out every finding
whose id it holds, so the gate grades only what is new. The text output says how many it held
back. `rosio diff` compares two atlases in full, and `rosio diff --rev main --rev HEAD` builds both
from git revisions.

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
