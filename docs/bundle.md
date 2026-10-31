# Handing a problem over

A robot that misbehaves on one machine is usually debugged by somebody on another. `rosio bundle`
puts the atlas, both reports, the files the findings cite, the configuration and the git revision
into one archive, and the person who opens it gets the same findings pointing at the same lines,
without the workspace, a build or a ROS installation.

```bash
rosio bundle create . --note "safety stop never triggers"
rosio bundle open rosio-out/bundle.rosio.zip --into problem
```

`open` unpacks into a directory whose `files/` tree stands in for the sender's workspace, writes
a `rosio.yaml` pointing at it, and prints the most severe findings with the file and line to edit.
`rosio drift`, `rosio explain` and the editor extensions all work on it as they do on a checkout.

What goes in:

- by default the files the findings cite, plus each package's `package.xml`; `--include cited`
  takes every file any evidence row cites, up to 2000 files and 50 MiB;
- the uncommitted text changes as `changes.diff`, up to 5 MiB, binary changes left out;
- a `manifest.json` listing every file that was left out and why.

Values whose name matches `password`, `passwd`, `secret`, `token`, `api_key` or `private_key`
are replaced by `<redacted>` in yaml, launch argument defaults and `name:=value` arguments, and
environment blocks are removed. That is a rule on names, so a credential stored under an unrelated
name has to be removed by hand before sending. Line counts stay as they were, so every cited line
still points at the right place. Two creates over the same state write the same bytes.
