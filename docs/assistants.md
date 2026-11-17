# Assistants

`rosio install` writes the files your assistant reads. Four mechanisms cover most tools: a skill
file, a section in an instruction file, a rules file, and an MCP server registration.

```bash
rosio install --list                   # the table: platform, mechanism, and the exact paths
rosio install claude-code              # this repository, which is the default scope
rosio install codex --scope user       # your user profile instead
rosio install cursor --dry-run         # print what would be written, write nothing
rosio uninstall claude-code            # put the tree back
```

A project file such as `.mcp.json` is committed, so its server entry runs `rosio serve --atlas
rosio-out/atlas.json` from the PATH, in the project directory the assistant starts it in, and
works in every clone. When `rosio` is not on a minimal PATH, install says so and names the fix:
`uv tool install` of your checkout puts `rosio` in `~/.local/bin`, and for an assistant whose user
scope holds a server entry, `--scope user` registers the interpreter that ran install by its
absolute path instead.

Claude Code, Codex, Cursor, Gemini CLI, GitHub Copilot, Aider, OpenCode, Amp, Kilo, and the cross
framework agent skills directory. Anything that speaks MCP can use the server directly:

```bash
rosio serve                                        # stdio
ROSIO_API_KEY="$KEY" rosio serve --http --host 0.0.0.0 # one server for a team, key kept out of ps
```

Fourteen tools: `overview`, `param_chain`, `drift`, `explain`, `query`, `path`, `neighbours`,
`graph`, `unresolved`, `stamps`, `transform`, `migrate`, `check`, `verify`. Every response is capped and says when it truncated and how to narrow the question.
`explain` also takes a file and line, `config/rover.yaml:9`, and answers what that line feeds:
each parameter value it sets and whether it wins or which line outbids it, and the findings
that cite it, so an agent knows before editing whether the edit reaches a node. `check` extracts
the workspace again after an edit and lists the findings that went and the ones that arrived,
which is how a fix is confirmed. `rosio extract` prints the same list when it replaces an atlas.
`verify` builds and tests the packages holding the files git reports changed, as `rosio verify`
does, and names each compiler error by file and line and each failed test with its first
message line.
Flags keep the command line's names (`only` as in `--only`, `source` and `target` as in
`rosio path`). Positional words get names of their own: `query` takes `question`, `param_chain`
takes `node` and `name`, and `migrate` takes `start` for `--from` and accepts `from` too. An
argument a tool does not take is refused with the names it does.
The server starts without an atlas, and each tool then names the extract command that builds one.
Over `--http`, `migrate` reads, `check` extracts and `verify` builds only the served workspace: the directory `--workspace` names, else
the one above the `rosio-out` that holds the atlas, else the directory of the atlas itself.
A worst case answer is under 4 KB. On very small models prefer the command
line, where the skill file is 3.4 KB and carries seven commands, the waiver file and a worked example.

For Claude Code, Codex and the portable skills directory, install also writes `review.md` beside
the skill: where ROS 2 code usually goes wrong (copy and paste, units and angles, frames, clocks,
parameters, QoS, executors, lifecycle, sensor message edge cases) and how to find each case with
a search instead of reading whole packages. The skill tells an agent to read it before hunting
bugs or reviewing a change, so it costs nothing on other tasks. `coding.md` beside it holds the
habits for writing, changing and debugging ROS 2 code, read before an agent edits code.

## Claude Code and Codex hooks

`rosio install claude-code` also registers three hooks in `.claude/settings.json`, and
`rosio install codex` registers the same three in `.codex/hooks.json`. Each prints
what rosio measured, and prints nothing and exits zero when there is no atlas or anything fails.

* `rosio hook brief`, at session start: the profiles, the finding counts by severity, the worst
  five error and warn findings as `path:line rule title`, and a line when a source file changed
  after `rosio-out/atlas.json` was written.
* `rosio hook prompt`, on each prompt: for up to three node, topic, service, action, parameter,
  frame, file or rule names the prompt spells exactly, where it is defined, its publishers and
  subscribers with QoS, the value a parameter has and the line that sets it, and the findings
  about it. It reads a name index cached under the system temporary directory, keyed by the
  atlas's size and modification time.
* `rosio hook coverage`, when a session stops: if the session edited a source, launch,
  parameter, robot model or build file, the workspace is extracted again into a temporary
  directory and the session is stopped once with each finding the edit added, its line and the
  exact replacement when the finding carries one. `rosio-out` stays as it was. A file counts as
  edited when the session's transcript names it and git reports it changed, or when git reports
  it changed and there is no transcript.

Codex runs a project hook only in a trusted project, and only after you approve it in `/hooks`
(or for one run under `codex exec --dangerously-bypass-hook-trust`). It asks again after
every edit to `.codex/hooks.json`, because the approval is kept per hook content.

## Privacy

Everything runs locally. Code, launch files, parameters and recordings are parsed on your machine
and nothing is uploaded. There is no telemetry and no network call anywhere in extraction, drift
or rendering. The MCP server binds to loopback and refuses to bind wider without an API key.

Python launch files are executed to read the description they build, the way `ros2 launch`
does before it starts anything. Every attempt to start a process during that walk is refused
and noted in the run. With a working `bwrap` (the `bubblewrap` apt package), the walk uses
a read only filesystem, private temporary and ROS log directories, and no network access.
Otherwise it keeps your user's file and network access. `launch_sandbox: required` in
`rosio.yaml` refuses that fallback, and `rosio extract --launch-sandbox off` disables isolation.
The default is `auto`. Only extract workspaces you would also `ros2 launch`.
