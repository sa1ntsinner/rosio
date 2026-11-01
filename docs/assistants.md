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

Twelve tools: `overview`, `param_chain`, `drift`, `explain`, `query`, `path`, `neighbours`,
`graph`, `unresolved`, `stamps`, `transform`, `migrate`. Every response is capped and says when it truncated and how to narrow the question.
Flags keep the command line's names (`only` as in `--only`, `source` and `target` as in
`rosio path`). Positional words get names of their own: `query` takes `question`, `param_chain`
takes `node` and `name`, and `migrate` takes `start` for `--from` and accepts `from` too. An
argument a tool does not take is refused with the names it does.
The server starts without an atlas, and each tool then names the extract command that builds one.
Over `--http`, `migrate` reads only the served workspace: the directory `--workspace` names, else
the one above the `rosio-out` that holds the atlas, else the directory of the atlas itself.
A worst case answer is under 4 KB. On very small models prefer the command
line, where the skill file is 3.6 KB and carries six commands, the waiver file and a worked example.

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
