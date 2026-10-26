# The engine is Python, the editor side is Rust, and the atlas is the only contract between them

Extraction has to import ROS 2 `launch`, `rclpy` and `rosbags`, which are Python, and it runs
where the robot is, often inside a ROS container. So the engine (collectors, join, drift, the CLI
and the MCP server) is Python, with pyyaml as its only core dependency. What sits in the editor
loop has to answer in milliseconds without importing ROS, so `rosio-lsp` is Rust and only reads
`atlas.json`. Neither side imports the other; a Rust port of an engine stage is accepted only when
it reproduces the Python atlas byte for byte on the fixtures and on public workspaces.

## Considered options

- All Rust. Rejected for now: the launch sandbox would still need a Python child, and the
  measured Python engine would be rewritten before anyone used it.
- All Python, with a Python language server. Rejected: an editor on a large workspace waits on
  Python start up and on a multi megabyte JSON parse on every reload.
