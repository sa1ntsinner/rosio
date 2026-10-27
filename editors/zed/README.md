# rosio for Zed

Connects Zed to `rosio-lsp` for diagnostics, hover, go to definition, references,
workspace symbols, inlay hints that say whether a parameter line wins or is shadowed, and
the quick fixes and migration actions from a ROS 2 atlas. It also provides an MCP context server through
`rosio serve` over stdio.

The graph, frames, parameters and robot model views of the VS Code extension are webview
panels, which Zed extensions cannot open. In Zed the same answers come from hover, from
`rosio graph` and `rosio explain` in a terminal, and from the MCP tools in the assistant.

## Install

The extension starts `rosio-lsp` from `lsp.rosio.binary.path` when set, else from PATH,
else it downloads `rosio-lsp-<target>` from the latest release of
[sa1ntsinner/rosio](https://github.com/sa1ntsinner/rosio/releases) once per version.
Release builds exist for linux-x64, linux-arm64, win32-x64, darwin-x64 and darwin-arm64.
To build it yourself instead, run `cargo install --path crates/rosio-lsp` from a rosio
checkout.

Install the Python CLI with MCP support, then extract an atlas from your ROS workspace:

```sh
pip install 'rosio[mcp]'
rosio extract .
```

The atlas comes from `rosio extract`, which writes `rosio-out/atlas.json`. Open the
workspace folder in Zed. In the command palette, run `zed: install dev extension` and
select this `editors/zed` directory. Zed builds the extension for wasm independently
of the root Cargo workspace. If needed, install its target first:

```sh
rustup target add wasm32-wasip1
```

The extension uses Zed's built in [C++](https://zed.dev/docs/languages/cpp),
[C](https://zed.dev/docs/languages/c), [Python](https://zed.dev/docs/languages/python)
and [YAML](https://zed.dev/docs/languages/yaml) support.

## Settings

Both binaries are read from PATH by default, and the language server is downloaded when
PATH has none. Set `lsp.rosio.binary.path` to override
the language server, and `context_servers.rosio.command.path` to override the MCP
command. Use absolute paths. On Windows these can point directly to `.exe` files,
including the pip shim for `rosio`.

```json
{
  "lsp": {
    "rosio": {
      "binary": {
        "path": "/path/to/rosio-lsp",
        "arguments": [],
        "env": { "RUST_LOG": "info" }
      },
      "initialization_options": {
        "atlasPath": "rosio-out/atlas.json",
        "roots": {},
        "extractCommand": ["rosio", "extract", "."],
        "extractOnSave": false
      }
    }
  },
  "context_servers": {
    "rosio": {
      "command": {
        "path": "/path/to/rosio",
        "arguments": ["serve", "--atlas", "/path/to/workspace/rosio-out/atlas.json"],
        "env": {}
      }
    }
  }
}
```

`binary.arguments` replaces the LSP arguments, which default to an empty list.
`binary.env` overrides matching variables in the worktree shell environment.
`initialization_options` passes through unchanged; when absent it defaults to
`{"atlasPath": "rosio-out/atlas.json"}`. `atlasPath` is relative to the workspace
folder. `roots` maps atlas root labels to local directories. `extractCommand` is a
program and arguments run without a shell; `extractOnSave` enables extraction when
a cited file is saved. See the [server README](../../crates/rosio-lsp/README.md).

The MCP `command.arguments` replaces the default `["serve"]`; keep `serve` when
using the rosio CLI. `command.env` supplies environment overrides for that process.
Use an absolute `--atlas` path to select the MCP atlas independently of its working
directory. MCP settings are separate from LSP initialization options.

If your language settings specify `language_servers`, include `"rosio"` alongside
`"..."` to keep the existing servers, for example:

```json
{
  "languages": {
    "C++": { "language_servers": ["rosio", "..."] },
    "C": { "language_servers": ["rosio", "..."] },
    "Python": { "language_servers": ["rosio", "..."] },
    "YAML": { "language_servers": ["rosio", "..."] }
  }
}
```

## Verify

From this directory:

```sh
cargo fmt --check
cargo clippy --target wasm32-wasip1 -- -D warnings
cargo build --release --target wasm32-wasip1
```
