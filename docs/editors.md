# Editors

`rosio-lsp` is the language server behind the VS Code extension and the Zed extension. Any
editor that speaks LSP can start it: it puts each finding on the line it cites, offers the fix
as a quick fix when the fix is one exact replacement, offers to waive any finding in
`rosio-waivers.yaml`, and answers hover, go to definition, references and workspace symbols.
The VS Code extension carries the server and Zed downloads it, so the sections below are for
every other editor.

## What you need

**The server.** Every GitHub release attaches one binary per platform: `rosio-lsp-linux-x64`,
`rosio-lsp-linux-arm64`, `rosio-lsp-darwin-x64`, `rosio-lsp-darwin-arm64` and
`rosio-lsp-win32-x64.exe`. Put it on `PATH` under the name `rosio-lsp`:

```bash
mkdir -p ~/.local/bin
curl -fL -o ~/.local/bin/rosio-lsp \
  https://github.com/sa1ntsinner/rosio/releases/latest/download/rosio-lsp-linux-x64
chmod +x ~/.local/bin/rosio-lsp
```

Or build it with Rust:

```bash
cargo install --locked --git https://github.com/sa1ntsinner/rosio rosio-lsp
```

**The engine.** The server only reads an atlas; it never reads ROS sources itself. The atlas
is `rosio-out/atlas.json`, which `rosio extract` writes, so the Python `rosio` has to be
installed and on `PATH` for the server to have anything to show:

```bash
pip install "rosio[cpp]"      # in an environment made with --system-site-packages
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosio init
rosio extract
```

Without an atlas the server starts, shows nothing and says so in a message, which names how to
install `rosio` when it is not on `PATH`.

**Keeping findings current.** The configurations below set `extractOnSave`: saving a file the
atlas cites runs `rosio extract .` in the workspace and reloads the findings when it is done.
Turn it off on a workspace where extraction takes too long for every save, and run
`rosio extract` yourself; the server reloads `atlas.json` when the editor reports that it
changed, and otherwise when it is restarted.

**Trying it.** `examples/rover_ws` in this repository has five faults planted on purpose:

```bash
git clone https://github.com/sa1ntsinner/rosio
cd rosio/examples/rover_ws
source /opt/ros/jazzy/setup.bash
rosio extract
```

Open the editor in `examples/rover_ws` and open `src/rover_bringup/config/rover.yaml`. Lines
4, 9 and 12 carry a rosio warning each, and the quick fix on line 9 is
`Replace stop_distnce with stop_distance`. In `src/rover_perception/src/scan_filter.cpp`, line
24 carries the QoS finding and line 34 offers `Replace laser_frame with laser_link`.

The server takes the first workspace folder the editor sends and looks upward from it for
`rosio-out/atlas.json`, so opening a package directory inside the workspace works once an
atlas exists. Extraction runs in the directory the atlas was found in.

## The initialization options

Every editor passes the same options. The full table is in
[crates/rosio-lsp/README.md](../crates/rosio-lsp/README.md#initialization-options).

| Option | Value used here |
| --- | --- |
| `atlasPath` | `rosio-out/atlas.json` |
| `extractCommand` | `["rosio", "extract", "."]`, run without a shell, so a wrapper script that sources ROS goes here |
| `extractOnSave` | `true` |

## Neovim

Neovim 0.11 and later: copy [editors/neovim/lsp/rosio.lua](../editors/neovim/lsp/rosio.lua)
to `~/.config/nvim/lsp/rosio.lua` and add one line to `init.lua`:

```lua
vim.lsp.enable('rosio')
vim.filetype.add({ extension = { xacro = 'xml' } })
```

It attaches to C, C++, Python, YAML, XML and CMake buffers beside any other server. Neovim
detects `.urdf` and `package.xml` as XML on its own; the second line does the same for
`.xacro`. The quick fixes are under
`vim.lsp.buf.code_action()`, mapped to `gra` by default.

Older Neovim with nvim-lspconfig, which has no entry for rosio, so the configuration is
registered first:

```lua
local configs = require('lspconfig.configs')
if not configs.rosio then
  configs.rosio = {
    default_config = {
      cmd = { 'rosio-lsp' },
      filetypes = { 'c', 'cpp', 'python', 'yaml', 'xml', 'cmake' },
      root_dir = require('lspconfig.util').root_pattern('rosio.yaml', 'rosio-out', '.git'),
      init_options = {
        atlasPath = 'rosio-out/atlas.json',
        extractCommand = { 'rosio', 'extract', '.' },
        extractOnSave = true,
      },
    },
  }
end
require('lspconfig').rosio.setup({})
```

## Helix

Append [editors/helix/languages.toml](../editors/helix/languages.toml) to
`~/.config/helix/languages.toml`. Helix replaces a language's server list rather than adding to
it, so the file repeats the servers Helix 25.07 starts by default for C, C++, Python, YAML and
CMake and adds `rosio` to each; keep the ones you use. `hx --health yaml` then lists `rosio`
among the configured servers. Quick fixes are under `space a`. Helix has no language for
`.urdf` and `.xacro`, so findings in a robot model are not shown there.

## Vim

With [vim-lsp](https://github.com/prabirshrestha/vim-lsp), a plugin written in Vim script that
needs nothing else installed: copy [editors/vim/rosio.vim](../editors/vim/rosio.vim) to
`~/.vim/plugin/rosio.vim`. `:LspStatus` lists `rosio` as running in a C, C++, Python, YAML,
XML or CMake buffer, and `:LspCodeAction` offers the quick fixes.

## Emacs

Load [editors/emacs/rosio.el](../editors/emacs/rosio.el) from your init file. It holds two
registrations, each active once its package loads:

```elisp
(with-eval-after-load 'lsp-mode
  (lsp-register-client
   (make-lsp-client
    :new-connection (lsp-stdio-connection '("rosio-lsp"))
    :activation-fn (lsp-activate-on "c" "cpp" "python" "yaml" "xml" "cmake")
    :add-on? t
    :server-id 'rosio
    :initialization-options '(:atlasPath "rosio-out/atlas.json"
                              :extractCommand ["rosio" "extract" "."]
                              :extractOnSave t))))

(with-eval-after-load 'eglot
  (add-to-list 'eglot-server-programs
               '((yaml-mode yaml-ts-mode nxml-mode)
                 . ("rosio-lsp" :initializationOptions
                    (:atlasPath "rosio-out/atlas.json"
                     :extractCommand ["rosio" "extract" "."]
                     :extractOnSave t)))))
```

lsp-mode runs rosio as an add-on beside clangd and pylsp. Eglot runs one server per buffer, so
its registration covers only YAML and XML buffers, where rosio takes the place of the server
Eglot would start there; findings in C++ and Python files need lsp-mode.

## Sublime Text

With the [LSP](https://packagecontrol.io/packages/LSP) package, open Preferences, Package
Settings, LSP, Settings and add:

```json
{
  "clients": {
    "rosio": {
      "enabled": true,
      "command": ["rosio-lsp"],
      "selector": "source.c | source.c++ | source.python | source.yaml | text.xml | source.cmake",
      "initializationOptions": {
        "atlasPath": "rosio-out/atlas.json",
        "extractCommand": ["rosio", "extract", "."],
        "extractOnSave": true
      }
    }
  }
}
```

Sublime LSP starts every client whose selector matches, so rosio runs beside the others.

## Kate

Settings, Configure Kate, LSP Client, User Server Settings:

```json
{
  "servers": {
    "rosio": {
      "command": ["rosio-lsp"],
      "rootIndicationFileNames": ["rosio.yaml", "rosio-out"],
      "highlightingModeRegex": "^(YAML|XML)$",
      "initializationOptions": {
        "atlasPath": "rosio-out/atlas.json",
        "extractCommand": ["rosio", "extract", "."],
        "extractOnSave": true
      }
    }
  }
}
```

Kate starts one server per document, so this covers YAML and XML files and leaves C++ and
Python to clangd and the Python server. Widen the regular expression to `^(C|C\+\+|Python|YAML|XML|CMake)$`
to have rosio in those files instead of them.

## Which of these were started

The Neovim (0.11 with `lsp/rosio.lua`, and 0.9 with the nvim-lspconfig snippet), Helix 25.07
and Vim 9.1 with vim-lsp configurations were started on `examples/rover_ws` with the released
`rosio-lsp-linux-x64`, and each attached the server and received the findings above. In
Neovim the two quick fixes named above were offered, and saving `rover.yaml` with the typo
corrected removed its finding. The Emacs, Sublime Text and Kate configurations follow the documentation of those
clients and have not been started.
