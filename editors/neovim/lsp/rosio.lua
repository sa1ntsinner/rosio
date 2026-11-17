-- rosio-lsp for Neovim 0.11 and later. Copy this file to ~/.config/nvim/lsp/rosio.lua and
-- call vim.lsp.enable('rosio') in init.lua. It attaches beside clangd, pyright and the rest.
return {
  cmd = { 'rosio-lsp' },
  filetypes = { 'c', 'cpp', 'python', 'yaml', 'xml', 'cmake' },
  root_markers = { 'rosio.yaml', 'rosio-out', '.git' },
  init_options = {
    atlasPath = 'rosio-out/atlas.json',
    extractCommand = { 'rosio', 'extract', '.' },
    -- Saving a file the atlas cites runs extractCommand and reloads the findings. Neovim on
    -- Linux does not report a change to atlas.json, so this is how findings follow an edit.
    extractOnSave = true,
    codeLens = true,
  },
}
