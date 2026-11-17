" rosio-lsp for Vim 9 with vim-lsp (https://github.com/prabirshrestha/vim-lsp).
" Copy to ~/.vim/plugin/rosio.vim. It registers beside any other server vim-lsp starts.

function! s:rosio_root(server_info) abort
  let l:dir = lsp#utils#find_nearest_parent_file_directory(
        \ lsp#utils#get_buffer_path(), ['rosio.yaml', 'rosio-out/', '.git/'])
  return lsp#utils#path_to_uri(empty(l:dir) ? getcwd() : l:dir)
endfunction

augroup rosio_lsp
  autocmd!
  autocmd User lsp_setup call lsp#register_server({
        \ 'name': 'rosio',
        \ 'cmd': {server_info -> ['rosio-lsp']},
        \ 'root_uri': function('s:rosio_root'),
        \ 'initialization_options': {
        \   'atlasPath': 'rosio-out/atlas.json',
        \   'extractCommand': ['rosio', 'extract', '.'],
        \   'extractOnSave': v:true,
        \ },
        \ 'allowlist': ['c', 'cpp', 'python', 'yaml', 'xml', 'cmake'],
        \ })
augroup END
