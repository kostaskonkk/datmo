let SessionLoad = 1
if &cp | set nocp | endif
let s:so_save = &so | let s:siso_save = &siso | set so=0 siso=0
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd ~/datmo_ws/src/datmo
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
set shortmess=aoO
argglobal
%argdel
$argadd src/datmo.cpp
$argadd src/datmo.hpp
$argadd src/cluster.cpp
$argadd src/cluster.hpp
$argadd src/l_shape_tracker.cpp
$argadd src/l_shape_tracker.hpp
edit src/l_shape_tracker.hpp
set splitbelow splitright
set nosplitbelow
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
argglobal
if bufexists("src/l_shape_tracker.hpp") | buffer src/l_shape_tracker.hpp | else | edit src/l_shape_tracker.hpp | endif
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 17 - ((16 * winheight(0) + 65) / 131)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
17
normal! 0
tabnext 1
badd +22 src/datmo.cpp
badd +2 src/datmo.hpp
badd +237 src/cluster.cpp
badd +72 src/cluster.hpp
badd +47 src/l_shape_tracker.cpp
badd +0 src/l_shape_tracker.hpp
badd +4 launch/sim_test.launch
badd +8 launch/test.launch
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 shortmess=filnxtToOSc
set winminheight=1 winminwidth=1
let s:sx = expand("<sfile>:p:r")."x.vim"
if file_readable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &so = s:so_save | let &siso = s:siso_save
nohlsearch
let g:this_session = v:this_session
let g:this_obsession = v:this_session
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
