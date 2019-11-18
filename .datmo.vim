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
$argadd src/cluster.cpp
edit src/ukf/filter_common.h
set splitbelow splitright
set nosplitbelow
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
argglobal
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
37
normal! zo
let s:l = 61 - ((60 * winheight(0) + 65) / 131)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
61
normal! 0
tabnext 1
badd +215 src/cluster.cpp
badd +56 src/cluster.hpp
badd +116 src/datmo.cpp
badd +65 src/datmo.hpp
badd +7 launch/sim_test.launch
badd +32 src/l_shape_tracker.cpp
badd +66 src/kalman-cpp/kalman.hpp
badd +324 src/ukf/ukf.cpp
badd +57 src/ukf/filter_base.cpp
badd +61 src/ukf/filter_common.h
badd +48 src/kalman-cpp/kalman.cpp
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
let g:this_session = v:this_session
let g:this_obsession = v:this_session
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
