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
edit src/cluster.hpp
set splitbelow splitright
set nosplitbelow
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 64 - ((46 * winheight(0) + 26) / 53)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
64
normal! 030|
tabnext 1
badd +53 src/cluster.hpp
badd +151 src/cluster.cpp
badd +17 src/ukf/filter_common.h
badd +3 src/datmo.cpp
badd +65 src/datmo.hpp
badd +7 launch/sim_test.launch
badd +120 src/l_shape_tracker.cpp
badd +66 src/kalman-cpp/kalman.hpp
badd +26 src/ukf/ukf.cpp
badd +57 src/ukf/filter_base.cpp
badd +48 src/kalman-cpp/kalman.cpp
badd +22 src/l_shape_tracker_ukf.cpp
badd +12 src/l_shape_tracker_ukf.hpp
badd +56 CMakeLists.txt
badd +24 src/l_shape_tracker.hpp
badd +0 l_shape_tracker.cpp:datmo/CMakeFiles/datmo.dir/build.make
badd +1 CMakeFiles/Makefile2
badd +1 Makefile
badd +0 b\ src/l_shape_tracker.cpp
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 shortmess=filnxtToOS
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
