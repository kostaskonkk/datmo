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
edit src/ukf/ukf.cpp
set splitbelow splitright
set nosplitbelow
set nosplitright
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
argglobal
if bufexists("src/ukf/ukf.cpp") | buffer src/ukf/ukf.cpp | else | edit src/ukf/ukf.cpp | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 63 - ((23 * winheight(0) + 26) / 53)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
63
normal! 07|
lcd ~/datmo_ws/src/datmo
tabnext 1
badd +43 ~/datmo_ws/src/datmo/src/ukf/filter_base.h
badd +263 ~/datmo_ws/src/datmo/src/datmo.cpp
badd +20 ~/datmo_ws/src/datmo/src/datmo.hpp
badd +191 ~/datmo_ws/src/datmo/src/cluster.cpp
badd +65 ~/datmo_ws/src/datmo/src/cluster.hpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.cpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.hpp
badd +43 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.h
badd +71 ~/datmo_ws/src/datmo/src/ukf/ukf.h
badd +9 ~/datmo_ws/src/datmo/launch/sim_test.launch
badd +8 ~/datmo_ws/src/datmo/launch/test.launch
badd +14 ~/datmo_ws/src/datmo/src/main.cpp
badd +60 ~/datmo_ws/src/datmo/CMakeLists.txt
badd +9 ~/datmo_ws/src/datmo/launch/simulation.launch
badd +1 ~/datmo_ws/src/datmo/src/datmo.h
badd +3 ~/datmo_ws/src/localization/launch/mocap_localization.launch
badd +2 ~/datmo_ws/src/datmo/msg/Track.msg
badd +13 ~/datmo_ws/src/datmo/src/kalman-cpp/kalman.cpp
badd +292 ~/datmo_ws/src/datmo/src/ukf/ukf.cpp
badd +76 ~/datmo_ws/src/datmo/src/ukf/filter_common.h
badd +33 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.cpp
badd +357 ~/datmo_ws/src/datmo/src/ukf/filter_base.cpp
badd +59 ~/datmo_ws/src/robot_localization/test/test_ukf.cpp
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
