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
set splitbelow splitright
set nosplitbelow
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
tabnext 1
badd +302 src/ukf/ukf.cpp
badd +106 src/cluster.cpp
badd +106 src/cluster.hpp
badd +17 src/ukf/filter_common.h
badd +3 src/datmo.cpp
badd +65 src/datmo.hpp
badd +7 launch/sim_test.launch
badd +45 src/l_shape_tracker.cpp
badd +66 src/kalman-cpp/kalman.hpp
badd +1 src/ukf/filter_base.cpp
badd +48 src/kalman-cpp/kalman.cpp
badd +22 src/l_shape_tracker_ukf.cpp
badd +12 src/l_shape_tracker_ukf.hpp
badd +56 CMakeLists.txt
badd +24 src/l_shape_tracker.hpp
badd +1 l_shape_tracker.cpp:datmo/CMakeFiles/datmo.dir/build.make
badd +1 CMakeFiles/Makefile2
badd +1 Makefile
badd +1 b\ src/l_shape_tracker.cpp
badd +120 src/ukf/ukf.h
badd +1 ~/.vimrc
badd +1 datmo/CMakeFiles/datmo.dir/build.make
badd +304 src/ukf/filter_base.h
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
