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
edit .git/index
set splitbelow splitright
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
exe '1resize ' . ((&lines * 36 + 37) / 74)
exe '2resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 2resize ' . ((&columns * 127 + 127) / 255)
exe '3resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 3resize ' . ((&columns * 127 + 127) / 255)
argglobal
if bufexists(".git/index") | buffer .git/index | else | edit .git/index | endif
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1 - ((0 * winheight(0) + 18) / 36)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
lcd ~/datmo_ws/src/datmo
wincmd w
argglobal
if bufexists("~/datmo_ws/src/datmo/src/ukf/ukf.cpp") | buffer ~/datmo_ws/src/datmo/src/ukf/ukf.cpp | else | edit ~/datmo_ws/src/datmo/src/ukf/ukf.cpp | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
1,31fold
75,79fold
54,80fold
85,86fold
89,90fold
118,121fold
105,122fold
135,137fold
139,141fold
143,145fold
132,146fold
130,147fold
184,186fold
192,198fold
207,213fold
179,214fold
219,221fold
230,233fold
238,242fold
259,261fold
264,266fold
257,267fold
253,268fold
272,290fold
93,291fold
345,348fold
353,355fold
361,364fold
293,381fold
50,383fold
50
normal! zo
54
normal! zo
75
normal! zo
93
normal! zo
105
normal! zo
118
normal! zo
130
normal! zo
132
normal! zo
135
normal! zo
139
normal! zo
143
normal! zo
130
normal! zc
179
normal! zo
184
normal! zo
192
normal! zo
207
normal! zo
219
normal! zo
230
normal! zo
238
normal! zo
253
normal! zo
257
normal! zo
259
normal! zo
264
normal! zo
272
normal! zo
93
normal! zc
293
normal! zo
345
normal! zo
353
normal! zo
361
normal! zo
let s:l = 341 - ((14 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
341
normal! 0
lcd ~/datmo_ws/src/datmo
wincmd w
argglobal
if bufexists("~/datmo_ws/src/datmo/src/ukf/filter_common.h") | buffer ~/datmo_ws/src/datmo/src/ukf/filter_common.h | else | edit ~/datmo_ws/src/datmo/src/ukf/filter_common.h | endif
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
let s:l = 75 - ((22 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
75
normal! 024|
lcd ~/datmo_ws/src/datmo
wincmd w
exe '1resize ' . ((&lines * 36 + 37) / 74)
exe '2resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 2resize ' . ((&columns * 127 + 127) / 255)
exe '3resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 3resize ' . ((&columns * 127 + 127) / 255)
tabnext 1
badd +33 ~/datmo_ws/src/datmo/src/ukf/ukf.cpp
badd +206 ~/datmo_ws/src/datmo/src/datmo.cpp
badd +49 ~/datmo_ws/src/datmo/src/datmo.hpp
badd +101 ~/datmo_ws/src/datmo/src/cluster.cpp
badd +33 ~/datmo_ws/src/datmo/src/cluster.hpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.cpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.hpp
badd +108 ~/datmo_ws/src/datmo/src/ukf/filter_base.h
badd +43 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.h
badd +85 ~/datmo_ws/src/datmo/src/ukf/ukf.h
badd +9 ~/datmo_ws/src/datmo/launch/sim_test.launch
badd +7 ~/datmo_ws/src/datmo/launch/test.launch
badd +14 ~/datmo_ws/src/datmo/src/main.cpp
badd +60 ~/datmo_ws/src/datmo/CMakeLists.txt
badd +9 ~/datmo_ws/src/datmo/launch/simulation.launch
badd +1 ~/datmo_ws/src/datmo/src/datmo.h
badd +3 ~/datmo_ws/src/localization/launch/mocap_localization.launch
badd +2 ~/datmo_ws/src/datmo/msg/Track.msg
badd +13 ~/datmo_ws/src/datmo/src/kalman-cpp/kalman.cpp
badd +66 ~/datmo_ws/src/datmo/src/ukf/filter_common.h
badd +33 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.cpp
badd +107 ~/datmo_ws/src/datmo/src/ukf/filter_base.cpp
badd +59 ~/datmo_ws/src/robot_localization/test/test_ukf.cpp
badd +1 ~/datmo_ws/src/datmo/.git/index
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
