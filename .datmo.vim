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
set stal=2
tabnew
tabnew
tabrewind
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
exe '1resize ' . ((&lines * 35 + 37) / 74)
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
let s:l = 1 - ((0 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
lcd ~/datmo_ws/src/datmo
wincmd w
argglobal
if bufexists("~/datmo_ws/src/datmo/src/ukf/filter_base.cpp") | buffer ~/datmo_ws/src/datmo/src/ukf/filter_base.cpp | else | edit ~/datmo_ws/src/datmo/src/ukf/filter_base.cpp | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
1,31fold
56,58fold
61,62fold
65,112fold
115,117fold
120,122fold
125,127fold
130,132fold
135,137fold
140,142fold
145,147fold
154,157fold
159,161fold
152,162fold
164,166fold
150,167fold
170,172fold
175,177fold
180,182fold
185,187fold
193,197fold
190,198fold
201,205fold
215,222fold
210,225fold
43,226fold
1
normal! zc
43
normal! zo
150
normal! zo
150
normal! zc
170
normal! zc
175
normal! zc
185
normal! zc
190
normal! zo
190
normal! zc
let s:l = 51 - ((10 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
51
normal! 05|
lcd ~/datmo_ws/src/datmo
wincmd w
argglobal
if bufexists("~/datmo_ws/src/datmo/src/ukf/filter_base.h") | buffer ~/datmo_ws/src/datmo/src/ukf/filter_base.h | else | edit ~/datmo_ws/src/datmo/src/ukf/filter_base.h | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
1,31fold
92,94fold
97,99fold
106,107fold
62,108fold
112,326fold
53,328fold
1
normal! zc
53
normal! zo
62
normal! zo
62
normal! zc
let s:l = 262 - ((26 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
262
normal! 075|
lcd ~/datmo_ws/src/datmo
wincmd w
exe '1resize ' . ((&lines * 35 + 37) / 74)
exe '2resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 2resize ' . ((&columns * 127 + 127) / 255)
exe '3resize ' . ((&lines * 35 + 37) / 74)
exe 'vert 3resize ' . ((&columns * 127 + 127) / 255)
tabnext
edit ~/datmo_ws/src/datmo/src/ukf/ukf.cpp
set splitbelow splitright
set nosplitbelow
set nosplitright
wincmd t
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
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
117,120fold
105,121fold
134,136fold
138,140fold
142,144fold
131,145fold
129,146fold
183,185fold
191,197fold
206,212fold
178,213fold
218,220fold
229,232fold
237,241fold
258,260fold
263,265fold
256,266fold
252,267fold
271,289fold
93,290fold
362,365fold
370,372fold
378,381fold
292,398fold
50,400fold
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
117
normal! zo
129
normal! zo
131
normal! zo
134
normal! zo
138
normal! zo
142
normal! zo
129
normal! zc
178
normal! zo
183
normal! zo
206
normal! zo
218
normal! zo
229
normal! zo
237
normal! zo
252
normal! zo
256
normal! zo
258
normal! zo
263
normal! zo
292
normal! zo
362
normal! zo
370
normal! zo
378
normal! zo
let s:l = 386 - ((50 * winheight(0) + 35) / 71)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
386
normal! 0
lcd ~/datmo_ws/src/datmo
tabnext
edit ~/datmo_ws/src/datmo/launch/test.launch
set splitbelow splitright
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
exe 'vert 1resize ' . ((&columns * 127 + 127) / 255)
exe 'vert 2resize ' . ((&columns * 127 + 127) / 255)
argglobal
if bufexists("~/datmo_ws/src/datmo/launch/test.launch") | buffer ~/datmo_ws/src/datmo/launch/test.launch | else | edit ~/datmo_ws/src/datmo/launch/test.launch | endif
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
1
normal! zo
let s:l = 7 - ((6 * winheight(0) + 35) / 71)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
7
normal! 075|
lcd ~/datmo_ws/src/datmo
wincmd w
argglobal
if bufexists("~/datmo_ws/src/datmo/src/datmo.cpp") | buffer ~/datmo_ws/src/datmo/src/datmo.cpp | else | edit ~/datmo_ws/src/datmo/src/datmo.cpp | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
22,31fold
3,33fold
36,41fold
35,42fold
69,73fold
75,78fold
94,97fold
101,103fold
91,104fold
112,115fold
111,116fold
117,120fold
108,121fold
125,127fold
133,143fold
132,145fold
149,153fold
148,154fold
168,180fold
182,186fold
162,187fold
208,214fold
53,215fold
216,218fold
43,221fold
249,255fold
236,258fold
222,261fold
271,273fold
270,274fold
280,285fold
279,286fold
313,315fold
307,317fold
320,323fold
339,342fold
336,343fold
333,345fold
331,347fold
363,366fold
367,370fold
360,373fold
352,375fold
263,377fold
388,395fold
379,396fold
43
normal! zo
53
normal! zo
let s:l = 124 - ((33 * winheight(0) + 35) / 71)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
124
normal! 028|
lcd ~/datmo_ws/src/datmo
wincmd w
exe 'vert 1resize ' . ((&columns * 127 + 127) / 255)
exe 'vert 2resize ' . ((&columns * 127 + 127) / 255)
tabnext 1
set stal=1
badd +33 ~/datmo_ws/src/datmo/src/ukf/ukf.cpp
badd +263 ~/datmo_ws/src/datmo/src/datmo.cpp
badd +20 ~/datmo_ws/src/datmo/src/datmo.hpp
badd +803 ~/datmo_ws/src/datmo/src/cluster.cpp
badd +65 ~/datmo_ws/src/datmo/src/cluster.hpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.cpp
badd +1 ~/datmo_ws/src/datmo/src/l_shape_tracker.hpp
badd +252 ~/datmo_ws/src/datmo/src/ukf/filter_base.h
badd +43 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.h
badd +85 ~/datmo_ws/src/datmo/src/ukf/ukf.h
badd +9 ~/datmo_ws/src/datmo/launch/sim_test.launch
badd +8 ~/datmo_ws/src/datmo/launch/test.launch
badd +14 ~/datmo_ws/src/datmo/src/main.cpp
badd +60 ~/datmo_ws/src/datmo/CMakeLists.txt
badd +9 ~/datmo_ws/src/datmo/launch/simulation.launch
badd +1 ~/datmo_ws/src/datmo/src/datmo.h
badd +3 ~/datmo_ws/src/localization/launch/mocap_localization.launch
badd +2 ~/datmo_ws/src/datmo/msg/Track.msg
badd +13 ~/datmo_ws/src/datmo/src/kalman-cpp/kalman.cpp
badd +47 ~/datmo_ws/src/datmo/src/ukf/filter_common.h
badd +33 ~/datmo_ws/src/datmo/src/ukf/filter_utilities.cpp
badd +216 ~/datmo_ws/src/datmo/src/ukf/filter_base.cpp
badd +59 ~/datmo_ws/src/robot_localization/test/test_ukf.cpp
badd +0 ~/datmo_ws/src/datmo/.git/index
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
