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
$argadd src/cluster.hpp
$argadd src/datmo.cpp
$argadd src/datmo.hpp
$argadd launch/test.launch
$argadd CMakeLists.txt
edit src/cluster.cpp
set splitbelow splitright
set nosplitbelow
set nosplitright
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
setlocal fdl=99
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
1,6fold
10,14fold
15,22fold
23,29fold
89,112fold
113,117fold
31,119fold
151,175fold
176,178fold
121,182fold
200,201fold
202,203fold
183,216fold
217,306fold
313,314fold
315,316fold
317,318fold
308,320fold
329,332fold
333,336fold
321,340fold
349,352fold
362,374fold
412,416fold
410,417fold
421,423fold
424,426fold
428,430fold
431,433fold
341,453fold
471,475fold
454,481fold
482,547fold
567,571fold
583,587fold
548,591fold
592,604fold
605,612fold
624,626fold
627,629fold
632,634fold
635,637fold
642,644fold
645,647fold
648,650fold
651,653fold
641,655fold
613,658fold
659,700fold
701,742fold
743,747fold
749,789fold
790,823fold
824,848fold
849,902fold
925,930fold
903,933fold
963,969fold
973,976fold
972,977fold
934,980fold
985,989fold
981,993fold
1005,1007fold
994,1024fold
1039,1042fold
1036,1043fold
1047,1061fold
1063,1068fold
1025,1069fold
31
normal! zo
121
normal! zo
121
normal! zc
183
normal! zo
183
normal! zc
217
normal! zc
308
normal! zo
308
normal! zc
321
normal! zo
321
normal! zc
341
normal! zo
341
normal! zc
454
normal! zo
454
normal! zc
482
normal! zc
548
normal! zo
548
normal! zc
592
normal! zc
605
normal! zc
613
normal! zo
613
normal! zc
659
normal! zc
701
normal! zc
743
normal! zc
790
normal! zc
824
normal! zc
849
normal! zc
903
normal! zo
903
normal! zc
934
normal! zo
934
normal! zc
981
normal! zo
981
normal! zc
994
normal! zo
994
normal! zc
1025
normal! zo
1025
normal! zc
let s:l = 788 - ((724 * winheight(0) + 63) / 127)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
788
normal! 0
tabnext 1
badd +766 src/cluster.cpp
badd +55 src/cluster.hpp
badd +143 src/datmo.cpp
badd +58 src/datmo.hpp
badd +7 launch/test.launch
badd +1 CMakeLists.txt
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
