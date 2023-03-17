bits 16

not ah
not bl
not sp
not si
not word [bp]
not byte [bp + 9905]

shl ah, 1
shr ax, 1
sar bx, 1
rol cx, 1
ror dh, 1
rcl sp, 1
rcr bp, 1

shl word [bp + 5], 1
shr byte [bx + si - 199], 1
sar byte [bx + di - 300], 1
rol word [bp], 1
ror word [4938], 1
rcl byte [3], 1
rcr word [bx], 1

shl ah, cl
shr ax, cl
sar bx, cl
rol cx, cl
ror dh, cl
rcl sp, cl
rcr bp, cl

shl word [bp + 5], cl
shr word [bx + si - 199], cl
sar byte [bx + di - 300], cl
rol byte [bp], cl
ror byte [4938], cl
rcl byte [3], cl
rcr word [bx], cl

and al, ah
and ch, cl
and bp, si
and di, sp
and al, 93
and ax, 20392
and [bp + si + 10], ch
and [bx + di + 1000], dx
and bx, [bp]
and cx, [4384]
and byte [bp - 39], 239
and word [bx + si - 4332], 10328

test bx, cx
test dh, [bp + 390]
test [bp + 2], si
test bl, 20
test byte [bx], 34
test ax, 23909

or al, ah
or ch, cl
or bp, si
or di, sp
or al, 93
or ax, 20392
or [bp + si + 10], ch
or [bx + di + 1000], dx
or bx, [bp]
or cx, [4384]
or byte [bp - 39], 239
or word [bx + si - 4332], 10328

xor al, ah
xor ch, cl
xor bp, si
xor di, sp
xor bx, 128
xor bl, 128
xor al, 93
xor ax, 20392
xor [bp + si + 10], ch
xor [bx + di + 1000], dx
xor bx, [bp]
xor cx, [4384]
xor byte [bp - 39], 239
xor word [bx + si - 4332], 10328
