bits 16

; mem to reg
add bx, [bx+si]
add bx, [bp]

; imm to reg
add si, 2
add bp, 2
add cx, 8

; mem to reg with displacement
add bx, [bp + 0]
add cx, [bx + 2]
add bh, [bp + si + 4]
add di, [bp + di + 6]

; reg to mem
add [bx+si], bx
add [bp], bx

; reg to mem with displacement
add [bp + 0], bx
add [bx + 2], cx
add [bp + si + 4], bh
add [bp + di + 6], di

; imm to mem
add byte [bx], 34
add word [bp + si + 1000], 29

; mem to acc
add ax, [bp]
add al, [bx + si]

; reg to acc
add ax, bx
add al, ah

; imm to acc
add ax, 1000
add al, -30
add al, 9