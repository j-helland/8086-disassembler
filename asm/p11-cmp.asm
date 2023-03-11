bits 16

cmp bx, [bx+si]
cmp bx, [bp]

cmp si, 2
cmp bp, 2
cmp cx, 8

cmp bx, [bp + 0]
cmp cx, [bx + 2]
cmp bh, [bp + si + 4]
cmp di, [bp + di + 6]

cmp [bx+si], bx
cmp [bp], bx
cmp [bp + 0], bx
cmp [bx + 2], cx
cmp [bp + si + 4], bh
cmp [bp + di + 6], di

cmp byte [bx], 34
cmp word [4834], 29

cmp ax, [bp]
cmp al, [bx + si]

cmp ax, bx
cmp al, ah

cmp ax, 1000
cmp al, -30
cmp al, 9