bits 16

; register
push ax
push bx
push cx
push dx
push di
push si
push bp
push sp

; memory
push word [bx]
push word [bx + si + 7]
push word [bx + si - 24]
push word [bx + si + 3458]
