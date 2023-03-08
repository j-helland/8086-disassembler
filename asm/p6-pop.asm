bits 16

; reg pop
pop ax
pop bx
pop cx
pop dx
pop di
pop si
pop bp
pop sp

; mem pop
pop word [bx]
pop word [bx + si + 7]
pop word [128]
pop word [3458]