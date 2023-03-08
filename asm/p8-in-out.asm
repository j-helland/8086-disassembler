bits 16

; fixed port
in ax, 8
in al, 8
out 8, ax
out 8, al

; variable port
in ax, dx
in al, dx
out dx, ax
out dx, al
