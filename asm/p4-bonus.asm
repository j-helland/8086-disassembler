; ========================================================================
; LISTING 40
; ========================================================================

bits 16

; Signed displacements
mov ax, [bx + di - 37]
mov [si - 300], cx
mov dx, [bx - 32]

; Direct address
mov bp, [5]
mov bx, [3458]

; Explicit sizes
mov [bp + di], byte 7
mov [di + 901], word 347

; Memory-to-accumulator test
mov ax, [2555]
mov ax, [16]

; Accumulator-to-memory test
mov [2554], ax
mov [15], ax