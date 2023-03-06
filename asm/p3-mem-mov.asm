; ========================================================================
; LISTING 39
; ========================================================================

bits 16

; Register-to-register
mov si, bx
mov dh, al

; 8-bit immediate-to-register
mov cx, 12
mov cx, -12

; 16-bit immediate-to-register
mov dx, 3948
mov dx, -3948

; Source address calculation
mov al, [bx + si]
mov bx, [bp + di]
mov dx, [bp]

; Source address calculation plus 8-bit displacement
mov ah, [bx + si + 4]

; Source address calculation plus 16-bit displacement
mov al, [bx + si + 4999]

; Dest address calculation
mov [bx + di], cx
mov [bp + si], cl
mov [bp], ch