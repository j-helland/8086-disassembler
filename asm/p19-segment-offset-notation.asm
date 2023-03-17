bits 16

mov bx, cx

;mov al, cs:[bx + si]
;mov bx, ds:[bp + di]
;mov dx, es:[bp]
;mov ah, ss:[bx + si + 4]
;
;and ss:[bp + si + 10], ch
;or ds:[bx + di + 1000], dx
;xor bx, es:[bp]
;cmp cx, es:[4384]
;test byte cs:[bp - 39], 239
;sbb word cs:[bx + si - 4332], 10328
;
;lock not byte CS:[bp + 9905]