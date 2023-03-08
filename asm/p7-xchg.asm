bits 16

; reg to reg
xchg bl, cl
xchg bx, cx

; reg to reg with accumulator
xchg ax, bx
xchg bx, ax

; mem to reg
xchg bl, byte [bp]
xchg bl, byte [bp + si + 7]
xchg bl, byte [bp + si - 7]
xchg bl, byte [bp + si + 3458]
xchg bl, byte [128]
xchg bl, byte [3458]
xchg bx, word [bp + 16]
xchg bx, word [bp - 16]
xchg bx, word [bp + 3458]
xchg bx, word [128]
xchg bx, word [3458]

; reg to mem
xchg byte [bp], bl
xchg byte [bp + si + 7], bl
xchg byte [bp + si - 7], bl
xchg byte [bp + si + 3458], bl
xchg byte [128], bl
xchg byte [3458], bl
xchg word [bp + 16], bx
xchg word [bp - 16], bx
xchg word [bp + 3458], bx
xchg word [128], bx
xchg word [3458], bx
