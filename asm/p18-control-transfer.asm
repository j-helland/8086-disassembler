bits 16

call [39201]
call [bp - 100]
call sp
call ax

jmp ax
jmp di
jmp [12]
jmp [4395]

ret -7
ret 500
ret

label:
je label
jl label
jle label
jb label
jbe label
jp label
jo label
js label
jne label
jnl label
jg label
jnb label
ja label
jnp label
jno label
jns label
loop label
loopz label
loopnz label
jcxz label

int 13
int 3

into
iret

clc
cmc
stc
cld
std
cli
sti
hlt
wait

lock not byte [bp + 9905]
lock xchg [100], al