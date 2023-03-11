bits 16

test_label0:
jnz test_label1
jnz test_label0
test_label1:
jnz test_label0
jnz test_label1

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