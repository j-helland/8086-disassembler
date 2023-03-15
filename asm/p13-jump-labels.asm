
bits 16

zero_label:
jne zero_label

jne first_label
jne first_label
jne second_label
mov bl, 128
mov bx, 128
mov [bp + 8], byte 128
mov [bp + 8], word 128
mov [bp + 12800], word 128
add bx, 128
add bx, 12800

first_label:
jne first_label
jne second_label

second_label:
jne first_label
jne second_label
