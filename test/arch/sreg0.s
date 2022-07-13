ldi r0, 2
srl r1, 0
cmi r1, 1
jne err
ldi r1, 8
srs r1, 0

err:
ldi r0, 4
jmp err

;pc = 8
ldi r0, 5
jal r6, next
nop

next:
cmi r6, 10
jne err
srl r0, 0
adi r0, r0, 4
srs r0, 0

jmp err
jmp pass
jmp err

.org 0x1000
pass: