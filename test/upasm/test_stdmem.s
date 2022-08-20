ldi r0, 0x1234
std r0, 0
sd8 r0, 2
ldi r0, 0

ld8 r0, 0
cmi r0, 0x34
jne err

ld8 r0, 1
cmi r0, 0x12
jne err

ldd r0, 0
cmi r0, 0x1234
jne err

ldi r0, 0xff
sd8 r0, 1

ldi r0, 0
ldd r0, 0
cmi r0, 0xff34
jne err

ldi r0, 0xee
sd8 r0, 0
ldd r0, 0
cmi r0, 0xffee
jne err

jmp pass

err:
jmp err

.org 0x400
pass:
jmp pass