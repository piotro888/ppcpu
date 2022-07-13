start:
    ldi r0, 1
    jal r6, inc
    add r0, r0, r0
    jal r6, inc
    cmi r0, 5
    jeq pass
fail:
    jmp fail

inc:
    adi r0, r0, 1
    srs r6, 0

.org 0x1000
pass:
    jmp pass