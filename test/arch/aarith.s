start:
    ldi r0, 0x008a
    sex r1, r0
    cmi r1, 0xff8a
    jne err

    ldi r0, 0b1011
    sri r0, r0, 2
    sli r0, r0, 1
    cmi r0, 0b100
    jne err

    ldi r0, 0x8003
    sai r0, r0, 1
    cmi r0, 0xc001
    jne err
    ldi r0, 0x8003
    sai r0, r0, 2
    cmi r0, 0xe000
    jne err
    ldi r0, 0x4003
    ldi r1, 1
    sar r0, r0, r1
    cmi r0, 0x2001
    jne err

    jmp pass    
err:
    jmp err

.org 0x1000
pass:
    jmp pass