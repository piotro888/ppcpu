start:
    ldi r0, 0b011 ; enable data paging
    srs r0, 1
    ldi r0, 0x004
    srs r0, 0x201 ; map page 1 to device address

    ; ldi r0, 0x1000
    ; srs r0, 0x200

    ; ldd r0, 0
    ; jal r6, putc
    ; sri r0, r0, 8
    ; jal r6, putc

    ; ; static test
    ; ldi r0, 0x0000
    ; std r0, 0x0
    ; ldi r0, 0x0000
    ; std r0, 0x2
    ; ldi r0, 0x0e00
    ; std r0, 0x4
    ; ldi r0, 0
    ; std r0, 6
    ; ldi r0, 0x0e
    ; std r0, 0x8
    ; ldi r0, 0
    ; std r0, 0xa
    ; srs r0, 2
    ; jmp start

    ; ldi r0, 0x000e
    ; std r0, 0x0
    ; ldi r0, 0x0000
    ; std r0, 0x2

    ; ldi r0, 0x000e
    ; std r0, 0x4
    ; ldi r0, 0x0000
    ; std r0, 0x6

    jal r6, print_welcome

    main_loop:
        ldi r0, 0x3e
        jal r6, putc

        jal r6, getc

        cmi r0, 0x70
        jeq dump_page

        cmi r0, 0x65
        jeq exec

        cmi r0, 0x6d
        jeq load_data_mem

        jmp main_loop

exec:
    ldi r0, 0b001 ; disable data paging
    srs r0, 1

    ; disable program paging and jump to true 0
    ldi r0, 0
    srs r0, 2
    jmp start

load_data_mem:
    ; read data size 0x0000
    jal r6, getc16
    mov r7, r0

    ; read start address (in 0xppPXXX)
    jal r6, getc16
    sri r3, r0, 12 ; page number
    ani r5, r0, 0x0fff ; local page addr

    jal r6, getc16
    sli r0, r0, 4
    orr r3, r3, r0 ; upper page number

    ; setup page0 for selected page load
    srs r3, 0x200

    mov r0, r5
    jal r6, putc

    ldi r1, 0 ; checksum
    
    load_loop:
        jal r6, getc16
        sto r0, r5, 0
        add r1, r1, r0s

        adi r5, r5, 2 
        cmi r5, 0x1000
        jne page_update_skip

            adi r3, r3, 1
            srs r3, 0x200
            ldi r5, 0

        page_update_skip:
        adi r7, r7, -2 
        cmi r7, 0

        jca load_loop_end  ; jgt unsigned
        jeq load_loop_end
        jmp load_loop
    load_loop_end:
    mov r0, r1
    jal r6, putc
    sri r0, r0, 8
    jal r6, putc
    jmp main_loop

dump_page:
    jal r6, getc16
    srs r0, 0x200

    ldi r3, 0

    dump_loop:
        ldo r0, r3, 0        
        jal r6, putc
        sri r0, r0, 8
        jal r6, putc

        ;adi r3, r3, 1 
        adi r3, r3, 2
        cmi r3, 0xffe
        jle dump_loop
    jmp main_loop

putc:
    putc_wait_ready:
        ldd r1, 0x1000 ; temporary serial device address
        cai r1, 0x2
        jeq putc_wait_ready
    std r0, 0x1004
    srs r6, 0

getc16:
    mov r2, r6
    jal r6, getc
    mov r4, r0
    jal r6, getc
    sli r0, r0, 8
    orr r0, r4, r0
    srs r2, 0

getc:
    getc_wait_ready:
        ldd r0, 0x1000
        cai r0, 0x1
        jeq getc_wait_ready
    ldd r0, 0x1002
    srs r6, 0

print_welcome:
    mov r5, r6
    ldi r0, 0x70
    jal r6, putc
    ldi r0, 0x70
    jal r6, putc
    ldi r0, 0x63
    jal r6, putc
    ldi r0, 0x70
    jal r6, putc
    ldi r0, 0x75
    jal r6, putc
    ldi r0, 0x20
    jal r6, putc
    ldi r0, 0x62
    jal r6, putc
    ldi r0, 0x6f
    jal r6, putc
    ldi r0, 0x6f
    jal r6, putc
    ldi r0, 0x74
    jal r6, putc
    ldi r0, 0x6c
    jal r6, putc
    ldi r0, 0x64
    jal r6, putc
    ldi r0, 0x0a
    jal r6, putc
    srs r5, 0