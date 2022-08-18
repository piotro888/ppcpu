ref0:
; resetv
jmp page0
; intv 
jmp inth

page0:
; init counters
ldi r5, 0
ldi r6, 0
ldi r7, 0

; setup page table for 0->page1
ldi r0, 1
srs r0, 0x100

; init exec info
ldi r1, 0 ; p1
ldi r2, 0 ; cnt 0

; enable instr paging
ldi r0, 1
srs r0, 2
jmp ref0

jmp err ; ASSERT_NOT_REACHED

inth:
adi r2, r2, 1
cmi r2, 100
jne pend_skip
cmi r1, 1
jeq final
ldi r1, 1
ldi r2, 0
pend_skip:
cmi r1, 0
jeq lpl_skip
; load p2 paging
ldi r0, 2
srs r0, 0x100
lpl_skip:
; reenable paging
ldi r0, 1
srs r0, 2
jmp ref0

jmp err ; ASSERT_NOT_REACHED


final:
cmi r5, 300
jne err
cmi r6, 100
jne err
cmi r7, 100
jne err
jmp pass

err:
jmp err

.org 0x400
pass:
jmp pass

.org 0x800
page1:
adi r5, r5, 1
adi r6, r6, 1
sys
errp1:
jmp errp1 ; ASSERT_NOT_REACHED

.org 0x1000
page2:
adi r5, r5, 2
adi r7, r7, 1
sys
errp2:
jmp errp2 ; ASSERT_NOT_REACHED