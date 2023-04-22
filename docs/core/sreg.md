# Special Register

Special registers controll different aspects of core function. Accesible via `SRS` and `SRL` instructions.

## 0x0: PC

Maps to Program Counter. Can be used for jumps to address from register.

## 0x1: PRIV CTRL

Priviledged controls - Privileged mode has to be enabled in order to write to this register (in other case writes are ignored).

* Bit 0 - PRIV - Enables Privileged mode. Set to 1 at reset.
* Bit 1 - DATA_PAGE - Enables paging for data. (More in memory docs)
* Bit 2 - IRQ - Enables external interrupts. Automatically set to 0 when handling interrupt and set to 1 at `IRT` instruction.
* Bit 3 - LONG_PTR - Enables long pointer addressing (24-bit).

## 0x2 : JTR - JUMP TRIGGERED REGISTER

This is settings register that delays applying changes until first jump or write to pc happens.
Writes updates buffered values and reads present currently applied control state. Writes require Privileged mode.

* Bit 0 - INSTR_PAGE - Enables instruction paging. Resets to 1 in core 0.
* Bit 1 - TRAP_FLAG - Causes each instruction to raise interrupt.
* Bit 2 - LONG_PC - Enables 24-bit PC mode.

## 0x3 : IRQ PC

Is set to interrupted address at interrupt. At the end of ISR, pc should be set to that value.

## 0x4 : ALU FLAGS

Provides read and write access to ALU flags in (order LSB,..,MSB: Z, C, N, O, P)

## 0x5 : IRQ FLAGS

Read only. Information about interrapt cause. Mulitple bits can be high if interrups from multiple sources
happended in the same cycle.

* Bit 0 - EXT - External interrupt (and interrupts were enabled)
* Bit 1 - SYS - Caused by `SYS` instruction
* Bit 2 - TRAP - Caused by `TRAP` flag.
* Bit 3 - MEM - Caused by memory access (segmentation fault or error response on data bus)
* Bit 4 - ICI - Inter-core interupt. Notification from another core.

## 0x6: SCRATCH

Without assignment. Useful in ISRs for temporary storage without memory.

## 0x7: CPUID

Contains identification number of CPU, ISA, process and suppoerted features.

## 0x8: CORE ID

Identifiaction number of core. Only core 0 is enabled by default. Core 0 handles external interrupt requests.

## 0x9: ICINT SET

Triggers intercore interrupt for given core. Bit number indicates targt core (LSB - Core 0). Privileged.

## 0xA: ICINT RESET

Resets intercore interrupt for given core. If other core would set interrupt in the same cycle, reset is ignored. Privileged.

## 0xB: CORE DISABLE

Each bit disables coresponding core. Resets to all bits set apart from LSB (core 0). Privileged.

## 0xC: HIGH PC

Access to high part of LONG PC.

## 0xD: HIGH PC BUFF

Writes apply to high part of LONG PC at jumps/pc writes (that modifes lower part).
