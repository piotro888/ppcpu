# ppcpu - Pipelined PCPU

This is a third revision of 16-bit PCPU processor with custom, deisgned from scratch RISC-type architecture.

It is a direct upgrade to [pcpu](https://github.com/piotro888/pcpu) project - it uses almost the same instrucion set, but inside is cleaner, more modern and advanced pipelined procesor (making it 40x faster).
Additionaly it features caches and multi-core operation.

## Features
* 16-bit custom RISC type architecture
* 4-stage pipelined microarchitecture
* Multi-core operation
* Memory paging, virtualization and protection
* 24-bit address space of Compressed Wishbone Bus
* Internal and external interrupts
* Bootloader
* Instruction and data caches

## Processor Structure

`rtl/` directory contains code of dual-core PPCPU processor, with data and instruction caches.

### Core

Architecture of core pipeline:
```nim
  [ FETCH             ]      [ DECODE                         ]     [ EXECUTE                 ]      [ MEMORY/REGISTER WRITEBACK   ]
I [ BRANCH PREDICTOR  ]      [ INSTRUCTION TO CONTROL SIGNALS ]     [ [REGISTER FILE]->[ALU]->]      [ REGISTER FILE WRITEBACK     ] D
N [                   ]      [                                ]     [             IMM->[ALU]  ]      [ MEMORY ACCESS STATE MACHINE ] A
S [                   ]  ==> [                                ] ==> [ BRANCH RESOLUTION       ]  ==> [                             ] T
T [                   ]      [                                ]     [ HAZARD DETECION         ]      [                             ] A
R [                   ]      [                                ]     [ SREGS-CONTROL REGISTERS ]      [                             ]
  [                   ]      [                                ]     [ MUL/DIV UNIT            ]      [                             ]
```
More details about supported instructions and SREG functions are located inside `docs/` directory

### Inter-Core connections

```nim
  [CORE 0]<->[I&D MMU]<->[                      ]     [                    ] <>  [    MAIN OUTSIDE CW BUS            ]
                         [  INNER_INTERCONNECT  ] <-> [ OUTER_INTERCONNECT ] 
  [CORE 1]<->[I&D MMU]<->[                      ]     [ +CLOCKING & CDC    ] <>  [EMBEDDED MODULE - SPI TO WB BRIDGE,]
                          ^v         ^v        ^v                                [INTERNAL RAM, GPIO                 ]
                      [ICACHE 0] [ICACHE 1] [DCACHE]

  | CORE | INTERNAL BUS             |           WISHBONE BUS    |  COMPRESSED WISHBONE BUS                            |
```

### SoC

PPCPU processor is located inside `soc/` module, that runs inside FPGA on my devboard.

SoC moudle decompresses CW bus back to Wishbone bus and connects it to following peripherals
* SDRAM controller (most of address space)
* VGA graphics card with text mode
* ROM with bootloader, that listens for commands on serial port
* UART interface
* I2C interface
* SPI interface
* Interrupt controller
* Programmable timers
* PS/2 interface

## Interesting links
* [pcsn](https://github.com/piotro888/pcsn) - PCPU SoC emulator
* [llvm-pcpu](https://github.com/piotro888/llvm-project-pcpu) - full LLVM toolchain retargeted for PCPU
* [piOS](https://github.com/piotro888/piOS) - full Operating System for PCPU!
* [ppcpu_caravel](https://github.com/piotro888/ppcpu_caravel) - silicon design of ppcpu core (with CW bus and embedded mode)

-----
`(C) 2022-2023 Piotr Węgrzyn`
