# Instruction set 

PCPU ISA rev. 3

## Instruction format
Instruction format is fixed for all instructions. Single instruction has 32-bit width.

| Bits 31-16        | Bits 15-13      | Bits 12-10      | Bits 9-7        | Bits 6-0   |
|-------------------|-----------------|-----------------|-----------------|------------|
| immediate/address | second reg (se) | first reg (fi)  | target reg (tg) | opcode     |

## Instructions

| HEX | Name | Description | Symbolic Description
|-----|------|-------------|-------
|0    | NOP  | No Opertation | -
|1    | MOV  | Move first reg to target | tg <- fi
|2    | LDD  | Load direct from memory  | tg <- [addr][+0:+1]
|3    | LDO  | Load indirect with offset from memory | tg <- [fi + addr][+0:+1]
|4    | LDI  | Load immediate to register | tg <- imm
|5    | STD  | Store direct to memory     | [addr][+0:+1] <- fi
|6    | STO  | Store indirect with offest to memory | [addr+se][+0:+1] <- fi
|7    | ADD  | Add registers | tg <- fi + se
|8    | ADI  | Add immediate to register | tg <- fi + imm
|9    | ADC  | Add registers with carry  | tg <- fi + se + c
|A    | SUB  | Substract registers | tg <- fi - se
|B    | SUC  | Substract registers with carry  | tg <- fi - se - c
|C    | CMP  | Compare registers  | fi - se
|D    | CMI  | Compare register with immediate | fi - imm
|E    | JMP  | Jump (see jump conditions)      | jump if condition [pc <- imm]
|F    | JAL  | Jump and link | tg <- pc; pc <- imm
|10   | SRL  | Load from special register | tg <- sr[addr]
|11   | SRS  | Store to special register | sr[addr] <- sr
|12   | SYS  | Syscall - raises interrupt with SYS flag | -
|13   | AND  | And registers | tg <- fi & se
|14   | ORR  | Or registers | tg <- fi \| se
|15   | XOR  | Xor registers | tg <- fi ^ se
|16   | ANI  | And register with immediate | tg <- fi & imm
|17   | ORI  | Or register with immediate | tg <- fi \| imm
|18   | XOI  | Xor register with immediate | tg <- fi ^ imm
|19   | SHL  | Bit shift left | tg <- fi >> se
|1A   | SHR  | Bit shift right | tg <- fi << se
|1B   | CAI  | And-compare with immediate | fi & se
|1C   | MUL  | Lower part multiplication | tg <- fi * se
|1D   | DIV  | Unsigned division | tg <- fi / se
|1E   | IRT  | Return from interrupt | pc <- sreg 0x3 + set INT
|1F   | LD8  | LDD for zero extended 8 bits | tg <- {0, 8[addr]}
|20   | LO8  | LDO for zero extended 8 bits | tg <- {0, 8[fi + addr]}
|21   | SD8  | STD for 8 bits | [addr] <- 8{fi}
|22   | SO8  | SDO for 8 bits | [addr+se] <- 8{fi}
|23   | SLI  | Shift left by immediate | tg <- fi >> imm
|24   | SRI  | Shift right by immediate | tg <- fi << imm
|25   | SAR  | Arithmetic shift left | tg <- fi a>> nd
|26   | SAI  | Arithmetic shift left by immediate | tg <- fi a>> nd
|27   | SEX  | Sign extend (8b -> 16b) | tg <- sext fi
|28   | LLO  | Long pointer load with offset | tg <- [fi + {fi+1} + imm][+0:+1]
|29   | SLO  | Long pointer store with offset | [se + {se+1} + imm] <- fi
|2A   | LL8  | Long pointer load with offset 8 bit | tg <- {0, 8[fi + {fi+1} + imm]}
|2B   | SL8  | Long pointer store with offset 8 bit | 8[se + {se+1} + imm] <- fi
|2C   | MOD  | Unsigned modulo | tg <- fi / se

### Jump modes

Jump mode code is defined by bits 10-7 in case of `JMP` opcode.
| Jump code | ASM OPCODE | Description | Flags
| --------- | --------- | ----------- | -----
| 0 | JMP | Unconditional jump | 1
| 1 | JCA | Jump if carry | C
| 2 | JEQ | Jump if equal / zero | Z
| 3 | JLT | Jump if less than / negative | N
| 4 | JGT | Jump if greater than / greater than zero | ~(N\|Z)
| 5 | JLE | Jump if less or equal | N\|Z
| 6 | JGE | Jump if greater or equal | ~N
| 7 | JNE | Jump if not equal / not zero | ~Z
| 8 | JOVF | Jump on signed overflow | O
| 9 | JPAR | Jump if even numbers of bits is set in result | P
| A | JGTU | Jump if greater than (unsigned version of comparasion) | ~(C\|Z)
| B | JGEU | Jump if greater or equal (unsigned version of comparasion) | ~C
| C | JLEU | Jump if less or equal (unsigned version of comparasion) | C\|Z

All reg-reg-reg/reg-reg-imm (apart from MUL, DIV and MOD) and compare operations update all flags

## Note

Memory is 8-bit addressed (in older cpus, memory was word (16-bit) addressed, but it could be disabled by STDMEM Sreg flag).
