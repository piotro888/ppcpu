// common values for cpu

// register and data width bits
`ifndef CONFIG_H
`define CONFIG_H

`define RW 16

`define REGNO 8
`define REGNO_LOG 3

`define ADDR_W 16
`define EXT_ADDR_W 24

`define I_SIZE 32

`define ADDR_BYTES 2

`define WB_ADDR_W 24

// -- ALU --
`define ALU_MODE_W 4
`define ALU_MODE_L_PASS `ALU_MODE_W'b0
`define ALU_MODE_R_PASS `ALU_MODE_W'b1
`define ALU_MODE_ADD `ALU_MODE_W'b10
`define ALU_MODE_SUB `ALU_MODE_W'b11
`define ALU_MODE_AND `ALU_MODE_W'b100
`define ALU_MODE_OR `ALU_MODE_W'b101
`define ALU_MODE_XOR `ALU_MODE_W'b110
`define ALU_MODE_SHL `ALU_MODE_W'b111
`define ALU_MODE_SHR `ALU_MODE_W'b1000
`define ALU_MODE_MUL `ALU_MODE_W'b1001
`define ALU_MODE_DIV `ALU_MODE_W'b1010
`define ALU_MODE_ASHR `ALU_MODE_W'b1011
`define ALU_MODE_SEXT `ALU_MODE_W'b1100
`define ALU_MODE_MOD `ALU_MODE_W'b1101
`define ALU_FLAG_W 3
`define ALU_FLAG_CNT 5
`define ALU_FLAG_Z `ALU_FLAG_W'b0
`define ALU_FLAG_C `ALU_FLAG_W'b1
`define ALU_FLAG_N `ALU_FLAG_W'b10
`define ALU_FLAG_O `ALU_FLAG_W'b11
`define ALU_FLAG_P `ALU_FLAG_W'b100

// -- SHARED DECODE --
`define JUMP_CODE_W 5 // 4+1
`define JUMP_CODE_BIT_EN 4

`endif