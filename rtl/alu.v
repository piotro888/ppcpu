`include "config.v"

`define ALU_MODE_W 3
`define ALU_MODE_L_PASS `ALU_MODE_W'b0
`define ALU_MODE_ADD `ALU_MODE_W'b1
`define ALU_MODE_SUB `ALU_MODE_W'b10
`define ALU_MODE_AND `ALU_MODE_W'b11
`define ALU_MODE_OR `ALU_MODE_W'b100
`define ALU_MODE_XOR `ALU_MODE_W'b101
`define ALU_MODE_SHL `ALU_MODE_W'b110
`define ALU_MODE_SHR `ALU_MODE_W'b111

module alu (
    input [`RW-1:0] i_l, i_r,
    output reg [`RW-1:0] o_out,

    input [`ALU_MODE_W-1:0] i_mode
);

always @* begin
    case (i_mode)
        default:
            o_out = i_l;
        `ALU_MODE_ADD:
            o_out = i_l + i_r;
        `ALU_MODE_SUB:
            o_out = i_l - i_r;
        `ALU_MODE_AND:
            o_out = i_l & i_r;
        `ALU_MODE_OR:
            o_out = i_l | i_r;
        `ALU_MODE_XOR:
            o_out = i_l ^ i_r;
        `ALU_MODE_SHL:
            o_out = i_l << i_r;
        `ALU_MODE_SHR:
            o_out = i_l >> i_r;
    endcase
end

endmodule