`include "config.v"

module alu (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input [`RW-1:0] i_l, i_r,
    output [`RW-1:0] o_out,

    input [`ALU_MODE_W-1:0] i_mode,
    input wire i_carry,

    output reg [`ALU_FLAG_CNT-1:0] o_flags
);

reg [`RW:0] outc;
assign o_out = outc[`RW-1:0];

always @* begin
    case (i_mode)
        default:
            outc = {1'b0, i_l};
        `ALU_MODE_R_PASS:
            outc = {1'b0, i_r};
        `ALU_MODE_ADD:
            outc = i_l + i_r + {15'b0, i_carry};
        `ALU_MODE_SUB:
            outc = i_l - i_r - {15'b0, i_carry};
        `ALU_MODE_AND:
            outc = {1'b0, i_l & i_r};
        `ALU_MODE_OR:
            outc = {1'b0, i_l | i_r};
        `ALU_MODE_XOR:
            outc = {1'b0, i_l ^ i_r};
        `ALU_MODE_SHL:
            outc = {1'b0, i_l} << {1'b0, i_r};
        `ALU_MODE_SHR:
            outc = {1'b0, i_l >> i_r};
        `ALU_MODE_MUL: // temporary
            outc = {1'b0, i_l*i_r};//16'b0};
        `ALU_MODE_DIV: // temporary
            outc = {1'b0, 16'b0};
        `ALU_MODE_ASHR:
            outc = {1'b0, (i_l >> i_r) | ({`RW{i_l[`RW-1]}} << (`RW-i_r))};
        `ALU_MODE_SEXT:
            outc = {1'b0, {8{i_l[7]}}, i_l[7:0]};
    endcase

    o_flags[`ALU_FLAG_Z] = ~(|outc[`RW-1:0]);
    o_flags[`ALU_FLAG_C] = outc[`RW];
    o_flags[`ALU_FLAG_N] = outc[`RW-1];
    o_flags[`ALU_FLAG_O] = (i_l[`RW-1] ^ (i_r[`RW-1]^(i_mode == `ALU_MODE_SUB))) & ((i_l[`RW-1]^outc[`RW-1]));
    o_flags[`ALU_FLAG_P] = ^outc[`RW-1:0];
end

endmodule
