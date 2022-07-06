`include "config.v"

// Instruction fetch stage

module fetch (
    input i_clk,
    input i_rst,

    output [`RW-1:0] o_req_addr,
    output o_req_active,
    input [`I_SIZE-1:0] i_req_data,
    input i_req_data_valid,
    
    output reg o_submit,
    output reg [`I_SIZE-1:0] o_instr
);

reg [`RW-1:0] fetch_pc, next_fetch_pc;
wire [`RW-1:0] instr_imm = o_instr[31:16];

// when req_data_valid is set (and new pc is not yet ready), memory is not 
// accepting requests and it starts on next cycle
// NOTE: if too slow make sync pred or make o_req sync and set after req_valid
assign o_req_active = 1'b1 & ~i_rst;
assign o_req_addr = next_fetch_pc;

always @(posedge i_clk) begin
    if(i_rst) begin
        fetch_pc <= -`RW'b1; // start from addr 0
        o_submit<= 1'b0; // wait until first requst is completed
        o_instr <= `I_SIZE'b0;
    end else begin
        o_submit <= 1'b0;
        if(i_req_data_valid) begin
            // memory request completed, submit instruction
            o_instr <= i_req_data;
            fetch_pc <= next_fetch_pc;
            o_submit <= 1'b1;
        end
        // always request new instruction
        // request address is set on fetch_pc update
    end
end

// BRANCH PREDICTION / PC DECODE
always @(*) begin
    if (o_instr[6:0] == 7'h0e) begin
        if (o_instr[10:7] == 4'h0) begin
            // unconditional jump
            next_fetch_pc = instr_imm;
        end else begin
            // try to predict jump
            if (fetch_pc > instr_imm) begin
                // back jump (taken)
                next_fetch_pc = instr_imm;
            end else begin
                // forward jump (not taken)
                next_fetch_pc = fetch_pc + `RW'b1;
            end
        end
    end else if (o_instr[6:0] == 7'h0f) begin
        next_fetch_pc = instr_imm;
    end else begin
        next_fetch_pc = fetch_pc + `RW'b1;
    end
end

endmodule