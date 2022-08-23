`include "config.v"

// Instruction fetch stage v2

module fetch (
    input i_clk,
    input i_rst,

    output [`RW-1:0] mem_addr, // address must be valid only if submit is set (registered on other end)
    input [`I_SIZE-1:0] mem_data,
    input mem_ack,
    output mem_submit, // pipelined submit signal
    
    input i_next_ready,
    output reg o_submit,
    input i_flush,
 
    output reg [`I_SIZE-1:0] o_instr,
    output reg o_jmp_predict,

    input [`RW-1:0] i_exec_pc
);

assign mem_addr = (pc_reset_override ? `RW'b0 : ((i_flush | pc_flush_override) ? i_exec_pc : pred_pc));
assign mem_submit = (((mem_ack & ~out_buffer_valid & i_next_ready & ~disable_prediction) | pc_reset_override | (pc_flush_override & ~instr_wait) | (out_buffer_valid & out_buff_read & ~disable_prediction))) & ~i_rst;

reg pc_reset_override;
always @(posedge i_clk) begin
    if (i_rst) begin
        pc_reset_override <= 1'b1;
    end else if (mem_submit) begin
        pc_reset_override <= 1'b0;
    end
end

reg pc_flush_override;
always @(posedge i_clk) begin
    if (i_rst | mem_submit)
        pc_flush_override <= 1'b0;
    else if (i_flush)
        pc_flush_override <= 1'b1;
end

reg flush_event_invalidate;
always @(posedge i_clk) begin
    if (i_rst | mem_ack)
        flush_event_invalidate <= 1'b0;
    else if (i_flush & instr_wait & ~mem_ack)
        flush_event_invalidate <= 1'b1;
end

reg instr_wait;
always @(posedge i_clk) begin
    if (i_rst)
        instr_wait <= 1'b0;
    else if (mem_submit)
        instr_wait <= 1'b1;
    else if (mem_ack)
        instr_wait <= 1'b0;
end

wire out_ready = out_buffer_valid | mem_ack;
wire [`I_SIZE-1:0] out_instr = (out_buffer_valid ? out_buffer_data_instr : mem_data);
wire out_jmp_predict = (out_buffer_valid ? out_buffer_data_pred : current_req_branch_pred);
wire submitable = out_ready & i_next_ready & ~(i_flush | flush_event_invalidate);
always @(posedge i_clk) begin
    o_submit <= submitable;
    if (submitable) begin
        o_instr <= out_instr;
        o_jmp_predict <= out_jmp_predict;
    end
end

reg [`RW-1:0] prev_request_pc;
always @(posedge i_clk) begin
    if(mem_submit)
        prev_request_pc <= mem_addr;
end

wire current_req_branch_pred = (mem_ack ? branch_pred_res : prev_req_branch_pred);
reg prev_req_branch_pred;
always @(posedge i_clk) begin
    if(mem_ack)
        prev_req_branch_pred <= branch_pred_res;
end

wire [`RW-1:0] pred_pc = (branch_pred_res ? branch_pred_imm : prev_request_pc + `RW'b1);

wire [`I_SIZE-1:0] branch_pred_instr = (out_buffer_valid ? out_buffer_data_instr : mem_data);
wire [`RW-1:0] branch_pred_imm = branch_pred_instr[`I_SIZE-1:`I_SIZE-`RW];

// disable predicting after sys, irt, srs0 instructions (always ends with flush)
wire disable_prediction = (branch_pred_instr[6:0] == 7'h12) || (branch_pred_instr[6:0] == 7'h1e) || (branch_pred_instr[6:0] == 7'h11 && ~(|branch_pred_imm));
reg branch_pred_res;

// BRANCH PREDICTION / PC DECODE
always @(*) begin
    if (pc_reset_override) begin
        branch_pred_res = 1'b0;
    end else if (branch_pred_instr[6:0] == 7'h0e) begin
        if (branch_pred_instr[10:7] == 4'h0) begin
            // unconditional jump
            branch_pred_res = 1'b1;
        end else begin
            // try to predict jump
            if (prev_request_pc > branch_pred_imm) begin
                // back jump (taken)
                branch_pred_res = 1'b1;
            end else begin
                // forward jump (not taken)
                branch_pred_res = 1'b0;
            end
        end
    end else if (branch_pred_instr[6:0] == 7'h0f) begin
        branch_pred_res = 1'b1;
    end else begin
        branch_pred_res = 1'b0;
    end
end

reg [`I_SIZE-1:0] out_buffer_data_instr;
reg out_buffer_data_pred;
reg out_buffer_valid;

wire out_buff_write = mem_ack & ~i_next_ready & ~(i_flush | flush_event_invalidate);
wire out_buff_read = out_buffer_valid & i_next_ready;
always @(posedge i_clk) begin
    if (i_rst) begin
        out_buffer_valid <= 1'b0;
    end else if (i_flush) begin
        out_buffer_valid <= 1'b0;
    end else if (out_buff_write) begin
        out_buffer_data_instr <= mem_data;
        out_buffer_data_pred <= current_req_branch_pred;
        out_buffer_valid <= 1'b1;
    end else if (out_buffer_valid & i_next_ready) begin
        out_buffer_valid <= 1'b0;
    end
end

endmodule
