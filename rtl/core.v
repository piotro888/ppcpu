`include "config.v"

module core (
    input i_clk,
    input i_rst,

    // fetch input singals
    output [`RW-1:0] o_req_addr,
    output o_req_active,
    input [`I_SIZE-1:0] i_req_data,
    input i_req_data_valid,

    output [`RW-1:0] dbg_r0, dbg_pc,

    // data memory connections
    output [`RW-1:0] o_mem_addr, o_mem_data,
    input [`RW-1:0] i_mem_data,
    output o_mem_req, o_mem_we,
    input i_mem_ack,

    input i_irq
);

// --- CPU INTERNAL CONNECTIONS ---

wire fetch_decode_next_ready;
wire fetch_decode_submit;
wire [`I_SIZE-1:0] fetch_decode_d_instr;
wire fetch_decode_jmp_pred;
wire execute_fetch_pc_update;
wire [`RW-1:0] execute_fetch_pc;
wire fde_pipeline_flush;

// Pipeline stage 0 - FETCH
fetch fetch (.i_clk(i_clk), .i_rst(i_rst), .o_req_active(o_req_active), .o_req_addr(o_req_addr),
    .i_req_data(i_req_data), .i_req_data_valid(i_req_data_valid), .i_next_ready(fetch_decode_next_ready),
    .o_submit(fetch_decode_submit), .o_instr(fetch_decode_d_instr), .o_jmp_predict(fetch_decode_jmp_pred),
    .i_exec_pc(execute_fetch_pc), .i_flush(fde_pipeline_flush));

wire decode_execute_next_ready;
wire decode_execute_submit;
wire [`I_SIZE-17:0] de_imm_pass;
wire de_jmp_pred;

wire dec_pc_inc, dec_pc_ie;
wire dec_r_bus_imm;
wire [`ALU_MODE_W-1:0] dec_alu_mode;
wire dec_alu_carry_en, dec_alu_flags_ie;
wire [`REGNO_LOG-1:0] dec_l_reg_sel, dec_r_reg_sel; 
wire [`REGNO-1:0] dec_rf_ie;
wire [`JUMP_CODE_W-1:0] dec_jump_cond_code;
wire dec_mem_access, dec_mem_we;
wire [1:0] dec_used_operands;
wire dec_sreg_load, dec_sreg_store, dec_sreg_jal_over, dec_sreg_irt;

// Pipeline stage 1 - DECODE
decode decode(.i_clk(i_clk), .i_rst(i_rst), .o_ready(fetch_decode_next_ready), .o_submit(decode_execute_submit),
    .i_next_ready(decode_execute_next_ready), .i_instr_l(fetch_decode_d_instr[15:0]), .i_imm_pass(fetch_decode_d_instr[`I_SIZE-1:16]),
    .o_imm_pass(de_imm_pass), .oc_pc_inc(dec_pc_inc), .oc_pc_ie(dec_pc_ie), .oc_r_bus_imm(dec_r_bus_imm), .oc_alu_mode(dec_alu_mode),
    .oc_alu_flags_ie(dec_alu_flags_ie), .oc_alu_carry_en(dec_alu_carry_en), .oc_l_reg_sel(dec_l_reg_sel), .oc_r_reg_sel(dec_r_reg_sel),
    .oc_rf_ie(dec_rf_ie), .i_submit(fetch_decode_submit), .oc_jump_cond_code(dec_jump_cond_code), .i_jmp_pred_pass(fetch_decode_jmp_pred),
    .o_jmp_pred_pass(de_jmp_pred), .i_flush(fde_pipeline_flush), .oc_mem_access(dec_mem_access), .oc_mem_we(dec_mem_we),
    .oc_used_operands(dec_used_operands), .oc_sreg_load(dec_sreg_load), .oc_sreg_store(dec_sreg_store), .oc_sreg_jal_over(dec_sreg_jal_over),
    .oc_sreg_irt(dec_sreg_irt));

wire [`RW-1:0] ew_data;
wire [`RW-1:0] ew_addr;
wire [`REGNO-1:0] ew_reg_ie;
wire ew_mem_access, ew_mem_we;
wire [`REGNO-1:0] we_reg_ie;
wire [`RW-1:0] we_reg_data;
wire ew_submit;
wire ew_next_ready;

// Pipeline stage 2 - EXECUTE
execute execute(.i_clk(i_clk), .i_rst(i_rst), .o_ready(decode_execute_next_ready), .i_imm(de_imm_pass), .c_pc_inc(dec_pc_inc),
    .c_pc_ie(dec_pc_ie), .c_r_bus_imm(dec_r_bus_imm), .c_alu_mode(dec_alu_mode), .c_alu_flags_ie(dec_alu_flags_ie), 
    .c_alu_carry_en(dec_alu_carry_en), .c_l_reg_sel(dec_l_reg_sel), .c_r_reg_sel(dec_r_reg_sel), .c_rf_ie(dec_rf_ie), 
    .dbg_pc(dbg_pc), .dbg_r0(dbg_r0), .i_submit(decode_execute_submit), .c_jump_cond_code(dec_jump_cond_code), .o_pc_update(execute_fetch_pc_update),
    .o_exec_pc(execute_fetch_pc), .i_jmp_predict(de_jmp_pred), .i_flush(fde_pipeline_flush), .o_flush(fde_pipeline_flush), .c_mem_access(dec_mem_access),
    .c_mem_we(dec_mem_we), .o_data(ew_data), .o_addr(ew_addr), .o_reg_ie(ew_reg_ie), .o_mem_access(ew_mem_access), .o_mem_we(ew_mem_we), .o_submit(ew_submit),
    .i_next_ready(ew_next_ready), .i_reg_ie(we_reg_ie), .i_reg_data(we_reg_data), .c_used_operands(dec_used_operands), .c_sreg_load(dec_sreg_load),
    .c_sreg_store(dec_sreg_store), .c_sreg_jal_over(dec_sreg_jal_over), .i_irq(i_irq), .c_sreg_irt(dec_sreg_irt));

// Pipeline stage 3 - MEM&WB
memwb memwb(.i_clk(i_clk), .i_rst(i_rst), .i_data(ew_data), .i_addr(ew_addr), .i_reg_ie(ew_reg_ie), .i_mem_access(ew_mem_access), .i_mem_we(ew_mem_we),
    .o_reg_ie(we_reg_ie), .o_reg_data(we_reg_data), .i_submit(ew_submit), .o_ready(ew_next_ready), .o_mem_req(o_mem_req), .o_mem_addr(o_mem_addr),
    .o_mem_data(o_mem_data), .o_mem_we(o_mem_we), .i_mem_data(i_mem_data), .i_mem_ack(i_mem_ack));

endmodule

`include "fetch.v"
`include "decode.v"
`include "execute.v"
`include "memwb.v"