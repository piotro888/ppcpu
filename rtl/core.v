`include "config.v"

module core (
    input i_clk,
    input i_rst,

    // fetch input singals
    output [`RW-1:0] o_req_addr,
    output o_req_active,
    input [`I_SIZE-1:0] i_req_data,
    input i_req_data_valid,

    output [`RW-1:0] dbg_r0, dbg_pc
);

// --- CPU INTERNAL CONNECTIONS ---

wire fetch_decode_next_ready;
wire fetch_decode_submit;
wire [`I_SIZE-1:0] fetch_decode_d_instr;

// Pipeline stage 0 - FETCH
fetch fetch (.i_clk(i_clk), .i_rst(i_rst), .o_req_active(o_req_active), .o_req_addr(o_req_addr),
    .i_req_data(i_req_data), .i_req_data_valid(i_req_data_valid), .i_next_ready(fetch_decode_next_ready),
    .o_submit(fetch_decode_submit), .o_instr(fetch_decode_d_instr));

wire decode_execute_next_ready;
wire decode_execute_submit;
wire [`I_SIZE-17:0] de_imm_pass;

wire dec_pc_inc, dec_pc_ie;
wire dec_r_bus_imm;
wire [`ALU_MODE_W-1:0] dec_alu_mode;
wire dec_alu_carry_en, dec_alu_flags_ie;
wire [`REGNO_LOG-1:0] dec_l_reg_sel, dec_r_reg_sel; 
wire [`REGNO-1:0] dec_rf_ie;

// Pipeline stage 1 - DECODE
decode decode(.i_clk(i_clk), .i_rst(i_rst), .o_ready(fetch_decode_next_ready), .o_submit(decode_execute_submit),
    .i_next_ready(decode_execute_next_ready), .i_instr_l(fetch_decode_d_instr[15:0]), .i_imm_pass(fetch_decode_d_instr[`I_SIZE-1:16]),
    .o_imm_pass(de_imm_pass), .oc_pc_inc(dec_pc_inc), .oc_pc_ie(dec_pc_ie), .oc_r_bus_imm(dec_r_bus_imm), .oc_alu_mode(dec_alu_mode),
    .oc_alu_flags_ie(dec_alu_flags_ie), .oc_alu_carry_en(dec_alu_carry_en), .oc_l_reg_sel(dec_l_reg_sel), .oc_r_reg_sel(dec_r_reg_sel),
    .oc_rf_ie(dec_rf_ie));

// Pipeline stage 2 - EXECUTE
execute execute(.i_clk(i_clk), .i_rst(i_rst), .o_ready(decode_execute_next_ready), .i_imm(de_imm_pass), .c_pc_inc(dec_pc_inc),
    .c_pc_ie(dec_pc_ie), .c_r_bus_imm(dec_r_bus_imm), .c_alu_mode(dec_alu_mode), .c_alu_flags_ie(dec_alu_flags_ie), 
    .c_alu_carry_en(dec_alu_carry_en), .c_l_reg_sel(dec_l_reg_sel), .c_r_reg_sel(dec_r_reg_sel), .c_rf_ie(dec_rf_ie), 
    .dbg_pc(dbg_pc), .dbg_r0(dbg_r0));

endmodule

`include "fetch.v"
`include "decode.v"
`include "execute.v"
