`include "config.v"

module execute (
    input i_clk,
    input i_rst,

    // Pipeline control singnals
    output reg o_ready,

    input [`RW-1:0] i_imm,

    // Execution control singals
    input c_pc_inc, c_pc_ie,
    input c_r_bus_imm,
    input [`ALU_MODE_W-1:0] c_alu_mode,
    input c_alu_carry_en, c_alu_flags_ie,
    input [`REGNO_LOG-1:0] c_l_reg_sel, c_r_reg_sel, 
    input [`REGNO-1:0] c_rf_ie,

    // Debug outputs
    output [`RW-1:0] dbg_r0, dbg_pc
);

// separate memory access stage?
assign o_ready = 1'b1; // for now this stage is always read (no mem)

// Internal buses
wire [`RW-1:0] reg_l_con, reg_r_con;
wire [`RW-1:0] alu_l_bus, alu_r_bus;
wire [`RW-1:0] alu_bus;

// Muxes definitions
assign alu_l_bus = reg_l_con;
assign alu_r_bus = (c_r_bus_imm ? i_imm : reg_r_con);

// Component connects
wire [`RW-1:0] pc_val;
wire [`ALU_FLAG_CNT-1:0] alu_flags_d, alu_flags_q;
assign dbg_pc = pc_val;

// Submodules
rf rf(.i_clk(i_clk), .i_rst(i_rst), .i_d(alu_bus), .o_lout(reg_l_con),
    .o_rout(reg_r_con), .i_lout_sel(c_l_reg_sel), .i_rout_sel(c_r_reg_sel),
    .i_ie(c_rf_ie), .dbg_r0(dbg_r0));

alu alu(.i_l(alu_l_bus), .i_r(alu_r_bus), .o_out(alu_bus), .i_mode(c_alu_mode), 
    .o_flags(alu_flags_d), .i_carry(alu_flags_q[`ALU_FLAG_C] & c_alu_carry_en));

pc pc(.i_clk(i_clk), .i_rst(i_rst), .i_bus(alu_bus), .i_c_pc_inc(c_pc_inc), 
    .i_c_pc_ie(c_pc_ie), .o_pc(pc_val));

// Cpu control registers
register  #(.N(`ALU_FLAG_CNT)) alu_flag_reg (.i_clk(i_clk), .i_rst(i_rst), 
    .i_d(alu_flags_d), .o_d(alu_flags_q), .i_ie(c_alu_flags_ie));

endmodule;

`include "alu.v"
`include "rf.v"