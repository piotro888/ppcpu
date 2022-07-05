`include "config.v"

module cpu (
    input i_clk,
    input i_rst
);

wire c_l_bus_imm;

wire [`RW-1:0] reg_l_con, reg_r_con, imm_con;
wire [`RW-1:0] alu_l_bus, alu_r_bus;
wire [`RW-1:0] alu_bus;

assign alu_l_bus = (c_l_bus_imm ? imm_con : reg_l_con);
assign alu_r_bus = reg_r_con;

rf rf(.i_d(alu_bus), .o_lout(reg_l_con), .o_rout(reg_r_con));
alu alu(.i_l(alu_l_bus), .i_r(alu_r_bus), .o_out(alu_bus));

endmodule;

`include "alu.v"
`include "rf.v"