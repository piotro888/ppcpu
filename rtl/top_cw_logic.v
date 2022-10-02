`define WB_DATA_W 16 
`define WB_SEL_BITS 2
`define WB_ADDR_W 24

module top_cw_logic (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    
    input i_irq,
    output irq_s,

    input cw_clk,
    input i_rst,
    output s_rst,
    output cw_rst_z,

    input ic_split_clock,
    output cw_rst,
    output c_wb_8_burst,
    output c_wb_4_burst,
    output c_wb_cyc,
    output c_wb_stb,
    output [`WB_DATA_W-1:0] c_wb_o_dat,
    output [`WB_ADDR_W-1:0] c_wb_adr,
    output c_wb_we,
    output [1:0] c_wb_sel,

    input cc_wb_8_burst, u_wb_8_burst,
    input cc_wb_4_burst, u_wb_4_burst,
    input cc_wb_cyc, u_wb_cyc,
    input cc_wb_stb, u_wb_stb,
    input [`WB_DATA_W-1:0] cc_wb_o_dat, u_wb_o_dat,
    input [`WB_ADDR_W-1:0] cc_wb_adr, u_wb_adr,
    input cc_wb_we, u_wb_we,
    input [1:0] cc_wb_sel, u_wb_sel,

    output u_wb_ack_mxed, u_wb_err,
    output [`WB_DATA_W-1:0] u_wb_i_dat,

    input u_wb_ack_cc, c_wb_ack_cmp, u_wb_err_cc, c_wb_err_cmp,
    input [`WB_DATA_W-1:0] u_wb_i_dat_cc, c_wb_i_dat_cmp,

    output u_wb_ack,
    output u_wb_ack_clk,

    output [15:0] m_cw_io_i,
    output m_cw_ack,
    output m_cw_err,
    input [15:0] la_cw_io_i,
    input la_cw_ack,
    input la_cw_ovr,
    input [15:0] cw_io_i,
    input cw_err,
    input cw_ack
);

assign irq_s = irq_s_ff[1];
reg [1:0] irq_s_ff;
always @(posedge i_clk) begin
    irq_s_ff[0] <= i_irq;
    irq_s_ff[1] <= irq_s_ff[0]; 
end

reset_sync rst_clk_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    .i_rst(i_rst),
    .o_rst(s_rst)
);

reset_sync rst_cw_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(i_rst),
    .o_rst(cw_rst_z)
);

assign cw_rst = (ic_split_clock ? cw_rst_z : s_rst);

// Wishbone mux to skip multiple clocks
assign c_wb_8_burst = (ic_split_clock ? cc_wb_8_burst : u_wb_8_burst);
assign c_wb_4_burst = (ic_split_clock ? cc_wb_4_burst : u_wb_4_burst);
assign c_wb_cyc = (ic_split_clock ? cc_wb_cyc : u_wb_cyc);
assign c_wb_stb = (ic_split_clock ? cc_wb_stb : u_wb_stb);
assign c_wb_o_dat = (ic_split_clock ? cc_wb_o_dat : u_wb_o_dat);
assign c_wb_adr = (ic_split_clock ? cc_wb_adr : u_wb_adr);
assign c_wb_we = (ic_split_clock ? cc_wb_we : u_wb_we);
assign c_wb_sel = (ic_split_clock ? cc_wb_sel : u_wb_sel);

assign u_wb_ack_mxed = (ic_split_clock ? u_wb_ack_cc : c_wb_ack_cmp);
assign u_wb_err = (ic_split_clock ? u_wb_err_cc : c_wb_err_cmp);
assign u_wb_i_dat = (ic_split_clock ? u_wb_i_dat_cc : c_wb_i_dat_cmp);

`define CLK_DIV_ADDR 24'h001001
assign u_wb_ack_clk = u_wb_cyc & u_wb_stb & u_wb_we & (u_wb_adr == `CLK_DIV_ADDR);

assign u_wb_ack = u_wb_ack_mxed | u_wb_ack_clk;

assign m_cw_io_i = (la_cw_ovr ? la_cw_io_i : cw_io_i);
assign m_cw_ack = (la_cw_ovr ? la_cw_ovr : cw_ack);
assign m_cw_err = cw_err & ~la_cw_ovr;

endmodule