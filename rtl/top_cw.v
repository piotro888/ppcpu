`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module top_cw (
    input wire i_clk,
    input wire i_rst,

    inout reg [`RW-1:0] cw_io,
    output cw_req,
    output cw_dir,
    input cw_ack,
    input cw_err,
    output cw_clk,
    input i_irq
);

wire u_wb_8_burst, u_wb_4_burst;
wire u_wb_cyc;
wire u_wb_stb;
wire [`WB_DATA_W-1:0] u_wb_o_dat;
wire[`WB_DATA_W-1:0] u_wb_i_dat;
wire [`WB_ADDR_W-1:0]  u_wb_adr;
wire u_wb_we;
wire u_wb_ack;
wire u_wb_err;
wire [`WB_SEL_BITS-1:0] u_wb_sel;

wire [`RW-1:0] ignore_dbg_r0, ignore_dbg_pc;

upper_core upc (
    .i_clk(i_clk),
    .i_rst(s_rst),
    .wb_cyc(u_wb_cyc),
    .wb_stb(u_wb_stb),
    .wb_o_dat(u_wb_o_dat),
    .wb_i_dat(u_wb_i_dat),
    .wb_adr(u_wb_adr),
    .wb_we(u_wb_we),
    .wb_ack(u_wb_ack),
    .wb_err(u_wb_err),
    .wb_sel(u_wb_sel),
    .i_irq(irq_s),
    .wb_rty(1'b0),
    .dbg_r0(ignore_dbg_r0),
    .dbg_pc(ignore_dbg_pc),
    .wb_4_burst(u_wb_4_burst),
    .wb_8_burst(u_wb_8_burst)
);

wire cmp_clk;

clock_div clock_div (
    .i_clk(i_clk),
    .i_rst(s_rst),
    .o_clk(cmp_clk),
    .div('0),
    .div_we(1'b0)
);

wire c_wb_8_burst, c_wb_4_burst;
wire c_wb_cyc;
wire c_wb_stb;
wire [`WB_DATA_W-1:0] c_wb_o_dat;
wire[`WB_DATA_W-1:0] c_wb_i_dat;
wire [`WB_ADDR_W-1:0]  c_wb_adr;
wire c_wb_we;
wire c_wb_ack;
wire c_wb_err;
wire [`WB_SEL_BITS-1:0] c_wb_sel;

wb_cross_clk wb_cross_clk (
    .clk_m(i_clk),
    .clk_s(cmp_clk),
    .m_rst(s_rst),
    .s_rst(cw_rst),

    .m_wb_cyc(u_wb_cyc),
    .m_wb_stb(u_wb_stb),
    .m_wb_o_dat(u_wb_o_dat),
    .m_wb_i_dat(u_wb_i_dat),
    .m_wb_adr(u_wb_adr),
    .m_wb_we(u_wb_we),
    .m_wb_ack(u_wb_ack),
    .m_wb_err(u_wb_err),
    .m_wb_sel(u_wb_sel),
    .m_wb_4_burst(u_wb_4_burst),
    .m_wb_8_burst(u_wb_8_burst),

    .s_wb_cyc(c_wb_cyc),
    .s_wb_stb(c_wb_stb),
    .s_wb_o_dat(c_wb_o_dat),
    .s_wb_i_dat(c_wb_i_dat),
    .s_wb_adr(c_wb_adr),
    .s_wb_we(c_wb_we),
    .s_wb_ack(c_wb_ack),
    .s_wb_err(c_wb_err),
    .s_wb_sel(c_wb_sel),
    .s_wb_4_burst(c_wb_4_burst),
    .s_wb_8_burst(c_wb_8_burst)
);

wb_compressor wb_compressor(
    .i_clk(cmp_clk),
    .i_rst(cw_rst),

    .cw_io(cw_io),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(c_wb_cyc),
    .wb_stb(c_wb_stb),
    .wb_o_dat(c_wb_o_dat),
    .wb_i_dat(c_wb_i_dat),
    .wb_adr(c_wb_adr),
    .wb_we(c_wb_we),
    .wb_ack(c_wb_ack),
    .wb_err(c_wb_err),
    .wb_sel(c_wb_sel),

    .wb_4_burst(c_wb_4_burst),
    .wb_8_burst(c_wb_8_burst)
);

assign cw_clk = cmp_clk;
wire cw_rst, s_rst;

reset_sync rst_clk_sync (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .o_rst(s_rst)
);

reset_sync rst_cw_sync (
    .i_clk(cw_clk),
    .i_rst(i_rst),
    .o_rst(cw_rst)
);

wire irq_s = irq_s_ff[1];
reg [1:0] irq_s_ff;
always @(posedge i_clk) begin
    irq_s_ff[0] <= i_irq;
    irq_s_ff[1] <= irq_s_ff[0]; 
end

endmodule

`include "upper_core.v"
`undef SW
`include "wishbone/wb_compressor.v"
`undef SW
`include "wishbone/wb_decomp.v"
`include "wishbone/wb_mclock.v"
`include "clock_div.v"
