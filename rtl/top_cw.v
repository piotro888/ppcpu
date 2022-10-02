/* Upper core with compressed wishbone bus output (at different clock domain) */

`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

`define USELA

module top_cw (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input wire i_clk,
    input wire i_rst,

    input [`RW-1:0] cw_io_i,
    output [`RW-1:0] cw_io_o,
    output cw_req,
    output cw_dir,
    input cw_ack,
    input cw_err,
    output cw_clk,
    input i_irq,
    output cw_rst,

    output [61:0] dbg_out,
    input [5:0] dbg_in,

    input [`RW-1:0] la_cw_io_i,
    input la_cw_ack,
    input la_cw_ovr
);

wire ic_split_clock = dbg_in[4];
wire u_wb_8_burst, u_wb_4_burst;
wire u_wb_cyc;
wire u_wb_stb;
wire [`WB_DATA_W-1:0] u_wb_o_dat;
wire[`WB_DATA_W-1:0] u_wb_i_dat;
wire [`WB_ADDR_W-1:0]  u_wb_adr;
wire u_wb_we;
wire u_wb_ack, u_wb_ack_cmp, u_wb_ack_clk;
wire u_wb_err;
wire [`WB_SEL_BITS-1:0] u_wb_sel;

wire [`RW-1:0] dbg_r0, ignore_dbg_pc;

upper_core upc (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
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
    .dbg_r0(dbg_r0),
    .dbg_pc(ignore_dbg_pc),
    .wb_4_burst(u_wb_4_burst),
    .wb_8_burst(u_wb_8_burst),

    .dbg_in({dbg_in[5], dbg_in[3:0]}),
    .dbg_out(dbg_out[61:0])
);

wire cmp_clk;

//`define CLK_DIV_ADDR `WB_ADDR_W'h001001
clk_div clk_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    .i_rst(s_rst),
    .o_clk(cmp_clk),
    .div(u_wb_o_dat[3:0]),
    .div_we(u_wb_ack_clk),
    .clock_sel(ic_split_clock)
);

wire cc_wb_8_burst, cc_wb_4_burst;
wire cc_wb_cyc;
wire cc_wb_stb;
wire [`WB_DATA_W-1:0] cc_wb_o_dat;
wire[`WB_DATA_W-1:0] cc_wb_i_dat;
wire [`WB_ADDR_W-1:0] cc_wb_adr;
wire cc_wb_we;
wire cc_wb_ack;
wire cc_wb_err;
wire [`WB_SEL_BITS-1:0] cc_wb_sel;

wb_cross_clk wb_cross_clk (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .clk_m(i_clk),
    .clk_s(cmp_clk),
    .m_rst(s_rst),
    .s_rst(cw_rst),

    .m_wb_cyc(u_wb_cyc),
    .m_wb_stb(u_wb_stb),
    .m_wb_o_dat(u_wb_o_dat),
    .m_wb_i_dat(u_wb_i_dat_cc),
    .m_wb_adr(u_wb_adr),
    .m_wb_we(u_wb_we),
    .m_wb_ack(u_wb_ack_cc),
    .m_wb_err(u_wb_err_cc),
    .m_wb_sel(u_wb_sel),
    .m_wb_4_burst(u_wb_4_burst),
    .m_wb_8_burst(u_wb_8_burst),

    .s_wb_cyc(cc_wb_cyc),
    .s_wb_stb(cc_wb_stb),
    .s_wb_o_dat(cc_wb_o_dat),
    .s_wb_i_dat(c_wb_i_dat_cmp),
    .s_wb_adr(cc_wb_adr),
    .s_wb_we(cc_wb_we),
    .s_wb_ack(c_wb_ack_cmp),
    .s_wb_err(c_wb_err_cmp),
    .s_wb_sel(cc_wb_sel),
    .s_wb_4_burst(cc_wb_4_burst),
    .s_wb_8_burst(cc_wb_8_burst)
);

// Wishbone mux to skip multiple clocks
wire c_wb_8_burst;
wire c_wb_4_burst;
wire c_wb_cyc;
wire c_wb_stb;
wire [`WB_DATA_W-1:0] c_wb_o_dat;
wire [`WB_ADDR_W-1:0]  c_wb_adr;
wire c_wb_we;
wire [`WB_SEL_BITS-1:0] c_wb_sel;

wire [`WB_DATA_W-1:0] c_wb_i_dat_cmp, u_wb_i_dat_cc;
wire c_wb_ack_cmp, c_wb_err_cmp, u_wb_ack_cc, u_wb_err_cc;
wire [`RW-1:0] m_cw_io_i;
wire m_cw_ack, m_cw_err;
wb_compressor wb_compressor(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(cw_rst),

    .cw_io_i(m_cw_io_i),
    .cw_io_o(cw_io_o),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(m_cw_ack),
    .cw_err(m_cw_err),

    .wb_cyc(c_wb_cyc),
    .wb_stb(c_wb_stb),
    .wb_o_dat(c_wb_o_dat),
    .wb_i_dat(c_wb_i_dat_cmp),
    .wb_adr(c_wb_adr),
    .wb_we(c_wb_we),
    .wb_ack(c_wb_ack_cmp),
    .wb_err(c_wb_err_cmp),
    .wb_sel(c_wb_sel),

    .wb_4_burst(c_wb_4_burst),
    .wb_8_burst(c_wb_8_burst)
);

assign cw_clk = cmp_clk; // clock is muxed inside clock_div moudle
wire s_rst, cw_rst_z;

wire irq_s;
wire u_wb_ack_mxed;
top_cw_logic top_cw_logic (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    
    .i_irq(i_irq),
    .irq_s(irq_s),

    .cw_clk(cw_clk),
    .i_rst(i_rst),
    .s_rst(s_rst),
    .cw_rst_z(cw_rst_z),

    .ic_split_clock(ic_split_clock),
    .cw_rst(cw_rst),

    .c_wb_8_burst(c_wb_8_burst),
    .c_wb_4_burst(c_wb_4_burst),
    .c_wb_cyc(c_wb_cyc),
    .c_wb_stb(c_wb_stb),
    .c_wb_o_dat(c_wb_o_dat),
    .c_wb_adr(c_wb_adr),
    .c_wb_we(c_wb_we),
    .c_wb_sel(c_wb_sel),

    .cc_wb_8_burst(cc_wb_8_burst),
    .u_wb_8_burst(u_wb_8_burst),
    .cc_wb_4_burst(cc_wb_4_burst),
    .u_wb_4_burst(u_wb_4_burst),
    .cc_wb_cyc(cc_wb_cyc),
    .u_wb_cyc(u_wb_cyc),
    .cc_wb_stb(cc_wb_stb),
    .u_wb_stb(u_wb_stb),
    .cc_wb_o_dat(cc_wb_o_dat),
    .u_wb_o_dat(u_wb_o_dat),
    .cc_wb_adr(cc_wb_adr),
    .u_wb_adr(u_wb_adr),
    .cc_wb_we(cc_wb_we),
    .u_wb_we(u_wb_we),
    .cc_wb_sel(cc_wb_sel),
    .u_wb_sel(u_wb_sel),

    .u_wb_ack_mxed(u_wb_ack_mxed),
    .u_wb_err(u_wb_err),
    .u_wb_i_dat(u_wb_i_dat),
    .u_wb_ack_cc(u_wb_ack_cc),
    .c_wb_ack_cmp(c_wb_ack_cmp),
    .u_wb_err_cc(u_wb_err_cc),
    .c_wb_err_cmp(c_wb_err_cmp),
    .u_wb_i_dat_cc(u_wb_i_dat_cc),
    .c_wb_i_dat_cmp(c_wb_i_dat_cmp),
    .u_wb_ack(u_wb_ack),
    .u_wb_ack_clk(u_wb_ack_clk),

    .m_cw_io_i(m_cw_io_i),
    .m_cw_ack(m_cw_ack),
    .m_cw_err(m_cw_err),
    .la_cw_io_i(la_cw_io_i),
    .la_cw_ack(la_cw_ack),
    .la_cw_ovr(la_cw_ovr),
    .cw_io_i(cw_io_i),
    .cw_err(cw_err),
    .cw_ack(cw_ack)
);

endmodule

`include "wishbone/wb_cross_clk.v"
`include "wishbone/wb_compressor.v"
