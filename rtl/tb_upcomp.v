`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module tb_upcomp (
    input wire i_clk,
    input wire i_rst,

    output wb_clk,
    output wb_cyc,
    output reg wb_stb,
    output reg [`WB_DATA_W-1:0] wb_o_dat,
    input [`WB_DATA_W-1:0] wb_i_dat,
    output reg [`WB_ADDR_W-1:0]  wb_adr,
    output reg wb_we,
    input wb_ack,
    input wb_err,
    output reg [`WB_SEL_BITS-1:0] wb_sel,

    input i_irq,
    output [61:0] dbg_out
);

wire cw_clk;
wire [`RW-1:0] cw_io_i, cw_io_o;
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;
wire cw_rst;

top_cw top_cw (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .cw_clk(cw_clk),

    .cw_io_i(cw_io_i),
    .cw_io_o(cw_io_o),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),
    .i_irq(i_irq),
    .cw_rst(cw_rst),

    .dbg_in(5'b10000),
    .dbg_out(dbg_out),

    .la_cw_ovr(1'b0),
    .la_cw_ack('b0),
    .la_cw_io_i('b0)
);

wb_decomp wb_decomp (
    .i_clk(cw_clk),
    .i_rst(i_rst),

    .cw_io_i(cw_io_o),
    .cw_io_o(cw_io_i),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb),
    .wb_o_dat(wb_o_dat),
    .wb_i_dat(wb_i_dat),
    .wb_adr(wb_adr),
    .wb_we(wb_we),
    .wb_ack(wb_ack),
    .wb_err(wb_err),
    .wb_sel(wb_sel)
);
assign wb_clk = cw_clk;

endmodule

`include "top_cw.v"
`include "wishbone/wb_decomp.v"
