`include "config.v"

`define MPRJ_IO_PADS 38

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module interconnect_outer (
    // IOs
    input  [`MPRJ_IO_PADS-1:0] m_io_in,
    output [`MPRJ_IO_PADS-1:0] m_io_out,
    output [`MPRJ_IO_PADS-1:0] m_io_oeb,

    // Wishbone from managment core
    input mgt_wb_clk_i,
    input mgt_wb_rst_i,
    input mgt_wb_stb_i,
    input mgt_wb_cyc_i,
    input mgt_wb_we_i,
    input [3:0] mgt_wb_sel_i,
    input [31:0] mgt_wb_dat_i,
    input [31:0] mgt_wb_adr_i,
    output mgt_wb_ack_o,
    output [31:0] mgt_wb_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IRQ
    output [2:0] irq,


    // Lower Interconnect
    output inner_clock, inner_reset,
    input inner_wb_cyc, inner_wb_stb,
    input inner_wb_we,
    input [`WB_ADDR_W-1:0] inner_wb_adr,
    input [`WB_DATA_W-1:0] inner_wb_o_dat,
    output [`WB_DATA_W-1:0] inner_wb_i_dat,
    output inner_wb_ack, inner_wb_err,
    input [`WB_SEL_BITS-1:0] inner_wb_sel,
    input inner_wb_4_burst, inner_wb_8_burst,
    output inner_ext_irq
);

wire soc_clock = mgt_wb_clk_i;
wire soc_reset = mgt_wb_rst_i;

wire core_clock = soc_clock;
wire soft_reset = (~la_oenb[0]) & la_data_in[0]; // in future & embed mode
wire core_reset = soc_reset | soft_reset;

assign inner_clock = core_clock;
assign inner_reset = core_reset;

// MASTER CW BUS

wire [`RW-1:0] cw_io_i; // S>M
wire [`RW-1:0] cw_io_o; // M>S
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;
wire cw_clk; // ouputs for bridge
wire cw_rst;

// PIN ASSIGNMENTS

assign m_io_out[0] = cw_req;
assign m_io_out[1] = cw_dir;
assign m_io_out[17:2] = cw_io_o;
assign cw_io_i = m_io_in[17:2];
assign cw_ack = m_io_in[18];
assign cw_err = m_io_in[19];
assign m_io_out[20] = cw_clk;
assign m_io_out[21] = cw_rst; // reset out

wire ext_irq = m_io_in[22];

wire cs_split_clock = m_io_in[23];

assign m_io_oeb[1:0] = 2'b0;
assign m_io_oeb[17:2] = {16{cw_dir}}; // try to invert
assign m_io_oeb[19:18] = 2'b11;
assign m_io_oeb[21:20] = 2'b00;
assign m_io_oeb[23:22] = 2'b11;
assign m_io_oeb[`MPRJ_IO_PADS-1:24] = {`MPRJ_IO_PADS-24{1'b1}}; 

// CLOCKING
`define CLK_DIV_ADDR `WB_ADDR_W'h001001
wire clk_div_write = (inner_wb_cyc & inner_wb_ack & inner_wb_we & (inner_wb_adr == `CLK_DIV_ADDR));
// TODO: ackkkkkkkkkkk
clk_div clk_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(soc_clock),
    .i_rst(core_reset),
    .o_clk(cw_clk),
    .div(inner_wb_o_dat[3:0]),
    .div_we(clk_div_write),
    .clock_sel(cs_split_clock)
);

// WISHBONE CLOCK DOMAIN CROSSING

wire cwclk_wb_cyc;
wire cwclk_wb_stb;
wire [`WB_ADDR_W-1:0] cwclk_wb_adr;
wire cwclk_wb_we;
wire [`WB_SEL_BITS-1:0] cwclk_wb_sel;
wire [`WB_DATA_W-1:0] cwclk_wb_o_dat;
wire[`WB_DATA_W-1:0] cwclk_wb_i_dat = cw_mux_wb_i_dat;
wire cwclk_wb_ack = cw_mux_wb_ack;
wire cwclk_wb_err = cw_mux_wb_err;
wire cwclk_wb_8_burst, cwclk_wb_4_burst;

// CROSS_CLK OUPUT SIGNALS FROM M_WB SIDE FOR MUXING
wire [`WB_DATA_W-1:0] inner_wb_i_dat_cross;
wire inner_wb_ack_cross, inner_wb_err_cross;

wb_cross_clk wb_cross_clk (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .clk_m(core_clock),
    .clk_s(cw_clk),
    .m_rst(core_reset),
    .s_rst(cw_rst),

    .m_wb_cyc(inner_wb_cyc),
    .m_wb_stb(inner_wb_stb),
    .m_wb_o_dat(inner_wb_o_dat),
    .m_wb_i_dat(inner_wb_i_dat_cross),
    .m_wb_adr(inner_wb_adr),
    .m_wb_we(inner_wb_we),
    .m_wb_ack(inner_wb_ack_cross),
    .m_wb_err(inner_wb_err_cross),
    .m_wb_sel(inner_wb_sel),
    .m_wb_4_burst(inner_wb_4_burst),
    .m_wb_8_burst(inner_wb_8_burst),

    .s_wb_cyc(cwclk_wb_cyc),
    .s_wb_stb(cwclk_wb_stb),
    .s_wb_o_dat(cwclk_wb_o_dat),
    .s_wb_i_dat(cwclk_wb_i_dat),
    .s_wb_adr(cwclk_wb_adr),
    .s_wb_we(cwclk_wb_we),
    .s_wb_ack(cwclk_wb_ack),
    .s_wb_err(cwclk_wb_err),
    .s_wb_sel(cwclk_wb_sel),
    .s_wb_4_burst(cwclk_wb_4_burst),
    .s_wb_8_burst(cwclk_wb_8_burst)
);

// clock split bypass

wire cw_mux_wb_cyc;
wire cw_mux_wb_stb;
wire [`WB_ADDR_W-1:0] cw_mux_wb_adr;
wire cw_mux_wb_we;
wire [`WB_SEL_BITS-1:0] cw_mux_wb_sel;
wire [`WB_DATA_W-1:0] cw_mux_wb_o_dat;
wire[`WB_DATA_W-1:0] cw_mux_wb_i_dat;
wire cw_mux_wb_ack;
wire cw_mux_wb_err;
wire cw_mux_wb_8_burst, cw_mux_wb_4_burst;

assign cw_mux_wb_cyc = (cs_split_clock ? cwclk_wb_cyc : inner_wb_cyc);
assign cw_mux_wb_stb = (cs_split_clock ? cwclk_wb_stb : inner_wb_stb);
assign cw_mux_wb_o_dat = (cs_split_clock ? cwclk_wb_o_dat : inner_wb_o_dat);
assign cw_mux_wb_adr = (cs_split_clock ? cwclk_wb_adr : inner_wb_adr);
assign cw_mux_wb_we = (cs_split_clock ? cwclk_wb_we : inner_wb_we);
assign cw_mux_wb_sel = (cs_split_clock ? cwclk_wb_sel : inner_wb_sel);
assign cw_mux_wb_8_burst = (cs_split_clock ? cwclk_wb_8_burst : inner_wb_8_burst);
assign cw_mux_wb_4_burst = (cs_split_clock ? cwclk_wb_4_burst : inner_wb_4_burst);
assign inner_wb_ack = (cs_split_clock ? inner_wb_ack_cross : cw_mux_wb_ack);
assign inner_wb_err = (cs_split_clock ? inner_wb_err_cross  : cw_mux_wb_err);
assign inner_wb_i_dat = (cs_split_clock ? inner_wb_i_dat_cross  : cw_mux_wb_i_dat);


// CW BUS CONVERTER

wb_compressor wb_compressor (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(cw_rst),

    .cw_io_i(cw_io_i),
    .cw_io_o(cw_io_o),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(cw_mux_wb_cyc),
    .wb_stb(cw_mux_wb_stb),
    .wb_o_dat(cw_mux_wb_o_dat),
    .wb_i_dat(cw_mux_wb_i_dat),
    .wb_adr(cw_mux_wb_adr),
    .wb_we(cw_mux_wb_we),
    .wb_ack(cw_mux_wb_ack),
    .wb_err(cw_mux_wb_err),
    .wb_sel(cw_mux_wb_sel),
    .wb_4_burst(cw_mux_wb_4_burst),
    .wb_8_burst(cw_mux_wb_8_burst)
);

// SYNCHRONIZERS

reset_sync rst_cw_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(soc_reset),
    .o_rst(cw_rst)
);

assign inner_ext_irq = irq_s_ff[1];
reg [1:0] irq_s_ff;
always @(posedge core_clock) begin
    irq_s_ff[0] <= ext_irq;
    irq_s_ff[1] <= irq_s_ff[0]; 
end


endmodule