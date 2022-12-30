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

    // Independent clock (on independent integer divider)
    input user_clock2,

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
    output inner_ext_irq,
    output inner_embed_mode,
    output inner_disable,

    // Internal ram
    output iram_clk,
    output [7:0] iram_addr,
    output [31:0] iram_i_data,
    input  [31:0] iram_o_data,
    output iram_we, iram_csb,
    output [3:0] iram_w_mask,
    input iram1_clk,
    output [7:0] iram1_addr,
    input  [31:0] iram1_dout,
    input iram1_csb
);

// Use user_clock2 as clock. It can be used independently from mgmt cpu and is not limited to 40Mhz by it.
// Wishbone port is not used, it is the same. Default freq is 10MHz
wire soc_clock = user_clock2;
wire soc_reset = mgt_wb_rst_i;

wire core_clock = soc_clock;
wire soft_reset = (~la_oenb[0]) & la_data_in[0]; // in future & embed mode
wire core_reset_us = soc_reset | soft_reset;
wire core_reset; // synced

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
// pins [4:0] are reserved to mgmt during boot

assign m_io_out[8] = cw_req;
assign m_io_out[9] = cw_dir;
assign m_io_out[25:10] = cw_io_o;
assign cw_io_i = m_io_in[25:10];
assign cw_ack = m_io_in[26];
assign cw_err = m_io_in[27];
assign m_io_out[28] = cw_clk;
assign m_io_out[29] = cw_rst; // reset out

wire ext_irq = m_io_in[30];

wire cs_split_clock = m_io_in[31];
wire core_disable = m_io_in[32];
wire embed_mode = m_io_in[33];

assign spi_clk = m_io_in[34];
assign spi_mosi = m_io_in[35];
assign m_io_out[36] = spi_miso;

assign gpio_in = m_io_in[7:0];
assign m_io_out[7:0] = gpio_out;
assign m_io_out[37] = 1'b1;

assign m_io_oeb[7:0] = gpio_dir;
assign m_io_oeb[9:8] = 2'b0;
assign m_io_oeb[25:10] = {16{cw_dir}};
assign m_io_oeb[27:26] = 2'b11;
assign m_io_oeb[29:28] = 2'b00;
assign m_io_oeb[33:30] = 4'b1111;
assign m_io_oeb[35:34] = 2'b11;
assign m_io_oeb[36] = 1'b0;
assign m_io_oeb[37] = 1'b0;

assign inner_embed_mode = embed_mode_synced;
assign inner_disable = core_disable_synced;

assign irq = 2'b00;

// CLOCKING
`define CLK_DIV_ADDR `WB_ADDR_W'h001001
wire clk_div_write = (m_wb_cyc & m_wb_ack & m_wb_we & wb_tsel_clk);
clk_div clk_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(soc_clock),
    .i_rst(core_reset),
    .o_clk(cw_clk),
    .div(m_wb_o_dat[3:0]),
    .div_we(clk_div_write),
    .clock_sel(cs_split_clock_synced)
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
wire [`WB_DATA_W-1:0] m_wb_i_dat_cross;
wire m_wb_ack_cross, m_wb_err_cross;

wb_cross_clk wb_cross_clk (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .clk_m(core_clock),
    .clk_s(cw_clk),
    .m_rst(core_reset),
    .s_rst(cw_rst),

    .m_wb_cyc(m_wb_cyc),
    .m_wb_stb(m_wb_stb & wb_tsel_cw),
    .m_wb_o_dat(m_wb_o_dat),
    .m_wb_i_dat(m_wb_i_dat_cross),
    .m_wb_adr(m_wb_adr),
    .m_wb_we(m_wb_we),
    .m_wb_ack(m_wb_ack_cross),
    .m_wb_err(m_wb_err_cross),
    .m_wb_sel(m_wb_sel),
    .m_wb_4_burst(m_wb_4_burst),
    .m_wb_8_burst(m_wb_8_burst),

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
wire cw_out_wb_ack;
wire cw_out_wb_err;
wire [`RW-1:0] cw_out_wb_i_dat;

assign cw_mux_wb_cyc = (cs_split_clock_synced ? cwclk_wb_cyc : m_wb_cyc);
assign cw_mux_wb_stb = (cs_split_clock_synced ? cwclk_wb_stb : m_wb_stb);
assign cw_mux_wb_o_dat = (cs_split_clock_synced ? cwclk_wb_o_dat : m_wb_o_dat);
assign cw_mux_wb_adr = (cs_split_clock_synced ? cwclk_wb_adr : m_wb_adr);
assign cw_mux_wb_we = (cs_split_clock_synced ? cwclk_wb_we : m_wb_we);
assign cw_mux_wb_sel = (cs_split_clock_synced ? cwclk_wb_sel : m_wb_sel);
assign cw_mux_wb_8_burst = (cs_split_clock_synced ? cwclk_wb_8_burst : m_wb_8_burst);
assign cw_mux_wb_4_burst = (cs_split_clock_synced ? cwclk_wb_4_burst : m_wb_4_burst);
assign cw_out_wb_ack = (cs_split_clock_synced ? m_wb_ack_cross : cw_mux_wb_ack);
assign cw_out_wb_err = (cs_split_clock_synced ? m_wb_err_cross  : cw_mux_wb_err);
assign cw_out_wb_i_dat = (cs_split_clock_synced ? m_wb_i_dat_cross  : cw_mux_wb_i_dat);


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
    .wb_stb(cw_mux_wb_stb & wb_tsel_cw),
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

// EXT SPI WISHBONE MASTER
wire spi_wb_cyc;
wire spi_wb_stb;
wire [`WB_ADDR_W-1:0] spi_wb_adr;
wire spi_wb_we;
wire [`WB_SEL_BITS-1:0] spi_wb_sel;
wire [`WB_DATA_W-1:0] spi_wb_o_dat;
wire [`WB_DATA_W-1:0] spi_wb_i_dat;
wire spi_wb_ack;
wire spi_wb_err;
wire spi_wb_8_burst=1'b0, spi_wb_4_burst=1'b0;

wire spi_clk;
wire spi_mosi;
wire spi_miso;

sspi sspi (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    .wb_cyc(spi_wb_cyc),
    .wb_stb(spi_wb_stb),
    .wb_o_dat(spi_wb_o_dat),
    .wb_i_dat(spi_wb_i_dat),
    .wb_adr(spi_wb_adr),
    .wb_we(spi_wb_we),
    .wb_ack(spi_wb_ack),
    .wb_err(spi_wb_err),
    .wb_sel(spi_wb_sel)
);

wire gpio_wb_stb;
wire [`RW-1:0] gpio_wb_o_dat;
wire gpio_wb_ack;
wire [7:0] gpio_in, gpio_out, gpio_dir;

gpio #(.N(8)) gpio (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .gpio_in(gpio_in),
    .gpio_out(gpio_out),
    .gpio_dir(gpio_dir),

    .wb_cyc(m_wb_cyc),
    .wb_stb(m_wb_stb),
    .wb_i_dat(m_wb_o_dat),
    .wb_o_dat(gpio_wb_o_dat),
    .wb_adr(m_wb_adr),
    .wb_we(m_wb_we),
    .wb_ack(gpio_wb_ack)
);

// WISHBONE ARBITTER INNER BUS, EXT SPI BUS
wire m_wb_cyc, m_wb_stb;
wire m_wb_we;
wire [`WB_ADDR_W-1:0]  m_wb_adr;
wire [`WB_DATA_W-1:0] m_wb_o_dat;
wire [`WB_SEL_BITS-1:0] m_wb_sel;
wire m_wb_4_burst, m_wb_8_burst;
reg m_wb_ack, m_wb_err;
reg [`WB_DATA_W-1:0] m_wb_i_dat;

assign spi_wb_i_dat = m_wb_i_dat;
assign inner_wb_i_dat = m_wb_i_dat; 

wishbone_arbiter m_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(m_wb_cyc),
    .owb_stb(m_wb_stb),
    .owb_we(m_wb_we),
    .owb_ack(m_wb_ack),
    .owb_adr(m_wb_adr),
    .owb_sel(m_wb_sel),
    .owb_err(m_wb_err),
    .owb_o_dat(m_wb_o_dat),
    .owb_4_burst(m_wb_4_burst),
    .owb_8_burst(m_wb_8_burst),

    .i_wb0_cyc(spi_wb_cyc),
    .wb0_stb(spi_wb_stb),
    .wb0_we(spi_wb_we),
    .wb0_ack(spi_wb_ack),
    .wb0_adr(spi_wb_adr),
    .wb0_sel(spi_wb_sel),
    .wb0_err(spi_wb_err),
    .wb0_o_dat(spi_wb_o_dat),
    .wb0_4_burst(spi_wb_4_burst),
    .wb0_8_burst(spi_wb_8_burst),

    .i_wb1_cyc(inner_wb_cyc),
    .wb1_stb(inner_wb_stb),
    .wb1_we(inner_wb_we),
    .wb1_ack(inner_wb_ack),
    .wb1_adr(inner_wb_adr),
    .wb1_sel(inner_wb_sel),
    .wb1_err(inner_wb_err),
    .wb1_o_dat(inner_wb_o_dat),
    .wb1_4_burst(inner_wb_4_burst),
    .wb1_8_burst(inner_wb_8_burst)
);

// INTERNAL RAM (for easier tesing in embed mode)
wire [`RW-1:0] iram_wb_i_dat;

assign iram_clk = core_clock;
assign iram_addr = m_wb_adr[7:0];
assign iram_i_data = {16'b0,m_wb_o_dat};
assign iram_wb_i_dat = iram_o_data[15:0];
assign iram_we = m_wb_cyc & m_wb_stb & m_wb_we & wb_tsel_iram;
assign iram_csb = 1'b1;
assign iram1_clk = 1'b0;
assign iram1_csb = 1'b0;
assign iram1_addr = 1'b0;

reg iram_wb_ack;
always @(posedge core_clock) begin
    if (core_reset) iram_wb_ack <= 1'b0;
    else iram_wb_ack <= m_wb_cyc & m_wb_stb & wb_tsel_iram & ~m_wb_ack; // 1 cyc delay
end

// WISHBONE TARGET SELECT
`define NE_INTMEM_BEGIN 24'h7ffe00
`define E_PROG_START    24'h800000
`define E_MEM_START     24'h100000
`define INTMEM_SIZE     24'h100
`define GPIO_START      24'h001010
`define GPIO_END        24'h001012

wire wb_tsel_cw = (~embed_mode && (m_wb_adr != `CLK_DIV_ADDR) && (~((m_wb_adr >= `GPIO_START) && (m_wb_adr <= `GPIO_END))) && ((m_wb_adr < `NE_INTMEM_BEGIN) || (m_wb_adr >= `NE_INTMEM_BEGIN+`INTMEM_SIZE)));
wire wb_tsel_iram = (~embed_mode && (m_wb_adr >= `NE_INTMEM_BEGIN) && (m_wb_adr < `NE_INTMEM_BEGIN+`INTMEM_SIZE))
                    || (embed_mode && (((m_wb_adr >= `E_PROG_START) && (m_wb_adr < `E_PROG_START+`INTMEM_SIZE)) 
                                    || (m_wb_adr >= `E_MEM_START) && (m_wb_adr < `E_MEM_START+`INTMEM_SIZE)));
wire wb_tsel_clk = (m_wb_adr == `CLK_DIV_ADDR);
wire wb_tsel_gpio = (m_wb_adr >= `GPIO_START) && (m_wb_adr <= `GPIO_END);

always @(*) begin
    if (wb_tsel_cw) begin
        m_wb_ack = cw_out_wb_ack;
        m_wb_err = cw_out_wb_err;
        m_wb_i_dat = cw_out_wb_i_dat;
    end else if (wb_tsel_iram) begin
        m_wb_ack = iram_wb_ack;
        m_wb_err = 1'b0;
        m_wb_i_dat = iram_wb_i_dat;
    end else if (wb_tsel_clk) begin
        m_wb_ack = 1'b1;
        m_wb_err = 1'b0;
        m_wb_i_dat = `RW'b0;
    end else if (wb_tsel_gpio) begin
        m_wb_ack = gpio_wb_ack;
        m_wb_err = 1'b0;
        m_wb_i_dat = gpio_wb_o_dat;
    end else begin
        m_wb_ack = 1'b0;
        m_wb_err = 1'b1;
        m_wb_i_dat = `RW'b0;
    end
end

// SYNCHRONIZERS

reset_sync rst_cw_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(core_reset_us),
    .o_rst(cw_rst)
);

reset_sync rst_soc_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(soc_clock),
    .i_rst(core_reset_us),
    .o_rst(core_reset)
);

// sync external gpio signals

assign inner_ext_irq = irq_s_ff[1];
reg [1:0] irq_s_ff;
always @(posedge core_clock) begin
    irq_s_ff[0] <= ext_irq;
    irq_s_ff[1] <= irq_s_ff[0]; 
end

wire cs_split_clock_synced = split_s_ff[1];
wire core_disable_synced = disable_s_ff[1];
wire embed_mode_synced = embed_s_ff[1];

reg [1:0] split_s_ff;
always @(posedge core_clock) begin
    split_s_ff[0] <= cs_split_clock;
    split_s_ff[1] <= split_s_ff[0]; 
end

reg [1:0] disable_s_ff;
always @(posedge core_clock) begin
    disable_s_ff[0] <= core_disable;
    disable_s_ff[1] <= disable_s_ff[0]; 
end

reg [1:0] embed_s_ff;
always @(posedge core_clock) begin
    embed_s_ff[0] <= embed_mode;
    embed_s_ff[1] <= embed_s_ff[0]; 
end

endmodule