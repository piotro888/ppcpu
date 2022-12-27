// Connects all moules to interconnects
// No logic or buffers allowed here
// See both interconnect codes to see how it is connected

`include "config.v"

`define MPRJ_IO_PADS 38
`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module top (
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
    output [2:0] irq
);

/*
 * UPPER INTERCONNECT
 * External bus, clocking, outside signals
 */
wire inner_clock, inner_reset;
interconnect_outer interconnect_outer (
    .m_io_in(m_io_in),
    .m_io_out(m_io_out),
    .m_io_oeb(m_io_oeb),
    .mgt_wb_clk_i(mgt_wb_clk_i),
    .mgt_wb_rst_i(mgt_wb_rst_i),
    .mgt_wb_stb_i(mgt_wb_stb_i),
    .mgt_wb_cyc_i(mgt_wb_cyc_i),
    .mgt_wb_we_i(mgt_wb_we_i),
    .mgt_wb_sel_i(mgt_wb_sel_i),
    .mgt_wb_dat_i(mgt_wb_dat_i),
    .mgt_wb_adr_i(mgt_wb_adr_i),
    .mgt_wb_ack_o(mgt_wb_ack_o),
    .mgt_wb_dat_o(mgt_wb_dat_o),
    .la_data_in(la_data_in),
    .la_data_out(la_data_out),
    .la_oenb(la_oenb),
    .irq(irq),
    .inner_clock(inner_clock),
    .inner_reset(inner_reset),
    .inner_wb_cyc(inner_wb_cyc),
    .inner_wb_stb(inner_wb_stb),
    .inner_wb_we(inner_wb_we),
    .inner_wb_adr(inner_wb_adr),
    .inner_wb_o_dat(inner_wb_o_dat),
    .inner_wb_i_dat(inner_wb_i_dat),
    .inner_wb_ack(inner_wb_ack),
    .inner_wb_err(inner_wb_err),
    .inner_wb_sel(inner_wb_sel),
    .inner_wb_4_burst(inner_wb_4_burst),
    .inner_wb_8_burst(inner_wb_8_burst),
    .inner_ext_irq(inner_ext_irq),
    .inner_embed_mode(inner_embed_mode),
    .inner_disable(inner_disable),
    .iram_clk(iram_clk),
    .iram_addr(iram_addr),
    .iram_i_data(iram_i_data),
    .iram_o_data(iram_o_data),
    .iram_we(iram_we)
);

wire inner_wb_cyc, inner_wb_stb;
wire inner_wb_we;
wire [`WB_ADDR_W-1:0] inner_wb_adr;
wire [`WB_DATA_W-1:0] inner_wb_o_dat;
wire [`WB_DATA_W-1:0] inner_wb_i_dat;
wire inner_wb_ack, inner_wb_err;
wire [`WB_SEL_BITS-1:0] inner_wb_sel;
wire inner_wb_4_burst, inner_wb_8_burst;
wire inner_ext_irq;
wire inner_embed_mode;
wire inner_disable;

/*
 * LOWER INTERCONNECT
 * All cpu functions: core and caches
 */

interconnect_inner interconnect_inner (
    .core_clock(inner_clock),
    .core_reset(inner_reset),
    .inner_wb_cyc(inner_wb_cyc), // why some compilers don't support .* ???
    .inner_wb_stb(inner_wb_stb),
    .inner_wb_we(inner_wb_we),
    .inner_wb_adr(inner_wb_adr),
    .inner_wb_o_dat(inner_wb_o_dat),
    .inner_wb_i_dat(inner_wb_i_dat),
    .inner_wb_ack(inner_wb_ack),
    .inner_wb_err(inner_wb_err),
    .inner_wb_sel(inner_wb_sel),
    .inner_wb_4_burst(inner_wb_4_burst),
    .inner_wb_8_burst(inner_wb_8_burst),
    .inner_ext_irq(inner_ext_irq),
    .inner_embed_mode(inner_embed_mode),
    .c0_clk(c0_clk),
    .c0_rst(c0_rst),
    .c0_disable(c0_disable),
    .c0_o_req_addr(c0_o_req_addr),
    .c0_o_req_active(c0_o_req_active), 
    .c0_o_req_ppl_submit(c0_o_req_ppl_submit),
    .c0_i_req_data(c0_i_req_data),
    .c0_i_req_data_valid(c0_i_req_data_valid),
    .c0_dbg_r0(c0_dbg_r0), 
    .c0_dbg_pc(c0_dbg_pc),
    .c0_o_mem_addr(c0_o_mem_addr),
    .c0_o_mem_data(c0_o_mem_data),
    .c0_i_mem_data(c0_i_mem_data),
    .c0_o_mem_req(c0_o_mem_req),
    .c0_o_mem_we(c0_o_mem_we),
    .c0_i_mem_ack(c0_i_mem_ack),
    .c0_o_mem_sel(c0_o_mem_sel),
    .c0_i_irq(c0_i_irq),
    .c0_o_c_instr_page(c0_o_c_instr_page),
    .c0_o_c_data_page(c0_o_c_data_page),
    .c0_sr_bus_addr(c0_sr_bus_addr),
    .c0_sr_bus_data_o(c0_sr_bus_data_o),
    .c0_sr_bus_we(c0_sr_bus_we),
    .c0_o_icache_flush(c0_o_icache_flush),
    .c0_i_mem_exception(c0_i_mem_exception),
    .c0_i_mc_core_int(c0_i_mc_core_int),
    .c0_i_core_int_sreg(c0_i_core_int_sreg),
    .c0_o_c_instr_long(c0_o_c_instr_long),
    .c0_o_instr_long_addr(c0_o_instr_long_addr),
    .c0_o_mem_long_mode(c0_o_mem_long),
    .c0_o_mem_high_addr(c0_o_mem_addr_high),
    .c0_dbg_out(c0_dbg_out),
    .c0_dbg_in(c0_dbg_in),
    .c1_clk(c1_clk),
    .c1_rst(c1_rst),
    .c1_disable(c1_disable),
    .c1_o_req_addr(c1_o_req_addr),
    .c1_o_req_active(c1_o_req_active),
    .c1_o_req_ppl_submit(c1_o_req_ppl_submit),
    .c1_i_req_data(c1_i_req_data),
    .c1_i_req_data_valid(c1_i_req_data_valid),
    .c1_dbg_r0(c1_dbg_r0),
    .c1_dbg_pc(c1_dbg_pc),
    .c1_o_mem_addr(c1_o_mem_addr),
    .c1_o_mem_data(c1_o_mem_data),
    .c1_i_mem_data(c1_i_mem_data),
    .c1_o_mem_req(c1_o_mem_req),
    .c1_o_mem_we(c1_o_mem_we),
    .c1_i_mem_ack(c1_i_mem_ack),
    .c1_o_mem_sel(c1_o_mem_sel),
    .c1_i_irq(c1_i_irq),
    .c1_o_c_instr_page(c1_o_c_instr_page),
    .c1_o_c_data_page(c1_o_c_data_page),
    .c1_sr_bus_addr(c1_sr_bus_addr),
    .c1_sr_bus_data_o(c1_sr_bus_data_o),
    .c1_sr_bus_we(c1_sr_bus_we),
    .c1_o_icache_flush(c1_o_icache_flush),
    .c1_i_mem_exception(c1_i_mem_exception),
    .c1_i_mc_core_int(c1_i_mc_core_int),
    .c1_i_core_int_sreg(c1_i_core_int_sreg),
    .c1_o_c_instr_long(c1_o_c_instr_long),
    .c1_o_instr_long_addr(c1_o_instr_long_addr),
    .c1_o_mem_long_mode(c1_o_mem_long),
    .c1_o_mem_high_addr(c1_o_mem_addr_high),
    .c1_dbg_out(c1_dbg_out),
    .c1_dbg_in(c1_dbg_in),
    .ic0_clk(ic0_clk),
    .ic0_rst(ic0_rst),
    .ic0_mem_req(ic0_mem_req),
    .ic0_mem_ack(ic0_mem_ack),
    .ic0_mem_addr(ic0_mem_addr),
    .ic0_mem_data(ic0_mem_data),
    .ic0_mem_ppl_submit(ic0_mem_ppl_submit),
    .ic0_mem_cache_flush(ic0_mem_cache_flush),
    .ic0_wb_cyc(ic0_wb_cyc),
    .ic0_wb_stb(ic0_wb_stb),
    .ic0_wb_i_dat(ic0_wb_i_dat),
    .ic0_wb_adr(ic0_wb_adr),
    .ic0_wb_we(ic0_wb_we),
    .ic0_wb_ack(ic0_wb_ack),
    .ic0_wb_sel(ic0_wb_sel),
    .ic0_wb_err(ic0_wb_err),
    .ic1_clk(ic1_clk),
    .ic1_rst(ic1_rst),
    .ic1_mem_req(ic1_mem_req),
    .ic1_mem_ack(ic1_mem_ack),
    .ic1_mem_addr(ic1_mem_addr),
    .ic1_mem_data(ic1_mem_data),
    .ic1_mem_ppl_submit(ic1_mem_ppl_submit),
    .ic1_mem_cache_flush(ic1_mem_cache_flush),
    .ic1_wb_cyc(ic1_wb_cyc),
    .ic1_wb_stb(ic1_wb_stb),
    .ic1_wb_i_dat(ic1_wb_i_dat),
    .ic1_wb_adr(ic1_wb_adr),
    .ic1_wb_we(ic1_wb_we),
    .ic1_wb_ack(ic1_wb_ack),
    .ic1_wb_sel(ic1_wb_sel),
    .ic1_wb_err(ic1_wb_err),
    .dcache_clk(dcache_clk),
    .dcache_rst(dcache_rst),
    .dcache_mem_req(dcache_mem_req),
    .dcache_mem_we(dcache_mem_we),
    .dcache_mem_ack(dcache_mem_ack),
    .dcache_mem_addr(dcache_mem_addr),
    .dcache_mem_i_data(dcache_mem_i_data),
    .dcache_mem_o_data(dcache_mem_o_data),
    .dcache_mem_sel(dcache_mem_sel),
    .dcache_mem_cache_enable(dcache_mem_cache_enable),
    .dcache_mem_exception(dcache_mem_exception),
    .dcache_wb_cyc(dcache_wb_cyc),
    .dcache_wb_stb(dcache_wb_stb),
    .dcache_wb_i_dat(dcache_wb_i_dat),
    .dcache_wb_o_dat(dcache_wb_o_dat),
    .dcache_wb_adr(dcache_wb_adr),
    .dcache_wb_we(dcache_wb_we),
    .dcache_wb_sel(dcache_wb_sel),
    .dcache_wb_4_burst(dcache_wb_4_burst),
    .dcache_wb_ack(dcache_wb_ack),
    .dcache_wb_err(dcache_wb_err),
    .inner_disable(inner_disable)
);

// CORES

wire c0_clk, c0_rst;
wire [`RW-1:0] c0_o_req_addr;
wire c0_o_req_active;
wire [`I_SIZE-1:0] c0_i_req_data;
wire c0_i_req_data_valid;
wire c0_o_req_ppl_submit;
wire [`RW-1:0] c0_o_mem_addr, c0_o_mem_data, c0_i_mem_data;
wire c0_o_mem_req, c0_o_mem_we, c0_i_mem_ack;
wire [`ADDR_BYTES-1:0] c0_o_mem_sel;
wire [`RW-1:0] c0_sr_bus_addr, c0_sr_bus_data_o, c0_i_core_int_sreg;
wire c0_sr_bus_we;
wire c0_o_c_instr_page, c0_o_c_data_page, c0_o_icache_flush, c0_i_mem_exception;
wire [`RW-1:0] c0_dbg_r0, c0_dbg_pc;
wire [35:0] c0_dbg_out;
wire [3:0] c0_dbg_in;
wire c0_i_mc_core_int, c0_disable, c0_i_irq;
wire c0_o_c_instr_long;
wire [7:0] c0_o_instr_long_addr;
wire c0_o_mem_long;
wire [7:0] c0_o_mem_addr_high;

core #(.CORENO(0), .INT_VEC(1)) core_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(c0_clk), .i_rst(c0_rst), .o_req_addr(c0_o_req_addr), .o_req_active(c0_o_req_active), .i_req_data(c0_i_req_data), .i_req_data_valid(c0_i_req_data_valid), .o_req_ppl_submit(c0_o_req_ppl_submit),
    .o_mem_addr(c0_o_mem_addr), .o_mem_data(c0_o_mem_data), .i_mem_data(c0_i_mem_data), .o_mem_req(c0_o_mem_req), .o_mem_we(c0_o_mem_we), .i_mem_ack(c0_i_mem_ack),
    .dbg_r0(c0_dbg_r0), .dbg_pc(c0_dbg_pc), .i_irq(c0_i_irq), .o_c_instr_page(c0_o_c_instr_page), .sr_bus_addr(c0_sr_bus_addr),
    .sr_bus_data_o(c0_sr_bus_data_o), .sr_bus_we(c0_sr_bus_we), .o_icache_flush(c0_o_icache_flush), .o_mem_sel(c0_o_mem_sel), .o_c_data_page(c0_o_c_data_page),
    .i_mem_exception(c0_i_mem_exception), .dbg_out(c0_dbg_out), .dbg_in(c0_dbg_in), .i_disable(c0_disable), .i_mc_core_int(c0_i_mc_core_int), .i_core_int_sreg(c0_i_core_int_sreg),
    .o_c_instr_long(c0_o_c_instr_long), .o_instr_long_addr(c0_o_instr_long_addr), .o_mem_long(c0_o_mem_long), .o_mem_addr_high(c0_o_mem_addr_high)
);

wire c1_clk, c1_rst;
wire [`RW-1:0] c1_o_req_addr;
wire c1_o_req_active;
wire [`I_SIZE-1:0] c1_i_req_data;
wire c1_i_req_data_valid;
wire c1_o_req_ppl_submit;
wire [`RW-1:0] c1_o_mem_addr, c1_o_mem_data, c1_i_mem_data;
wire c1_o_mem_req, c1_o_mem_we, c1_i_mem_ack;
wire [`ADDR_BYTES-1:0] c1_o_mem_sel;
wire [`RW-1:0] c1_sr_bus_addr, c1_sr_bus_data_o, c1_i_core_int_sreg;
wire c1_sr_bus_we;
wire c1_o_c_instr_page, c1_o_c_data_page, c1_o_icache_flush, c1_i_mem_exception;
wire [`RW-1:0] c1_dbg_r0, c1_dbg_pc;
wire [35:0] c1_dbg_out;
wire [3:0] c1_dbg_in;
wire c1_i_mc_core_int, c1_disable, c1_i_irq;
wire c1_o_c_instr_long;
wire [7:0] c1_o_instr_long_addr;
wire c1_o_mem_long;
wire [7:0] c1_o_mem_addr_high;

core #(.CORENO(1), .INT_VEC(2)) core_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(c1_clk), .i_rst(c1_rst), .o_req_addr(c1_o_req_addr), .o_req_active(c1_o_req_active), .i_req_data(c1_i_req_data), .i_req_data_valid(c1_i_req_data_valid), .o_req_ppl_submit(c1_o_req_ppl_submit),
    .o_mem_addr(c1_o_mem_addr), .o_mem_data(c1_o_mem_data), .i_mem_data(c1_i_mem_data), .o_mem_req(c1_o_mem_req), .o_mem_we(c1_o_mem_we), .i_mem_ack(c1_i_mem_ack),
    .dbg_r0(c1_dbg_r0), .dbg_pc(c1_dbg_pc), .i_irq(c1_i_irq), .o_c_instr_page(c1_o_c_instr_page), .sr_bus_addr(c1_sr_bus_addr),
    .sr_bus_data_o(c1_sr_bus_data_o), .sr_bus_we(c1_sr_bus_we), .o_icache_flush(c1_o_icache_flush), .o_mem_sel(c1_o_mem_sel), .o_c_data_page(c1_o_c_data_page),
    .i_mem_exception(c1_i_mem_exception), .dbg_out(c1_dbg_out), .dbg_in(c1_dbg_in), .i_disable(c1_disable), .i_mc_core_int(c1_i_mc_core_int), .i_core_int_sreg(c1_i_core_int_sreg),
    .o_c_instr_long(c1_o_c_instr_long), .o_instr_long_addr(c1_o_instr_long_addr), .o_mem_long(c1_o_mem_long), .o_mem_addr_high(c1_o_mem_addr_high)
);

// DCACHE

wire dcache_clk;
wire dcache_rst;
wire dcache_mem_req;
wire dcache_mem_we;
wire dcache_mem_ack;
wire [`WB_ADDR_W-1:0] dcache_mem_addr;
wire [`RW-1:0] dcache_mem_i_data;
wire [`RW-1:0] dcache_mem_o_data;
wire [1:0] dcache_mem_sel;
wire dcache_mem_cache_enable;
wire dcache_mem_exception;
wire dcache_wb_cyc;
wire dcache_wb_stb;
wire [`RW-1:0] dcache_wb_i_dat;
wire [`RW-1:0] dcache_wb_o_dat;
wire [`WB_ADDR_W-1:0]  dcache_wb_adr;
wire dcache_wb_we;
wire [1:0] dcache_wb_sel;
wire dcache_wb_4_burst;
wire dcache_wb_ack;
wire dcache_wb_err;

dcache dcache (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(dcache_clk), .i_rst(dcache_rst), .mem_req(dcache_mem_req), .mem_addr(dcache_mem_addr), .mem_i_data(dcache_mem_i_data), .mem_o_data(dcache_mem_o_data), .mem_we(dcache_mem_we), .mem_ack(dcache_mem_ack), .mem_sel(dcache_mem_sel),
    .wb_cyc(dcache_wb_cyc), .wb_stb(dcache_wb_stb), .wb_we(dcache_wb_we), .wb_ack(dcache_wb_ack), .wb_i_dat(dcache_wb_i_dat), .wb_o_dat(dcache_wb_o_dat), .wb_adr(dcache_wb_adr), .wb_sel(dcache_wb_sel), .mem_cache_enable(dcache_mem_cache_enable), .wb_4_burst(dcache_wb_4_burst), .wb_err(dcache_wb_err), .mem_exception(dcache_mem_exception)
);

// ICACHES

wire ic0_clk, ic0_rst;
wire ic0_mem_req;
wire ic0_mem_ack;
wire [`RW-1:0] ic0_mem_addr;
wire [`I_SIZE-1:0] ic0_mem_data;
wire ic0_mem_ppl_submit;
wire ic0_mem_cache_flush;
wire ic0_wb_cyc;
wire ic0_wb_stb;
wire [`RW-1:0] ic0_wb_i_dat;
wire [`RW-1:0]  ic0_wb_adr;
wire ic0_wb_we;
wire ic0_wb_ack;
wire [1:0] ic0_wb_sel;
wire ic0_wb_err;

icache icache_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(ic0_clk), .i_rst(ic0_rst), .mem_req(ic0_mem_req), .mem_addr(ic0_mem_addr), .mem_data(ic0_mem_data), .mem_ack(ic0_mem_ack), .mem_ppl_submit(ic0_mem_ppl_submit),
    .wb_cyc(ic0_wb_cyc), .wb_stb(ic0_wb_stb), .wb_we(ic0_wb_we), .wb_ack(ic0_wb_ack), .wb_i_dat(ic0_wb_i_dat), .wb_adr(ic0_wb_adr), .wb_sel(ic0_wb_sel), .mem_cache_flush(ic0_mem_cache_flush), .wb_err(ic0_wb_err)
);

wire ic1_clk, ic1_rst;
wire ic1_mem_req;
wire ic1_mem_ack;
wire [`RW-1:0] ic1_mem_addr;
wire [`I_SIZE-1:0] ic1_mem_data;
wire ic1_mem_ppl_submit;
wire ic1_mem_cache_flush;
wire ic1_wb_cyc;
wire ic1_wb_stb;
wire [`RW-1:0] ic1_wb_i_dat;
wire [`RW-1:0]  ic1_wb_adr;
wire ic1_wb_we;
wire ic1_wb_ack;
wire [1:0] ic1_wb_sel;
wire ic1_wb_err;

icache icache_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(ic1_clk), .i_rst(ic1_rst), .mem_req(ic1_mem_req), .mem_addr(ic1_mem_addr), .mem_data(ic1_mem_data), .mem_ack(ic1_mem_ack), .mem_ppl_submit(ic1_mem_ppl_submit),
    .wb_cyc(ic1_wb_cyc), .wb_stb(ic1_wb_stb), .wb_we(ic1_wb_we), .wb_ack(ic1_wb_ack), .wb_i_dat(ic1_wb_i_dat), .wb_adr(ic1_wb_adr), .wb_sel(ic1_wb_sel), .mem_cache_flush(ic1_mem_cache_flush), .wb_err(ic1_wb_err)
);

// Internal ram
wire iram_clk;
wire [8:0] iram_addr;
wire [`RW-1:0] iram_i_data, iram_o_data;
wire iram_we;
int_ram int_ram (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(iram_clk),
    .i_addr(iram_addr),
    .i_data(iram_i_data),
    .o_data(iram_o_data),
    .i_we(iram_we)
);

endmodule
