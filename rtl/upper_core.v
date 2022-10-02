/* Core with all memory access related modules */

`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module upper_core (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input wire i_clk,
    input wire i_rst,

    output wb_cyc,
    output wb_stb,
    output [`WB_DATA_W-1:0] wb_o_dat,
    input [`WB_DATA_W-1:0] wb_i_dat,
    output [`WB_ADDR_W-1:0]  wb_adr,
    output wb_we,
    input wb_ack,
    input wb_err,
    input wb_rty,
    output [`WB_SEL_BITS-1:0] wb_sel,
    output wb_8_burst, wb_4_burst,

    input i_irq,
    output [`RW-1:0] dbg_r0, dbg_pc,

    output [61:0] dbg_out,
    input [4:0] dbg_in
);

// fetch input singals
wire [`RW-1:0] fetch0_req_addr, fetch1_req_addr;
wire fetch0_req_active, fetch1_req_active;
wire [`I_SIZE-1:0] fetch0_req_data, fetch1_req_data;
wire fetch0_req_ack, fetch1_req_ack;
wire fetch0_ppl_submit, fetch1_ppl_submit;

// data memory connections
wire [`RW-1:0] data0_mem_addr, data0_o_mem_data, data0_i_mem_data;
wire [`WB_ADDR_W-1:0] data0_mem_addr_paged;
wire data0_mem_req, data0_mem_we, data0_mem_ack;
wire [`ADDR_BYTES-1:0] data0_mem_sel;

wire [`RW-1:0] sr_bus_addr0, sr_bus_data_o0, sr_bus_data_i0;
wire sr_bus_we0;
wire cc_instr_page0, cc_data_page0, icache0_flush, dcache_exception0;
wire data_cacheable0;
wire [35:0] dbg_out_core0, dbg_out_core;

wire [`RW-1:0] data1_mem_addr, data1_o_mem_data, data1_i_mem_data;
wire [`WB_ADDR_W-1:0] data1_mem_addr_paged;
wire data1_mem_req, data1_mem_we, data1_mem_ack;
wire [`ADDR_BYTES-1:0] data1_mem_sel;

wire [`RW-1:0] sr_bus_addr1, sr_bus_data_o1, sr_bus_data_i1;
wire sr_bus_we1;
wire cc_instr_page1, cc_data_page1, icache1_flush, dcache_exception1;
wire data_cacheable1;
wire [35:0] dbg_out_core1;

wire i_irq0, i_irq1;
wire [`RW-1:0] dbg0_r0, dbg0_pc, dbg1_r0, dbg1_pc;

core #(.CORENO(0), .INT_VEC(1)) core_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .o_req_addr(fetch0_req_addr), .o_req_active(fetch0_req_active), .i_req_data(fetch0_req_data), .i_req_data_valid(fetch0_req_ack),
    .o_mem_addr(data0_mem_addr), .o_mem_data(data0_o_mem_data), .i_mem_data(data0_i_mem_data), .o_mem_req(data0_mem_req), .o_mem_we(data0_mem_we), .i_mem_ack(data0_mem_ack),
    .dbg_r0(dbg_r0), .dbg_pc(dbg_pc), .i_irq(i_irq0), .o_req_ppl_submit(fetch0_ppl_submit), .o_c_instr_page(cc_instr_page0), .sr_bus_addr(sr_bus_addr0),
    .sr_bus_data_o(sr_bus_data_o0), .sr_bus_we(sr_bus_we0), .o_icache_flush(icache0_flush), .o_mem_sel(data0_mem_sel), .o_c_data_page(cc_data_page0),
    .i_mem_exception(dcache_exception0), .dbg_out(dbg_out_core0), .dbg_in(dbg_in[3:0]), .i_disable(c0_disable), .i_mc_core_int(c0_core_int), .i_core_int_sreg(sr_bus_data_i0));

core #(.CORENO(1), .INT_VEC(2)) core_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .o_req_addr(fetch1_req_addr), .o_req_active(fetch1_req_active), .i_req_data(fetch1_req_data), .i_req_data_valid(fetch1_req_ack),
    .o_mem_addr(data1_mem_addr), .o_mem_data(data1_o_mem_data), .i_mem_data(data1_i_mem_data), .o_mem_req(data1_mem_req), .o_mem_we(data1_mem_we), .i_mem_ack(data1_mem_ack),
    .dbg_r0(dbg1_r0), .dbg_pc(dbg1_pc), .i_irq(i_irq1), .o_req_ppl_submit(fetch1_ppl_submit), .o_c_instr_page(cc_instr_page1), .sr_bus_addr(sr_bus_addr1),
    .sr_bus_data_o(sr_bus_data_o1), .sr_bus_we(sr_bus_we1), .o_icache_flush(icache1_flush), .o_mem_sel(data1_mem_sel), .o_c_data_page(cc_data_page1),
    .i_mem_exception(dcache_exception1), .dbg_out(dbg_out_core1), .dbg_in(dbg_in[3:0]), .i_disable(c1_disable), .i_mc_core_int(c1_core_int), .i_core_int_sreg(sr_bus_data_i1));

wire fetch0_wb_cyc, fetch0_wb_stb, fetch0_wb_we;
wire fetch0_wb_ack, fetch0_wb_err, fetch0_wb_rty;
wire [`WB_DATA_W-1:0] fetch0_wb_o_dat;
wire [`RW-1:0]  fetch0_wb_adr;
wire [`WB_ADDR_W-1:0]  fetch0_wb_adr_paged;
wire [`WB_SEL_BITS-1:0] fetch0_wb_sel;

wire fetch1_wb_cyc, fetch1_wb_stb, fetch1_wb_we;
wire fetch1_wb_ack, fetch1_wb_err, fetch1_wb_rty;
wire [`WB_DATA_W-1:0] fetch1_wb_o_dat;
wire [`RW-1:0]  fetch1_wb_adr;
wire [`WB_ADDR_W-1:0]  fetch1_wb_adr_paged;
wire [`WB_SEL_BITS-1:0] fetch1_wb_sel;

wire data_wb_cyc, data_wb_stb, data_wb_we;
wire data_wb_ack, data_wb_err, data_wb_rty;
wire [`WB_DATA_W-1:0] data_wb_o_dat;
wire [`WB_ADDR_W-1:0]  data_wb_adr;
wire [`WB_SEL_BITS-1:0] data_wb_sel;
wire data_wb_4_burst;

icache icache_0(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_req(fetch0_req_active), .mem_addr(fetch0_req_addr), .mem_data(fetch0_req_data), .mem_ack(fetch0_req_ack), .mem_ppl_submit(fetch0_ppl_submit),
    .wb_cyc(fetch0_wb_cyc), .wb_stb(fetch0_wb_stb), .wb_we(fetch0_wb_we), .wb_ack(fetch0_wb_ack), .wb_i_dat(wb_i_dat), .wb_adr(fetch0_wb_adr), .wb_sel(fetch0_wb_sel), .mem_cache_flush(icache0_flush), .wb_err(fetch0_wb_err));

icache icache_1(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_req(fetch1_req_active), .mem_addr(fetch1_req_addr), .mem_data(fetch1_req_data), .mem_ack(fetch1_req_ack), .mem_ppl_submit(fetch1_ppl_submit),
    .wb_cyc(fetch1_wb_cyc), .wb_stb(fetch1_wb_stb), .wb_we(fetch1_wb_we), .wb_ack(fetch1_wb_ack), .wb_i_dat(wb_i_dat), .wb_adr(fetch1_wb_adr), .wb_sel(fetch1_wb_sel), .mem_cache_flush(icache1_flush), .wb_err(fetch1_wb_err));

wire fetch_wb_cyc, fetch_wb_stb, fetch_wb_we;
wire fetch_wb_ack, fetch_wb_err;
wire [`WB_DATA_W-1:0] fetch_wb_o_dat;
wire [`WB_ADDR_W-1:0]  fetch_wb_adr;
wire [`WB_ADDR_W-1:0]  fetch_wb_adr;
wire [`WB_SEL_BITS-1:0] fetch_wb_sel;
wire fetch_arb_sel;
wire fetch_wb_4_burst, fetch_wb_8_burst;

wishbone_arbiter icache_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_wb0_cyc(fetch0_wb_cyc), .i_wb1_cyc(fetch1_wb_cyc), .o_wb_cyc(fetch_wb_cyc), .o_sel_sig(fetch_arb_sel),
    .wb0_stb(fetch0_wb_stb), .wb0_we(fetch0_wb_we), .wb0_ack(fetch0_wb_ack), .wb0_adr(fetch0_wb_adr_paged), .wb0_sel(fetch0_wb_sel), .wb0_err(fetch0_wb_err), .wb0_o_dat(fetch0_wb_o_dat), .wb0_4_burst(fetch0_wb_4_burst), .wb0_8_burst(fetch0_wb_8_burst),
    .wb1_stb(fetch1_wb_stb), .wb1_we(fetch1_wb_we), .wb1_ack(fetch1_wb_ack), .wb1_adr(fetch1_wb_adr_paged), .wb1_sel(fetch1_wb_sel), .wb1_err(fetch1_wb_err), .wb1_o_dat(fetch1_wb_o_dat), .wb1_4_burst(fetch1_wb_4_burst), .wb1_8_burst(fetch1_wb_8_burst),
    .owb_stb(fetch_wb_stb), .owb_we(fetch_wb_we), .owb_ack(fetch_wb_ack), .owb_adr(fetch_wb_adr), .owb_sel(fetch_wb_sel), .owb_err(fetch_wb_err), .owb_o_dat(fetch_wb_o_dat), .owb_4_burst(fetch_wb_4_burst), .owb_8_burst(fetch_wb_8_burst));

wire [`RW-1:0] data_o_mem_data, data_i_mem_data;
wire [`WB_ADDR_W-1:0] data_mem_addr;
wire data_mem_req, data_mem_we, data_mem_ack;
wire [`ADDR_BYTES-1:0] data_mem_sel;
wire data_cacheable, data_exception;

mem_dcache_arb mem_dcache_arb (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    .i_rst(i_rst),

    .mem_req(data_mem_req), .mem_we(data_mem_we), .mem_ack(data_mem_ack), .mem_addr(data_mem_addr), .mem_i_data(data_i_mem_data), .mem_o_data(data_o_mem_data), .mem_sel(data_mem_sel), .mem_cache_enable(data_cacheable), .mem_exception(data_exception),
    .mem0_req(data0_mem_req), .mem0_we(data0_mem_we), .mem0_ack(data0_mem_ack), .mem0_addr(data0_mem_addr_paged), .mem0_i_data(data0_i_mem_data), .mem0_o_data(data0_o_mem_data), .mem0_sel(data0_mem_sel), .mem0_cache_enable(data_cacheable0), .mem0_exception(dcache_exception0),
    .mem1_req(data1_mem_req), .mem1_we(data1_mem_we), .mem1_ack(data1_mem_ack), .mem1_addr(data1_mem_addr_paged), .mem1_i_data(data1_i_mem_data), .mem1_o_data(data1_o_mem_data), .mem1_sel(data1_mem_sel), .mem1_cache_enable(data_cacheable1), .mem1_exception(dcache_exception1)
);

dcache dcache(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_req(data_mem_req), .mem_addr(data_mem_addr), .mem_i_data(data_o_mem_data), .mem_o_data(data_i_mem_data), .mem_we(data_mem_we), .mem_ack(data_mem_ack), .mem_sel(data_mem_sel),
    .wb_cyc(data_wb_cyc), .wb_stb(data_wb_stb), .wb_we(data_wb_we), .wb_ack(data_wb_ack), .wb_i_dat(wb_i_dat), .wb_o_dat(data_wb_o_dat), .wb_adr(data_wb_adr), .wb_sel(data_wb_sel), .mem_cache_enable(data_cacheable), .wb_4_burst(data_wb_4_burst), .wb_err(data_wb_err), .mem_exception(data_exception));

wire data_wb_8_burst, fetch0_wb_4_burst, fetch1_wb_4_burst, fetch0_wb_8_burst, fetch1_wb_8_burst;
wire c1_disable;
wire [`RW-1:0] sr0_bus_data_i, sr1_bus_data_i;

wire c0_disable, c0_core_int, c1_core_int;

upper_core_logic upper_core_logic (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .fetch0_wb_adr(fetch0_wb_adr), .data0_mem_addr(data0_mem_addr), .fetch1_wb_adr(fetch1_wb_adr), .data1_mem_addr(data1_mem_addr),
    .data0_mem_addr_paged(data0_mem_addr_paged), .fetch0_wb_adr_paged(fetch0_wb_adr_paged), .data1_mem_addr_paged(data1_mem_addr_paged), .fetch1_wb_adr_paged(fetch1_wb_adr_paged),
    .sr0_bus_addr(sr_bus_addr0), .sr0_bus_data_o(sr_bus_data_o0), .sr1_bus_addr(sr_bus_addr1), .sr1_bus_data_o(sr_bus_data_o1),
    .sr0_bus_we(sr_bus_we0), .sr1_bus_we(sr_bus_we1), .cc0_instr_page(cc_instr_page0), .cc1_instr_page(cc_instr_page1), .cc0_data_page(cc_data_page0), .cc1_data_page(cc_data_page1), .data0_cacheable(data_cacheable0), .data1_cacheable(data_cacheable1),
    .fetch0_wb_o_dat(fetch0_wb_o_dat), .fetch1_wb_o_dat(fetch0_wb_o_dat), .data_wb_8_burst(data_wb_8_burst), .fetch0_4_burst(fetch0_wb_4_burst), .fetch1_4_burst(fetch1_wb_4_burst), .fetch0_8_burst(fetch0_wb_8_burst), .fetch1_8_burst(fetch1_wb_8_burst),
    .c1_disable(c1_disable), .sr0_bus_data_i(sr0_bus_data_i), .sr1_bus_data_i(sr1_bus_data_i), .c0_disable(c0_disable), .c0_core_int(c0_core_int), .c1_core_int(c1_core_int),
    .c0_dbg(dbg_out_core0), .c1_dbg(dbg_out_core1), .dbg_out(dbg_out_core), .dbg_sel(dbg_in[4]), .i_irq(i_irq), .o_irq0(i_irq0), .o_irq1(i_irq1)
);

wire arb_sel;
wishbone_arbiter wb_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_wb0_cyc(data_wb_cyc), .i_wb1_cyc(fetch_wb_cyc), .o_wb_cyc(wb_cyc), .o_sel_sig(arb_sel),
    .wb0_stb(data_wb_stb), .wb0_we(data_wb_we), .wb0_ack(data_wb_ack), .wb0_adr(data_wb_adr), .wb0_sel(data_wb_sel), .wb0_err(data_wb_err), .wb0_o_dat(data_wb_o_dat), .wb0_4_burst(data_wb_4_burst), .wb0_8_burst(data_wb_8_burst),
    .wb1_stb(fetch_wb_stb), .wb1_we(fetch_wb_we), .wb1_ack(fetch_wb_ack), .wb1_adr(fetch_wb_adr), .wb1_sel(fetch_wb_sel), .wb1_err(fetch_wb_err), .wb1_o_dat(fetch_wb_o_dat), .wb1_4_burst(fetch_wb_4_burst), .wb1_8_burst(fetch_wb_8_burst),
    .owb_stb(wb_stb), .owb_we(wb_we), .owb_ack(wb_ack), .owb_adr(wb_adr), .owb_sel(wb_sel), .owb_err(wb_err), .owb_o_dat(wb_o_dat), .owb_4_burst(wb_4_burst), .owb_8_burst(wb_8_burst));

assign dbg_out = {wb_adr, wb_we, wb_stb, dbg_out_core};

endmodule

`include "wishbone/wishbone_arbiter.v"
