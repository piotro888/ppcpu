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
    input [3:0] dbg_in
);

// fetch input singals
wire [`RW-1:0] fetch_req_addr;
wire fetch_req_active;
wire [`I_SIZE-1:0] fetch_req_data;
wire fetch_req_ack;
wire fetch_ppl_submit;

// data memory connections
wire [`RW-1:0] data_mem_addr, data_o_mem_data, data_i_mem_data;
wire [`WB_ADDR_W-1:0] data_mem_addr_paged;
wire data_mem_req, data_mem_we, data_mem_ack;
wire [`ADDR_BYTES-1:0] data_mem_sel;

wire [`RW-1:0] sr_bus_addr, sr_bus_data_o;
wire sr_bus_we;
wire cc_instr_page, cc_data_page, icache_flush, dcache_exception;
wire data_cacheable;
wire [35:0] dbg_out_core;

core core (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .o_req_addr(fetch_req_addr), .o_req_active(fetch_req_active), .i_req_data(fetch_req_data), .i_req_data_valid(fetch_req_ack),
    .o_mem_addr(data_mem_addr), .o_mem_data(data_o_mem_data), .i_mem_data(data_i_mem_data), .o_mem_req(data_mem_req), .o_mem_we(data_mem_we), .i_mem_ack(data_mem_ack),
    .dbg_r0(dbg_r0), .dbg_pc(dbg_pc), .i_irq(i_irq), .o_req_ppl_submit(fetch_ppl_submit), .o_c_instr_page(cc_instr_page), .sr_bus_addr(sr_bus_addr),
    .sr_bus_data_o(sr_bus_data_o), .sr_bus_we(sr_bus_we), .o_icache_flush(icache_flush), .o_mem_sel(data_mem_sel), .o_c_data_page(cc_data_page),
    .i_mem_exception(dcache_exception), .dbg_out(dbg_out_core), .dbg_in(dbg_in));

wire fetch_wb_cyc, fetch_wb_stb, fetch_wb_we;
wire fetch_wb_ack, fetch_wb_err, fetch_wb_rty;
wire [`WB_DATA_W-1:0] fetch_wb_o_dat;
wire [`RW-1:0]  fetch_wb_adr;
wire [`WB_ADDR_W-1:0]  fetch_wb_adr_paged;
wire [`WB_SEL_BITS-1:0] fetch_wb_sel;

wire data_wb_cyc, data_wb_stb, data_wb_we;
wire data_wb_ack, data_wb_err, data_wb_rty;
wire [`WB_DATA_W-1:0] data_wb_o_dat;
wire [`WB_ADDR_W-1:0]  data_wb_adr;
wire [`WB_SEL_BITS-1:0] data_wb_sel;
wire data_wb_4_burst;

icache icache(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_req(fetch_req_active), .mem_addr(fetch_req_addr), .mem_data(fetch_req_data), .mem_ack(fetch_req_ack), .mem_ppl_submit(fetch_ppl_submit),
    .wb_cyc(fetch_wb_cyc), .wb_stb(fetch_wb_stb), .wb_we(fetch_wb_we), .wb_ack(fetch_wb_ack), .wb_i_dat(wb_i_dat), .wb_adr(fetch_wb_adr), .wb_sel(fetch_wb_sel), .mem_cache_flush(icache_flush), .wb_err(fetch_wb_err));

dcache dcache(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_req(data_mem_req), .mem_addr(data_mem_addr_paged), .mem_i_data(data_o_mem_data), .mem_o_data(data_i_mem_data), .mem_we(data_mem_we), .mem_ack(data_mem_ack), .mem_sel(data_mem_sel),
    .wb_cyc(data_wb_cyc), .wb_stb(data_wb_stb), .wb_we(data_wb_we), .wb_ack(data_wb_ack), .wb_i_dat(wb_i_dat), .wb_o_dat(data_wb_o_dat), .wb_adr(data_wb_adr), .wb_sel(data_wb_sel), .mem_cache_enable(data_cacheable), .wb_4_burst(data_wb_4_burst), .wb_err(data_wb_err), .mem_exception(dcache_exception));

wire wb0_8_burst, wb1_4_burst, wb1_8_burst;
upper_core_logic upper_core_logic (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .fetch_wb_adr(fetch_wb_adr), .data_mem_addr(data_mem_addr),
    .data_mem_addr_paged(data_mem_addr_paged), .fetch_wb_adr_paged(fetch_wb_adr_paged),
    .sr_bus_addr(sr_bus_addr), .sr_bus_data_o(sr_bus_data_o),
    .sr_bus_we(sr_bus_we), .cc_instr_page(cc_instr_page), .cc_data_page(cc_data_page), .data_cacheable(data_cacheable),
    .fetch_wb_o_dat(fetch_wb_o_dat), .wb0_8_burst(wb0_8_burst), .wb1_4_burst(wb1_4_burst), .wb1_8_burst(wb1_8_burst)
);

wire arb_sel;
wishbone_arbiter wb_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_wb0_cyc(data_wb_cyc), .i_wb1_cyc(fetch_wb_cyc), .o_wb_cyc(wb_cyc), .o_sel_sig(arb_sel),
    .wb0_stb(data_wb_stb), .wb0_we(data_wb_we), .wb0_ack(data_wb_ack), .wb0_adr(data_wb_adr), .wb0_sel(data_wb_sel), .wb0_err(data_wb_err), .wb0_o_dat(data_wb_o_dat), .wb0_4_burst(data_wb_4_burst), .wb0_8_burst(wb0_8_burst),
    .wb1_stb(fetch_wb_stb), .wb1_we(fetch_wb_we), .wb1_ack(fetch_wb_ack), .wb1_adr(fetch_wb_adr_paged), .wb1_sel(fetch_wb_sel), .wb1_err(fetch_wb_err), .wb1_o_dat(fetch_wb_o_dat), .wb1_4_burst(wb1_4_burst), .wb1_8_burst(wb1_8_burst),
    .owb_stb(wb_stb), .owb_we(wb_we), .owb_ack(wb_ack), .owb_adr(wb_adr), .owb_sel(wb_sel), .owb_err(wb_err), .owb_o_dat(wb_o_dat), .owb_4_burst(wb_4_burst), .owb_8_burst(wb_8_burst));

assign dbg_out = {wb_adr, wb_we, wb_stb, dbg_out_core};

endmodule

`include "wishbone/wishbone_arbiter.v"
