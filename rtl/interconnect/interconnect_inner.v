/*
 * Everything connecting cores, caches together up to main wishbone bus
 */

`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module interconnect_inner (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input core_clock,
    input core_reset,

    // INTERCONNECT WB OUTPUT
    output inner_wb_cyc, inner_wb_stb,
    output inner_wb_we,
    output [`WB_ADDR_W-1:0] inner_wb_adr,
    output [`WB_DATA_W-1:0] inner_wb_o_dat,
    input [`WB_DATA_W-1:0] inner_wb_i_dat,
    input inner_wb_ack, inner_wb_err,
    output [`WB_SEL_BITS-1:0] inner_wb_sel,
    output inner_wb_4_burst, inner_wb_8_burst,
    input inner_ext_irq,
    input inner_embed_mode,
    input inner_disable,

    // CORE 0
    output c0_clk,
    output c0_rst,
    output c0_disable,
    input [`RW-1:0] c0_o_req_addr,
    input c0_o_req_active, c0_o_req_ppl_submit,
    output [`I_SIZE-1:0] c0_i_req_data,
    output c0_i_req_data_valid,
    input [`RW-1:0] c0_dbg_r0, c0_dbg_pc,
    input [`RW-1:0] c0_o_mem_addr, c0_o_mem_data,
    output [`RW-1:0] c0_i_mem_data,
    input c0_o_mem_req, c0_o_mem_we,
    output c0_i_mem_ack,
    input [`ADDR_BYTES-1:0] c0_o_mem_sel,
    output c0_i_irq,
    input c0_o_c_instr_page, c0_o_c_data_page,
    input [`RW-1:0] c0_sr_bus_addr, c0_sr_bus_data_o,
    input c0_sr_bus_we,
    input c0_o_icache_flush,
    output c0_i_mem_exception,
    output c0_i_mc_core_int,
    input c0_o_c_instr_long,
    input [7:0] c0_o_instr_long_addr,
    input c0_o_mem_long_mode,
    input [7:0] c0_o_mem_high_addr,
    output [`RW-1:0] c0_i_core_int_sreg,
    input [35:0] c0_dbg_out,
    output [3:0] c0_dbg_in,

    // CORE 1
    output c1_clk,
    output c1_rst,
    output c1_disable,
    input [`RW-1:0] c1_o_req_addr,
    input c1_o_req_active, c1_o_req_ppl_submit,
    output [`I_SIZE-1:0] c1_i_req_data,
    output c1_i_req_data_valid,
    input [`RW-1:0] c1_dbg_r0, c1_dbg_pc,
    input [`RW-1:0] c1_o_mem_addr, c1_o_mem_data,
    output [`RW-1:0] c1_i_mem_data,
    input c1_o_mem_req, c1_o_mem_we,
    output c1_i_mem_ack,
    input [`ADDR_BYTES-1:0] c1_o_mem_sel,
    output c1_i_irq,
    input c1_o_c_instr_page, c1_o_c_data_page,
    input [`RW-1:0] c1_sr_bus_addr, c1_sr_bus_data_o,
    input c1_sr_bus_we,
    input c1_o_icache_flush,
    output c1_i_mem_exception,
    output c1_i_mc_core_int,
    input c1_o_c_instr_long,
    input [7:0] c1_o_instr_long_addr,
    input c1_o_mem_long_mode,
    input [7:0] c1_o_mem_high_addr,
    output [`RW-1:0] c1_i_core_int_sreg,
    input [35:0] c1_dbg_out,
    output [3:0] c1_dbg_in,

    // ICACHE 0
    output ic0_clk,
    output ic0_rst,
    output ic0_mem_req,
    input ic0_mem_ack,
    output [`RW-1:0] ic0_mem_addr,
    input [`I_SIZE-1:0] ic0_mem_data,
    output ic0_mem_ppl_submit,
    output ic0_mem_cache_flush,
    input ic0_wb_cyc,
    input ic0_wb_stb,
    output [`RW-1:0] ic0_wb_i_dat,
    input [`RW-1:0]  ic0_wb_adr,
    input ic0_wb_we,
    output ic0_wb_ack,
    input [1:0] ic0_wb_sel,
    output ic0_wb_err,

    // ICACHE 1
    output ic1_clk,
    output ic1_rst,
    output ic1_mem_req,
    input ic1_mem_ack,
    output [`RW-1:0] ic1_mem_addr,
    input [`I_SIZE-1:0] ic1_mem_data,
    output ic1_mem_ppl_submit,
    output ic1_mem_cache_flush,
    input ic1_wb_cyc,
    input ic1_wb_stb,
    output [`RW-1:0] ic1_wb_i_dat,
    input [`RW-1:0]  ic1_wb_adr,
    input ic1_wb_we,
    output ic1_wb_ack,
    input [1:0] ic1_wb_sel,
    output ic1_wb_err,

    // DCACHE
    output dcache_clk,
    output dcache_rst,
    output dcache_mem_req,
    output dcache_mem_we,
    input dcache_mem_ack,
    output [`WB_ADDR_W-1:0] dcache_mem_addr,
    output [`RW-1:0] dcache_mem_i_data,
    input [`RW-1:0] dcache_mem_o_data,
    output [1:0] dcache_mem_sel,
    output dcache_mem_cache_enable,
    input dcache_mem_exception,
    input dcache_wb_cyc,
    input dcache_wb_stb,
    output [`RW-1:0] dcache_wb_i_dat,
    input [`RW-1:0] dcache_wb_o_dat,
    input [`WB_ADDR_W-1:0]  dcache_wb_adr,
    input dcache_wb_we,
    input [1:0] dcache_wb_sel,
    input dcache_wb_4_burst,
    output dcache_wb_ack,
    output dcache_wb_err

);

////////////////////////////////////////////////////////
// INNER SECTION CORE > MMU > CACHES > CACHE_WB_MUXES //
////////////////////////////////////////////////////////

assign c0_clk = core_clock;
assign c0_rst = core_reset;
assign c1_clk = core_clock;
assign c1_rst = core_reset;

// CORE <-> ICACHE <-> IMMU
assign ic0_clk = core_clock;
assign ic0_rst = core_reset;
assign ic0_mem_req = c0_o_req_active;
assign ic0_mem_addr = c0_o_req_addr;
assign c0_i_req_data = ic0_mem_data;
assign c0_i_req_data_valid = ic0_mem_ack;
assign ic0_mem_ppl_submit = c0_o_req_ppl_submit;
assign ic0_mem_cache_flush = c0_o_icache_flush;

wire [`WB_ADDR_W-1:0] ic0_wb_adr_paged;
immu immu_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset), 
    .i_addr(ic0_wb_adr), .o_addr(ic0_wb_adr_paged),
    .i_sr_addr(c0_sr_bus_addr), .i_sr_data(c0_sr_bus_data_o), .i_sr_we(c0_sr_bus_we),
    .c_pag_en(c0_o_c_instr_page & ~inner_embed_mode),
    .c_long_mode(c0_o_c_instr_long),
    .i_long_high_addr(c0_o_instr_long_addr)
);

assign ic1_clk = core_clock;
assign ic1_rst = core_reset;
assign ic1_mem_req = c1_o_req_active;
assign ic1_mem_addr = c1_o_req_addr;
assign c1_i_req_data = ic1_mem_data;
assign c1_i_req_data_valid = ic1_mem_ack;
assign ic1_mem_ppl_submit = c1_o_req_ppl_submit;
assign ic1_mem_cache_flush = c1_o_icache_flush;

wire [`WB_ADDR_W-1:0] ic1_wb_adr_paged;
immu immu_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset), 
    .i_addr(ic1_wb_adr), .o_addr(ic1_wb_adr_paged),
    .i_sr_addr(c1_sr_bus_addr), .i_sr_data(c1_sr_bus_data_o), .i_sr_we(c1_sr_bus_we),
    .c_pag_en(c1_o_c_instr_page & ~inner_embed_mode),
    .c_long_mode(c1_o_c_instr_long),
    .i_long_high_addr(c1_o_instr_long_addr)
);



// ICACHE{0,1} ARBITER
wire icache_wb_cyc, icache_wb_stb;
wire icache_wb_we;
wire icache_wb_ack, icache_wb_err;
wire [`WB_ADDR_W-1:0]  icache_wb_adr;
wire [`WB_DATA_W-1:0] icache_wb_o_dat, icache_wb_i_dat;
wire [`WB_SEL_BITS-1:0] icache_wb_sel;
wire icache_wb_4_burst, icache_wb_8_burst;

assign ic0_wb_i_dat = icache_wb_i_dat;
assign ic1_wb_i_dat = icache_wb_i_dat; 

wishbone_arbiter icache_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(icache_wb_cyc),
    .owb_stb(icache_wb_stb),
    .owb_we(icache_wb_we),
    .owb_ack(icache_wb_ack),
    .owb_adr(icache_wb_adr),
    .owb_sel(icache_wb_sel),
    .owb_err(icache_wb_err),
    .owb_o_dat(icache_wb_o_dat),
    .owb_4_burst(icache_wb_4_burst),
    .owb_8_burst(icache_wb_8_burst),

    .i_wb0_cyc(ic0_wb_cyc),
    .wb0_stb(ic0_wb_stb),
    .wb0_we(ic0_wb_we),
    .wb0_ack(ic0_wb_ack),
    .wb0_adr(ic0_wb_adr_paged),
    .wb0_sel(ic0_wb_sel),
    .wb0_err(ic0_wb_err),
    .wb0_o_dat(`RW'b0),
    .wb0_4_burst(1'b0),
    .wb0_8_burst(1'b1),

    .i_wb1_cyc(ic1_wb_cyc),
    .wb1_stb(ic1_wb_stb),
    .wb1_we(ic1_wb_we),
    .wb1_ack(ic1_wb_ack),
    .wb1_adr(ic1_wb_adr_paged),
    .wb1_sel(ic1_wb_sel),
    .wb1_err(ic1_wb_err),
    .wb1_o_dat(`RW'b0),
    .wb1_4_burst(1'b0),
    .wb1_8_burst(1'b1)
);

// CORE -> DMMU -> DCACHE BUS ARBITER

wire [`WB_ADDR_W-1:0] c0_mem_addr_paged;
wire c0_mmu_data_cacheable;
dmmu dmmu0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),
    .i_addr(c0_o_mem_addr), .o_addr(c0_mem_addr_paged),
    .i_sr_addr(c0_sr_bus_addr), .i_sr_data(c0_sr_bus_data_o), .i_sr_we(c0_sr_bus_we),
    .c_pag_en(c0_o_c_data_page), .o_cacheable(c0_mmu_data_cacheable), .c_long(c0_o_mem_long_mode), .i_high_addr(c0_o_mem_high_addr)
);

wire [`WB_ADDR_W-1:0] c1_mem_addr_paged;
wire c1_mmu_data_cacheable;
dmmu dmmu1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),
    .i_addr(c1_o_mem_addr), .o_addr(c1_mem_addr_paged),
    .i_sr_addr(c1_sr_bus_addr), .i_sr_data(c1_sr_bus_data_o), .i_sr_we(c1_sr_bus_we),
    .c_pag_en(c1_o_c_data_page), .o_cacheable(c1_mmu_data_cacheable), .c_long(c1_o_mem_long_mode), .i_high_addr(c1_o_mem_high_addr)
);

mem_dcache_arb mem_dcache_arb (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .mem_req(dcache_mem_req), .mem_we(dcache_mem_we), .mem_ack(dcache_mem_ack), .mem_addr(dcache_mem_addr), .mem_i_data(dcache_mem_o_data), .mem_o_data(dcache_mem_i_data), .mem_sel(dcache_mem_sel), .mem_cache_enable(dcache_mem_cache_enable), .mem_exception(dcache_mem_exception),
    .mem0_req(c0_o_mem_req), .mem0_we(c0_o_mem_we), .mem0_ack(c0_i_mem_ack), .mem0_addr(c0_mem_addr_paged), .mem0_i_data(c0_i_mem_data), .mem0_o_data(c0_o_mem_data), .mem0_sel(c0_o_mem_sel), .mem0_cache_enable(c0_mmu_data_cacheable), .mem0_exception(c0_i_mem_exception),
    .mem1_req(c1_o_mem_req), .mem1_we(c1_o_mem_we), .mem1_ack(c1_i_mem_ack), .mem1_addr(c1_mem_addr_paged), .mem1_i_data(c1_i_mem_data), .mem1_o_data(c1_o_mem_data), .mem1_sel(c1_o_mem_sel), .mem1_cache_enable(c1_mmu_data_cacheable), .mem1_exception(c1_i_mem_exception)
);

assign dcache_clk = core_clock;
assign dcache_rst = core_reset; 

// {DCACHE, ICACHE} WB ARBITER

assign icache_wb_i_dat = inner_wb_i_dat;
assign dcache_wb_i_dat = inner_wb_i_dat;

wishbone_arbiter inner_wb_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(inner_wb_cyc),
    .owb_stb(inner_wb_stb),
    .owb_we(inner_wb_we),
    .owb_ack(inner_wb_ack),
    .owb_adr(inner_wb_adr),
    .owb_sel(inner_wb_sel),
    .owb_err(inner_wb_err),
    .owb_o_dat(inner_wb_o_dat),
    .owb_4_burst(inner_wb_4_burst),
    .owb_8_burst(inner_wb_8_burst),

    .i_wb0_cyc(icache_wb_cyc),
    .wb0_stb(icache_wb_stb),
    .wb0_we(icache_wb_we),
    .wb0_ack(icache_wb_ack),
    .wb0_adr(icache_wb_adr),
    .wb0_sel(icache_wb_sel),
    .wb0_err(icache_wb_err),
    .wb0_o_dat(icache_wb_o_dat),
    .wb0_4_burst(icache_wb_4_burst),
    .wb0_8_burst(icache_wb_8_burst),

    .i_wb1_cyc(dcache_wb_cyc),
    .wb1_stb(dcache_wb_stb),
    .wb1_we(dcache_wb_we),
    .wb1_ack(dcache_wb_ack),
    .wb1_adr(dcache_wb_adr),
    .wb1_sel(dcache_wb_sel),
    .wb1_err(dcache_wb_err),
    .wb1_o_dat(dcache_wb_o_dat),
    .wb1_4_burst(dcache_wb_4_burst),
    .wb1_8_burst(1'b0)
);

// INTERCORE CONTROL

assign c0_i_irq = inner_ext_irq;
assign c1_i_irq = 1'b0;
wire c0_sc_disable, c1_sc_disable;
assign c0_disable = c0_sc_disable | inner_disable;
assign c1_disable = c1_sc_disable | inner_disable;

intercore_sregs icore_sregs (
    .i_clk(core_clock),
    .i_rst(core_reset),

    .c0_sr_bus_addr(c0_sr_bus_addr),
    .c0_sr_bus_data_o(c0_sr_bus_data_o),
    .c0_sr_bus_data_i(c0_i_core_int_sreg),
    .c0_sr_bus_we(c0_sr_bus_we),

    .c1_sr_bus_addr(c1_sr_bus_addr),
    .c1_sr_bus_data_o(c1_sr_bus_data_o),
    .c1_sr_bus_data_i(c1_i_core_int_sreg),
    .c1_sr_bus_we(c1_sr_bus_we),

    .c0_disable(c0_sc_disable),
    .c0_core_int(c0_i_mc_core_int),
    .c1_disable(c1_sc_disable),
    .c1_core_int(c1_i_mc_core_int)
);
    
endmodule