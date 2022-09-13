`include "config.v"

module upper_core_logic (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk, i_rst,
    input [`RW-1:0] fetch_wb_adr, data_mem_addr,
    output [`WB_ADDR_W-1:0] data_mem_addr_paged, fetch_wb_adr_paged,
    input [`RW-1:0] sr_bus_addr, sr_bus_data_o,
    input sr_bus_we,
    input cc_instr_page, cc_data_page,
    output data_cacheable,

    output [`RW-1:0] fetch_wb_o_dat,
    output wb0_8_burst, wb1_4_burst, wb1_8_burst
);

immu i_mmu(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(fetch_wb_adr), .o_addr(fetch_wb_adr_paged), .i_sr_addr(sr_bus_addr), .i_sr_data(sr_bus_data_o), .i_sr_we(sr_bus_we), .c_pag_en(cc_instr_page));

dmmu d_mmu(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(data_mem_addr), .o_addr(data_mem_addr_paged), .i_sr_addr(sr_bus_addr), .i_sr_data(sr_bus_data_o), .i_sr_we(sr_bus_we), .c_pag_en(cc_data_page), .o_cacheable(data_cacheable));

assign fetch_wb_o_dat = `RW'b0;
assign wb0_8_burst = 1'b0;
assign wb1_4_burst = 1'b0;
assign wb1_8_burst = 1'b1;

endmodule