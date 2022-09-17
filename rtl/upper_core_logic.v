`include "config.v"

module upper_core_logic (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk, i_rst,
    input [`RW-1:0] fetch0_wb_adr, data0_mem_addr,
    output [`WB_ADDR_W-1:0] data0_mem_addr_paged, fetch0_wb_adr_paged,
    input [`RW-1:0] fetch1_wb_adr, data1_mem_addr,
    output [`WB_ADDR_W-1:0] data1_mem_addr_paged, fetch1_wb_adr_paged,

    input [`RW-1:0] sr0_bus_addr, sr0_bus_data_o,
    input [`RW-1:0] sr1_bus_addr, sr1_bus_data_o,
    output [`RW-1:0] sr0_bus_data_i, sr1_bus_data_i,
    input sr0_bus_we, sr1_bus_we,
    input cc0_instr_page, cc0_data_page,
    input cc1_instr_page, cc1_data_page,
    output data0_cacheable, data1_cacheable,

    output [`RW-1:0] fetch0_wb_o_dat, fetch1_wb_o_dat,
    output data_wb_8_burst, fetch0_4_burst, fetch0_8_burst, fetch1_4_burst, fetch1_8_burst,
    output c1_disable,

    output c0_disable,
    output c0_core_int, c1_core_int,

    input [35:0] c0_dbg, c1_dbg,
    input dbg_sel,
    output [35:0] dbg_out,

    input i_irq,
    output o_irq0, o_irq1
);

immu i_mmu_0(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(fetch0_wb_adr), .o_addr(fetch0_wb_adr_paged), .i_sr_addr(sr0_bus_addr), .i_sr_data(sr0_bus_data_o), .i_sr_we(sr0_bus_we), .c_pag_en(cc0_instr_page));

dmmu d_mmu_0(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(data0_mem_addr), .o_addr(data0_mem_addr_paged), .i_sr_addr(sr0_bus_addr), .i_sr_data(sr0_bus_data_o), .i_sr_we(sr0_bus_we), .c_pag_en(cc0_data_page), .o_cacheable(data0_cacheable));

immu i_mmu_1(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(fetch1_wb_adr), .o_addr(fetch1_wb_adr_paged), .i_sr_addr(sr1_bus_addr), .i_sr_data(sr1_bus_data_o), .i_sr_we(sr1_bus_we), .c_pag_en(cc1_instr_page));

dmmu d_mmu_1(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(data1_mem_addr), .o_addr(data1_mem_addr_paged), .i_sr_addr(sr1_bus_addr), .i_sr_data(sr1_bus_data_o), .i_sr_we(sr1_bus_we), .c_pag_en(cc1_data_page), .o_cacheable(data1_cacheable));

assign fetch0_wb_o_dat = `RW'b0;
assign fetch1_wb_o_dat = `RW'b0;
assign data_wb_8_burst = 1'b0;
assign fetch0_4_burst = 1'b0;
assign fetch0_8_burst = 1'b1;
assign fetch1_4_burst = 1'b0;
assign fetch1_8_burst = 1'b1;


// inter core interupt sregs
`define SREG_MCINT_SET `RW'b1001
`define SREG_MCINT_RESET `RW'b1010
`define SREG_MCDISABLE `RW'b1011
`define CORES 2
wire [`CORES-1:0] ic_irq_set_read_w;
assign ic_irq_set_read_w[0] = (sr0_bus_addr == `SREG_MCINT_SET) && sr0_bus_we;
assign ic_irq_set_read_w[1] = (sr1_bus_addr == `SREG_MCINT_SET) && sr1_bus_we;

wire [`CORES-1:0] ic_irq_reset_w;
assign ic_irq_reset_w[0] = (sr0_bus_addr == `SREG_MCINT_RESET) && sr0_bus_we;
assign ic_irq_reset_w[1] = (sr1_bus_addr == `SREG_MCINT_RESET) && sr1_bus_we;

wire [`RW-1:0] sr_bus_dat [`CORES-1:0];
assign sr_bus_dat[0] = sr0_bus_data_o;
assign sr_bus_dat[1] = sr1_bus_data_o;

assign sr0_bus_data_i = {14'b0, ic_irq_state};
assign sr1_bus_data_i = {14'b0, ic_irq_state};

reg [`CORES-1:0] ic_irq_state;

always @(posedge i_clk) begin
    if (i_rst) begin
        ic_irq_state = 2'b0;
    end else begin
        if (ic_irq_reset_w[0])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_reset_w[1])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[1][`CORES-1:0];
        
        if (ic_irq_set_read_w[0])
            ic_irq_state = ic_irq_state | sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_set_read_w[1])
            ic_irq_state = ic_irq_state | sr_bus_dat[1][`CORES-1:0];
    end
end

wire c0_core_hold_we = (sr0_bus_addr == `SREG_MCDISABLE) && sr0_bus_we;
assign c0_disable = 1'b0;
assign c0_core_int = ic_irq_state[0];
assign c1_core_int = ic_irq_state[1];

reg core1_disable;
assign c1_disable = core1_disable;
always @(posedge i_clk) begin
    if (i_rst) begin
        core1_disable <= 1'b1;
    end else begin
        if (c0_core_hold_we)
            core1_disable <= sr_bus_dat[0][1];
    end
end

assign o_irq0 = i_irq;
assign o_irq1 = 1'b0;

endmodule