/* Core with all memory access related modules */

`include "config.v"

`define WB_DATA_W 16 
`define WB_ADDR_W 24
`define WB_SEL_BITS 2

module upper_core (
    input wire i_clk,
    input wire i_rst,

    output wb_cyc,
    output reg wb_stb,
    output reg [`WB_DATA_W-1:0] wb_o_dat,
    input [`WB_DATA_W-1:0] wb_i_dat,
    output reg [`WB_ADDR_W-1:0]  wb_adr,
    output reg wb_we,
    input wb_ack,
    input wb_err,
    input wb_rty,
    output reg [`WB_SEL_BITS-1:0] wb_sel,

    input i_irq,
    output [`RW-1:0] dbg_r0, dbg_pc
);

// fetch input singals
wire [`RW-1:0] fetch_req_addr;
wire fetch_req_active;
wire [`I_SIZE-1:0] fetch_req_data;
wire fetch_req_ack;
wire fetch_ppl_submit;

// data memory connections
wire [`RW-1:0] data_mem_addr, data_o_mem_data, data_i_mem_data;
wire data_mem_req, data_mem_we, data_mem_ack;

core core (.i_clk(i_clk), .i_rst(i_rst), .o_req_addr(fetch_req_addr), .o_req_active(fetch_req_active), .i_req_data(fetch_req_data), .i_req_data_valid(fetch_req_ack),
    .o_mem_addr(data_mem_addr), .o_mem_data(data_o_mem_data), .i_mem_data(data_i_mem_data), .o_mem_req(data_mem_req), .o_mem_we(data_mem_we), .i_mem_ack(data_mem_ack),
    .dbg_r0(dbg_r0), .dbg_pc(dbg_pc), .i_irq(i_irq), .o_req_ppl_submit(fetch_ppl_submit));

wire fetch_wb_cyc, fetch_wb_stb, fetch_wb_we;
reg fetch_wb_ack, fetch_wb_err, fetch_wb_rty;
wire [`WB_DATA_W-1:0] fetch_wb_o_dat;
wire [`WB_ADDR_W-1:0]  fetch_wb_adr;
wire [`WB_SEL_BITS-1:0] fetch_wb_sel;

wire data_wb_cyc, data_wb_stb, data_wb_we;
reg data_wb_ack, data_wb_err, data_wb_rty;
wire [`WB_DATA_W-1:0] data_wb_o_dat;
wire [`WB_ADDR_W-1:0]  data_wb_adr;
wire [`WB_SEL_BITS-1:0] data_wb_sel;

wishbone_adapter data_wbm(.i_clk(i_clk), .i_rst(i_rst), .wb_cyc(data_wb_cyc), .wb_stb(data_wb_stb),
    .wb_we(data_wb_we), .wb_ack(data_wb_ack), .wb_err(data_wb_err), .wb_rty(data_wb_rty), .wb_i_dat(wb_i_dat),
    .wb_o_dat(data_wb_o_dat), .wb_adr(data_wb_adr), .wb_sel(data_wb_sel), .i_mem_addr({8'b0, data_mem_addr}), .i_mem_data(data_o_mem_data), 
    .o_mem_data(data_i_mem_data), .i_mem_req(data_mem_req), .i_mem_we(data_mem_we), .o_mem_ack(data_mem_ack), .i_mem_next(1'b0));

icache icache(.i_clk(i_clk), .i_rst(i_rst), .mem_req(fetch_req_active), .mem_addr(fetch_req_addr), .mem_data(fetch_req_data), .mem_ack(fetch_req_ack), .mem_ppl_submit(fetch_ppl_submit),
    .wb_cyc(fetch_wb_cyc), .wb_stb(fetch_wb_stb), .wb_we(fetch_wb_we), .wb_ack(fetch_wb_ack), .wb_i_dat(wb_i_dat), .wb_adr(fetch_wb_adr), .wb_sel(fetch_wb_sel));

wire arb_sel;
wishbone_priority_arbiter wb_arb(.i_clk(i_clk), .i_rst(i_rst), .i_wb0_cyc(data_wb_cyc), .i_wb1_cyc(fetch_wb_cyc), .o_wb_cyc(wb_cyc), .o_sel_sig(arb_sel));

always @(*) begin
    if(~arb_sel) begin
        wb_stb = data_wb_stb;
        wb_o_dat = data_wb_o_dat;
        wb_adr = data_wb_adr;
        wb_we = data_wb_we;
        wb_sel = data_wb_sel;
        data_wb_ack = wb_ack;
        data_wb_err = wb_err;
        data_wb_rty = wb_rty;
        fetch_wb_ack = 1'b0;
        fetch_wb_err = 1'b0;
        fetch_wb_rty = 1'b0;
    end else begin
        wb_stb = fetch_wb_stb;
        wb_o_dat = fetch_wb_o_dat;
        wb_adr = fetch_wb_adr;
        wb_we = fetch_wb_we;
        wb_sel = fetch_wb_sel;
        fetch_wb_ack = wb_ack;
        fetch_wb_err = wb_err;
        fetch_wb_rty = wb_rty;
        data_wb_ack = 1'b0;
        data_wb_err = 1'b0;
        data_wb_rty = 1'b0;
    end
end

endmodule

`include "core.v"
`include "wishbone/wishbone_adapter.v"
`include "wishbone/wishbone_arbiter.v"
`include "icache.v"
