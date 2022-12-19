`include "config.v"

`define IN_ADDR_W 16
`define OUT_ADDR_W 24

module dmmu (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`IN_ADDR_W-1:0] i_addr,
    input [7:0] i_high_addr,
    output [`OUT_ADDR_W-1:0] o_addr,
    output o_cacheable,

    input [`RW-1:0] i_sr_addr,
    input [`RW-1:0] i_sr_data,
    input i_sr_we,

    input c_pag_en,
    input c_long
);

`define OFF_W 11
`define PAGE_IDX_W 4
`define PAGE_ENTRIES 16
`define PAGE_RES_SIZE 13
`define SR_ADDR_OFF `RW'h200

wire [`OFF_W-1:0] page_off = i_addr[`OFF_W-1:0];
wire [`PAGE_IDX_W-1:0] page_idx = i_addr[`OFF_W+`PAGE_IDX_W-1:`OFF_W];

reg [`PAGE_RES_SIZE-1:0] page_table [`PAGE_ENTRIES-1:0];

wire [`PAGE_RES_SIZE-1:0] page_res = {page_table[page_idx][`PAGE_RES_SIZE-1:0]};

wire [`RW-1:0] sr_addr_idx = i_sr_addr-`SR_ADDR_OFF;
wire addr_in_range = (i_sr_addr >= `SR_ADDR_OFF) && (i_sr_addr < `SR_ADDR_OFF+16);
reg [7:0] long_off_reg;

always @(posedge i_clk) begin
    if (i_rst) begin
        page_table[0] <= `PAGE_RES_SIZE'b0;
        page_table[1] <= `PAGE_RES_SIZE'b0;
        page_table[2] <= `PAGE_RES_SIZE'b0;
        page_table[3] <= `PAGE_RES_SIZE'b0;
        page_table[4] <= `PAGE_RES_SIZE'b0;
        page_table[5] <= `PAGE_RES_SIZE'b0;
        page_table[6] <= `PAGE_RES_SIZE'b0;
        page_table[7] <= `PAGE_RES_SIZE'b0;
        page_table[8] <= `PAGE_RES_SIZE'b0;
        page_table[9] <= `PAGE_RES_SIZE'b0;
        page_table[10] <= `PAGE_RES_SIZE'b0;
        page_table[11] <= `PAGE_RES_SIZE'b0;
        page_table[12] <= `PAGE_RES_SIZE'b0;
        page_table[13] <= `PAGE_RES_SIZE'b0;
        page_table[14] <= `PAGE_RES_SIZE'b0;
        page_table[15] <= `PAGE_RES_SIZE'b0;
    end else if (i_sr_we & addr_in_range)
        page_table[sr_addr_idx[`PAGE_IDX_W-1:0]] <= i_sr_data[`PAGE_RES_SIZE-1:0];
    else if (i_sr_we & i_sr_addr == `SR_ADDR_OFF+16)
        long_off_reg <= i_sr_data[7:0];
end

`define PAGE_DEFAULT_PREFIX 8'h10
wire [`OUT_ADDR_W-1:0] page_disable_address = {`PAGE_DEFAULT_PREFIX, i_addr[`IN_ADDR_W-1:0]};
wire [`OUT_ADDR_W-1:0] page_enable_address = {page_res, page_off};
wire [`OUT_ADDR_W-1:0] long_mode_address = (i_high_addr == 8'b0 && ~i_addr[15] ? page_enable_address : {i_high_addr[7:0]+long_off_reg, i_addr});

assign o_addr = c_long ? long_mode_address : (c_pag_en ? page_enable_address : page_disable_address);
assign o_cacheable = (o_addr >= `OUT_ADDR_W'h100000 && o_addr < `OUT_ADDR_W'h800000);

endmodule