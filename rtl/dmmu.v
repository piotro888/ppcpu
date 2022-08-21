`include "config.v"

`define IN_ADDR_W 16
`define OUT_ADDR_W 24

module dmmu (
    input i_clk,
    input i_rst,

    input [`IN_ADDR_W-1:0] i_addr,
    output [`OUT_ADDR_W-1:0] o_addr,

    wire [`RW-1:0] i_sr_addr,
    wire [`RW-1:0] i_sr_data,
    wire i_sr_we,

    wire c_pag_en
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
always @(posedge i_clk) begin
    if (i_rst) begin
        for (integer i=0; i<16; i=i+1) begin
            page_table[i] <= `PAGE_RES_SIZE'b0;
        end
    end else if (i_sr_we & addr_in_range)
        page_table[sr_addr_idx[`PAGE_IDX_W-1:0]] <= i_sr_data[`PAGE_RES_SIZE-1:0];
end

`define PAGE_DEFAULT_PREFIX 8'h10
wire [`OUT_ADDR_W-1:0] page_disable_address = {`PAGE_DEFAULT_PREFIX, i_addr[`IN_ADDR_W-1:0]};
wire [`OUT_ADDR_W-1:0] page_enable_address = {page_res, page_off};

assign o_addr = c_pag_en ? page_enable_address : page_disable_address;

endmodule