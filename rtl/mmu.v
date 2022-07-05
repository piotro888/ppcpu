`include "config.v"

`define PAGE_TABLE_ADDR_W 8+4
`define PAGE_TABLE_ENT 16
`define PAGE_TABLE_OVER_ADDR_W 4
`define PAGE_TABLE_ADDR_DIFF 8

`define SR_START_ADDR `RW'h10

module mmu (
    input i_clk,
    input i_rst,

    input [`ADDR_W-1:0] i_addr,
    output [`EXT_ADDR_W-1:0] o_addr,

    input [`RW-1:0] i_sr_pt_sel,
    input [`RW-1:0] i_sr_data,
    input i_sr_pt_ie,
    input i_c_paging_enable
);

reg [`PAGE_TABLE_ADDR_W-1:0] ram_page_table [`PAGE_TABLE_ENT-1:0];
wire [`PAGE_TABLE_ADDR_W-1:0] pt_addr = 
    ram_page_table[i_addr[`ADDR_W-1:`ADDR_W-`PAGE_TABLE_OVER_ADDR_W]];

assign o_addr = (i_c_paging_enable
    ? {pt_addr, i_addr[`ADDR_W-`PAGE_TABLE_OVER_ADDR_W-1:0]}
    : {`PAGE_TABLE_ADDR_DIFF'b0, i_addr});


wire [`RW-1:0] sr_off_addr = i_sr_pt_sel-`SR_START_ADDR;
always @(posedge i_clk) begin
    if (i_sr_pt_ie && i_sr_pt_sel >= `SR_START_ADDR && i_sr_pt_sel < `SR_START_ADDR+`PAGE_TABLE_ENT)
        ram_page_table[sr_off_addr[`PAGE_TABLE_OVER_ADDR_W-1:0]] <= i_sr_data[`PAGE_TABLE_ADDR_W-1:0];
end


endmodule