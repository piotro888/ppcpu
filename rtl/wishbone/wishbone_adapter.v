`include "../config.v"

`define WB_DATA_W 16 
`define WB_ADDR_W 24
`define WB_SEL_BITS 2

module wishbone_adapter (
    input i_clk,
    input i_rst,

    // Cpu interface
    input [`WB_ADDR_W-1:0] i_mem_addr, 
    input [`WB_DATA_W-1:0] i_mem_data,
    output [`WB_DATA_W-1:0] o_mem_data,
    input i_mem_req, i_mem_we,
    output o_mem_ack,
    input i_mem_next,

    // Wishbone interface
    output wb_cyc,
    output wb_stb,
    output [`WB_DATA_W-1:0] wb_o_dat,
    input [`WB_DATA_W-1:0] wb_i_dat,
    output [`WB_ADDR_W-1:0]  wb_adr,
    output wb_we,
    input wb_ack,
    input wb_err,
    input wb_rty,
    output [`WB_SEL_BITS-1:0] wb_sel
);

assign wb_sel = 2'b11;

assign wb_cyc = i_mem_req | wb_ack;
assign wb_stb = i_mem_req;
assign o_mem_ack = wb_ack;
assign o_mem_data = wb_i_dat;
assign wb_o_dat = i_mem_data;
assign wb_we = i_mem_we;

// same condition as for hold update, but without wb_ack, because we want to preserve old address on ack signal,
// and update it on next edge
wire new_cycle = ((~prev_wb_cyc & wb_cyc) | prev_term);
assign wb_adr =  new_cycle ? i_mem_addr : reg_hold_addr;

wire mem_cyc_term = wb_ack & ~i_mem_next;

reg prev_wb_cyc, prev_term;
always @(posedge i_clk) begin
    if(i_rst) begin
        prev_wb_cyc <= 1'b0;
        prev_term <= 1'b1;
    end else begin
        prev_wb_cyc <= i_mem_req;
        prev_term <= mem_cyc_term;
    end
end

reg [`WB_ADDR_W-1:0] reg_hold_addr;
always @(posedge i_clk) begin
    // update address for wishbone, only works usefull with i_mem_next
    // if mem_next is enabled, first address is shown only on first cycle of reqest,
    // then update to next address on ack
    if(new_cycle | (wb_ack & i_mem_next)) begin
        reg_hold_addr <= i_mem_addr;
    end
end

endmodule
