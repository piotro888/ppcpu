// Memory and Writeback stage
`include "config.v"

module memwb (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`RW-1:0] i_data,
    input [`RW-1:0] i_addr,
    input [7:0] i_addr_high,
    input [`REGNO-1:0] i_reg_ie,
    input i_mem_access,
    input i_mem_long,
    input i_mem_we,
    input i_mem_width,

    output [`REGNO-1:0] o_reg_ie,
    output [`RW-1:0] o_reg_data,

    input i_submit,
    output o_ready,

    output reg o_mem_req,
    output [`RW-1:0] o_mem_data,
    output [`RW-1:0] o_mem_addr,
    output [7:0] o_mem_addr_high,
    output o_mem_long,
    output o_mem_we,
    input i_mem_ack,
    input [`RW-1:0] i_mem_data,
    output [`ADDR_BYTES-1:0] o_mem_sel,
    input o_mem_exception,

    output dbg_out
);

wire [`RW-1:0] mem_result = (i_mem_width ? (i_addr[0] ? (i_mem_data>>`RW'h8) : (i_mem_data&`RW'hff)) : i_mem_data);
assign o_reg_data = (i_mem_access ? mem_result : i_data);

assign o_mem_data = ((i_mem_width & i_addr[0]) ? (i_data<<`RW'h8) : i_data);
assign o_mem_addr = {(i_addr_high[0]&i_mem_long), 15'b0} | (i_addr>>`RW'b1); // LSB is used for byte selection in STDMEM
assign o_mem_we = i_mem_we;
assign o_mem_addr_high = (i_addr_high>>`RW'b1);
assign o_mem_long = i_mem_long;

assign o_mem_sel = (i_mem_width ? (`ADDR_BYTES'b1 << i_addr[0]) : `ADDR_BYTES'b11);

wire reg_ie = ((i_mem_access & i_mem_ack) | (~i_mem_access & i_submit)) & (|i_reg_ie);
assign o_reg_ie = (reg_ie ? i_reg_ie : `REGNO'b0);

assign o_ready = (~o_mem_req | (o_mem_req & i_mem_ack)) & ~(i_submit & i_mem_access);

always @(posedge i_clk) begin
    if (i_rst) begin
        o_mem_req <= 1'b0;
    end else if (i_submit) begin
        o_mem_req <= i_mem_access;
    end else if (i_mem_ack | o_mem_exception) begin
        o_mem_req <= 1'b0;
    end
end

assign dbg_out = o_ready;
    
endmodule
