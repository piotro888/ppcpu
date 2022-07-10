// Memory and Writeback stage
`include "config.v"

module memwb (
    input i_clk,
    input i_rst,

    input [`RW-1:0] i_data,
    input [`RW-1:0] i_addr,
    input [`REGNO-1:0] i_reg_ie,
    input i_mem_access,
    input i_mem_we,

    output [`REGNO-1:0] o_reg_ie,
    output [`RW-1:0] o_reg_data,

    input i_submit,
    output o_ready,

    output reg o_mem_req,
    output [`RW-1:0] o_mem_data,
    output [`RW-1:0] o_mem_addr,
    output o_mem_we,
    input i_mem_ack,
    input [`RW-1:0] i_mem_data
);

assign o_reg_data = (i_mem_access ? i_mem_data : i_data);

assign o_mem_data = i_data;
assign o_mem_addr = i_addr;
assign o_mem_we = i_mem_we;

wire reg_ie = ((i_mem_access & i_mem_ack) | (~i_mem_access & i_submit)) & (|i_reg_ie);
assign o_reg_ie = (reg_ie ? i_reg_ie : `REGNO'b0);

assign o_ready = ~o_mem_req | (o_mem_req & i_mem_ack);

always @(posedge i_clk) begin
    if (i_rst) begin
        o_mem_req <= 1'b0;
    end else if (i_submit) begin
        o_mem_req <= i_mem_access;
    end else if (i_mem_ack) begin
        o_mem_req <= 1'b0;
    end
end
    
endmodule
