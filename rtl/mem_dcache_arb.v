`include "config.v"

module mem_dcache_arb (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    input i_rst,
    input disable_mem1,

    output reg  mem_req,
    output reg mem_we,
    input mem_ack,
    output reg [`WB_ADDR_W-1:0] mem_addr,
    output reg [`RW-1:0] mem_i_data,
    input [`RW-1:0] mem_o_data,
    output reg [1:0] mem_sel,
    output reg mem_cache_enable,
    input mem_exception,

    input mem0_req,
    input mem0_we,
    output mem0_ack,
    input [`WB_ADDR_W-1:0] mem0_addr,
    input [`RW-1:0] mem0_i_data,
    output reg [`RW-1:0] mem0_o_data,
    input [1:0] mem0_sel,
    input mem0_cache_enable,
    output mem0_exception,

    input mem1_req,
    input mem1_we,
    output mem1_ack,
    input [`WB_ADDR_W-1:0] mem1_addr,
    input [`RW-1:0] mem1_i_data,
    output reg [`RW-1:0] mem1_o_data,
    input [1:0] mem1_sel,
    input mem1_cache_enable,
    output mem1_exception
);
    
wire request_term = mem_ack | mem_exception;

reg select, transfer_active;
reg req0_pending, req1_pending; // TODO: assign out req

wire req_w = mem0_req | mem1_req | req0_pending | req1_pending;

wire req_start = ~transfer_active & req_w;

assign mem_req = req_start;

always @(posedge i_clk) begin
    if (i_rst) begin
        select <= 1'b0;
        transfer_active <= 1'b0;
    end else if (req_start) begin
        select <= req_sel;
        transfer_active <= 1'b1;
    end else if (transfer_active & request_term) begin
        transfer_active <= 1'b0;
    end
end

always @(posedge i_clk) begin
    if (i_rst) begin
        req0_pending <= 1'b0;
        req1_pending <= 1'b0;
    end else if (req_start & ~req_sel) begin
        req0_pending <= 1'b0;
        req1_pending <= req1_pending | mem1_req;
    end else if (req_start & req_sel) begin
        req1_pending <= 1'b0;
        req0_pending <= req0_pending | mem0_req;
    end else if (transfer_active) begin
        req0_pending <= req0_pending | mem0_req;
        req1_pending <= req1_pending | mem1_req;
    end
end

reg req_sel;
always @* begin // round robin
    if (disable_mem1)
        req_sel = 1'b0;
    else if (select == 1'b0 && (mem1_req | req1_pending))
        req_sel = 1'b1;
    else if (select == 1'b1 && (mem0_req | req0_pending))
        req_sel = 1'b0;
    else if (mem0_req | req0_pending)
        req_sel = 1'b0;
    else if (mem1_req | req1_pending)
        req_sel = 1'b1;
    else
        req_sel = 1'b0;
end

wire select_wire = (req_start ? req_sel : select);

always @(*) begin
    {mem0_ack, mem0_o_data, mem0_exception, mem1_ack, mem1_o_data, mem1_exception} = 'b0;
    if(~select_wire) begin
        mem_we = mem0_we;
        mem_addr = mem0_addr;
        mem_i_data = mem0_i_data;
        mem_sel = mem0_sel;
        mem_cache_enable = mem0_cache_enable;
        mem0_ack = mem_ack;
        mem0_o_data = mem_o_data;
        mem0_exception = mem_exception;
    end else begin
        mem_we = mem1_we;
        mem_addr = mem1_addr;
        mem_i_data = mem1_i_data;
        mem_sel = mem1_sel;
        mem_cache_enable = mem1_cache_enable;
        mem1_ack = mem_ack;
        mem1_o_data = mem_o_data;
        mem1_exception = mem_exception;
    end
end

endmodule