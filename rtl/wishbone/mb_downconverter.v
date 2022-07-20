`include "config.v"

module mb_downconverter (
    // 32b data width -> 16 data width 
    input i_clk,
    input i_rst,

    input [`RW-1:0] u_req_addr,
    input u_req_active,
    output [`I_SIZE-1:0] u_req_data,
    output reg u_req_data_valid,

    output [`RW-1:0] d_req_addr,
    output reg d_req_active,
    input [`RW-1:0] d_req_data,
    input d_req_data_valid,
    output reg d_req_next
);

wire [`RW-1:0] first_addr = {u_req_addr[`RW-2:0], 1'b0};
wire [`RW-1:0] second_addr = {u_req_addr[`RW-2:0], 1'b1};

assign d_req_active = u_req_active & ~u_req_data_valid & ~(lower_valid & d_req_data_valid);
assign d_req_next = d_req_active & ~prev_done & ~lower_valid;
assign d_req_addr = (d_req_next | lower_valid) ? second_addr : first_addr;

assign u_req_data = {d_req_data, lower_part};

reg prev_done, req_reg;
reg lower_valid;
reg [`RW-1:0] lower_part;

always @(posedge i_clk) begin
    if (i_rst) begin
        lower_valid <= 1'b0;
        prev_done <= 1'b1;
    end else if (d_req_data_valid & ~lower_valid) begin      
        lower_valid <= 1'b1;
        lower_part <= d_req_data;
        u_req_data_valid <= 1'b0;
    end else if (d_req_data_valid & lower_valid) begin
        lower_valid <= 1'b0;
        u_req_data_valid <= 1'b1;
    end else begin
        u_req_data_valid <= 1'b0;
    end
    req_reg <= ~u_req_active;
    prev_done <= req_reg | u_req_data_valid;
end

endmodule