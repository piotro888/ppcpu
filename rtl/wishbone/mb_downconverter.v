`include "config.v"

module mb_downconverter (
    // 32b data width -> 16 data width 
    input i_clk,
    input i_rst,

    input [`RW-1:0] u_req_addr,
    input u_req_active,
    output [`I_SIZE-1:0] u_req_data,
    output u_req_data_valid,

    output [`RW-1:0] d_req_addr,
    output reg d_req_active,
    input [`RW-1:0] d_req_data,
    input d_req_data_valid,
    output reg d_req_next
);

wire [`RW-1:0] first_addr = {u_req_addr[`RW-2:0], 1'b0};
wire [`RW-1:0] second_addr = {u_req_addr[`RW-2:0], 1'b1};

assign d_req_active = u_req_active & ~u_req_data_valid;
assign d_req_next = ~lower_valid & prev_d_req_active;
assign d_req_addr = (~prev_d_req_active ? first_addr : second_addr);

assign u_req_data_valid = data_valid;
assign u_req_data = {d_req_data, lower_part};

reg prev_d_req_active;
reg lower_valid, data_valid;
reg [`RW-1:0] lower_part;

always @(posedge i_clk) begin
    if (i_rst) begin
        lower_valid <= 1'b0;
        data_valid <= 1'b0;
    end else if (d_req_data_valid & ~lower_valid) begin      
        lower_valid <= 1'b1;
        lower_part <= d_req_data;
    end else if (d_req_data_valid & lower_valid) begin
        lower_valid <= 1'b0;
        data_valid <= 1'b1;
    end else begin
        data_valid <= 1'b0;
    end
    prev_d_req_active <= d_req_active;
end

endmodule