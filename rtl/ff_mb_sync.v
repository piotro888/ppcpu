module ff_mb_sync #(parameter DATA_W = 1) (
    input src_clk,
    input dst_clk,

    input [DATA_W-1:0] i_data,
    output [DATA_W-1:0] o_data,

    input i_xfer_req // transfer is possible only one time per 3 cycles
);

reg [DATA_W-1:0] s_data_ff;
always @(posedge src_clk) begin
    if (i_xfer_req)
        s_data_ff <= i_data;
end

reg s_xfer_xor_flag;
always @(posedge src_clk) begin // RESET
    if (i_xfer_req)
        s_xfer_xor_flag <= ~s_xfer_xor_flag;
end

reg [2:0] d_xfer_xor_sync;
always @(posedge dst_clk) begin
    d_xfer_xor_sync[0] <= s_xfer_xor_flag;
    d_xfer_xor_sync[1] <= d_xfer_xor_sync[0];
    d_xfer_xor_sync[2] <= d_xfer_xor_sync[1];
end

wire d_xfer_flag = d_xfer_xor_sync[1] ^ d_xfer_xor_sync[2];

reg [DATA_W-1:0] d_data;
always @(posedge dst_clk) begin
    if(d_xfer_flag)
        d_data <= s_data_ff;
end

assign o_data = d_data;


endmodule