module reset_sync (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_rst,
    output o_rst,
    input i_clk
);

assign o_rst = reset_sync_ff[1];
reg [1:0] reset_sync_ff;
always @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        reset_sync_ff[0] <= 1'b1;
        reset_sync_ff[1] <= 1'b1;
    end else begin
        reset_sync_ff[0] <= 1'b0;
        reset_sync_ff[1] <= reset_sync_ff[0];
    end
end

endmodule