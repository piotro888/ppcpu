module dcache_ram (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,
    input i_rst,

    input [5:0] i_addr,
    input [81:0] i_data,
    output reg [81:0] o_data,
    input i_we
);

reg [81:0] mem [63:0];

always @(posedge i_clk) begin
    if (i_rst) begin
        for (integer row = 0; row < 64; row = row+1) begin
            mem[row][1:0] <= 2'b0;
        end
    end else begin
        if(i_we)
            mem[i_addr] <= i_data;
        o_data <= mem[i_addr];
    end
end

endmodule
