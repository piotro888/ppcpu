module icache_ram (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,

    input [4:0] i_addr,
    input [137:0] i_data,
    output reg [137:0] o_data,
    input i_we
);

reg [137:0] mem [31:0];

always @(posedge i_clk) begin
    if(i_we)
        mem[i_addr] <= i_data;
    o_data <= mem[i_addr];
end

endmodule
