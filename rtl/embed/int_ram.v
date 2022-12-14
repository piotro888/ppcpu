`include "config.v"

// Internal ram for use in embed mode (programmable from outside)
// It maps to address 0x800000 and 0x100000 noncacheable (the same segment, p/m split left to the user)
// In other case maps to 0x7ffe00 cacheable as internal fast memory

module int_ram (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,

    input [8:0] i_addr, // 1kB
    input [`RW-1:0] i_data,
    output reg [`RW-1:0] o_data,
    input i_we
);

reg [`RW-1:0] mem [511:0];

always @(posedge i_clk) begin
    if(i_we)
        mem[i_addr] <= i_data;
    o_data <= mem[i_addr];
end

endmodule