`include "config.v"
module pc #(parameter INT_VEC = `RW'b1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    output reg [`RW-1:0] o_pc,
    input [`RW-1:0] i_bus,

    input i_c_pc_inc,
    input i_c_pc_ie,
    input i_c_pc_irq
);

always @(posedge i_clk) begin
    if (i_rst) begin
        o_pc <= `RW'b0;
    end else if (i_c_pc_irq) begin
        o_pc <= INT_VEC;
    end else if (i_c_pc_ie) begin
        o_pc <= i_bus;
    end else if (i_c_pc_inc) begin
        o_pc <= o_pc + `RW'b1;
    end
end

endmodule