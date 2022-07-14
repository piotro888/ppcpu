`include "config.v"

module rf (
    input i_clk,
    input i_rst,

    input i_gie,
    input [`REGNO-1:0] i_ie,
    input [`REGNO_LOG-1:0] i_lout_sel,
    input [`REGNO_LOG-1:0] i_rout_sel,
    
    input [`RW-1:0] i_d,
    output [`RW-1:0] o_lout,
    output [`RW-1:0] o_rout,

    output [`RW-1:0] dbg_r0
);

wire [`RW-1:0] reg_outputs [`REGNO-1:0];

assign o_lout = reg_outputs[i_lout_sel];
assign o_rout = reg_outputs[i_rout_sel];

assign dbg_r0 = reg_outputs[0];

genvar i;
generate
    for (i=0; i<`REGNO; i=i+1) begin : rf_regs
        register rf_reg(
            .i_clk(i_clk),
            .i_rst(i_rst),
            .i_d(i_d),
            .i_ie(i_ie[i] & i_gie),
            .o_d(reg_outputs[i])
        );
    end
endgenerate

endmodule

module register #(parameter N = `RW, parameter RESET_VAL = 0) (
    input i_clk,
    input i_rst,

    input [N-1:0] i_d,
    output reg [N-1:0] o_d,
    input i_ie 
);

always @(posedge i_clk) begin
    if (i_rst)
        o_d <= RESET_VAL;
    else if (i_ie)
        o_d <= i_d;
end

    
endmodule