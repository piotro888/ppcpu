`include "config.v"

module alu_mul_div (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`RW-1:0] i_a, i_b,
    output [`RW-1:0] o_d,

    input i_mul,
    input i_div,
    input i_mod,

    input i_submit,
    input i_flush,
    output o_busy
);

`define RWLOG 4

reg [`RWLOG-1:0] cbit;
reg comp;

always @(posedge i_clk) begin
    if (i_rst | i_flush) begin
        comp <= 1'b0;
    end else if (i_submit) begin
        cbit <= (i_mul ? `RWLOG'b1 : `RWLOG'b0);
        comp <= 1'b1;
    end else if (comp) begin
        cbit <= cbit+1'b1;
        if (cbit == `RWLOG'd15) begin
            comp <= 1'b0;
        end
    end
end

reg [`RW-1:0] mul_res;

always @(posedge i_clk) begin
    if (i_submit & i_mul) begin
        mul_res <= (i_b[0] ? i_a : `RW'b0);
    end else if (comp & i_b[cbit] & i_mul) begin
        mul_res <= mul_res + (i_a<<cbit);
    end
end

reg [`RW-1:0] div_res, div_cur;
wire [`RW-1:0] div_diff = div_cur-i_b;

always @(posedge i_clk) begin
    if (i_submit & (i_div | i_mod)) begin
        div_res <= `RW'b0;
        div_cur <= {{`RW-1{1'b0}}, i_a[`RW-1]};
    end else if (comp & (i_div | i_mod)) begin
        if (div_cur >= i_b) begin
            div_res[`RW-cbit-1] <= 1'b1;
            if (cbit != `RWLOG'd15)
                div_cur <= {div_diff[14:0], i_a[`RW-cbit-2]};
            else
                div_cur <= div_diff;
        end else begin
            if (cbit != `RWLOG'd15)
                div_cur <= {div_cur[14:0], i_a[`RW-cbit-2]};
        end
    end
end

assign o_d = (i_mod ? div_cur : (i_div ? div_res : mul_res));
assign o_busy = (i_submit | comp);

endmodule