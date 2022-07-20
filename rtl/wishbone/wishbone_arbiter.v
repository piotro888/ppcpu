module wishbone_priority_arbiter (
    input i_clk,
    input i_rst,

    input i_wb0_cyc,
    input i_wb1_cyc,

    output o_wb_cyc,
    output reg o_sel_sig
);

wire bus_req = i_wb0_cyc | i_wb1_cyc;

assign o_wb_cyc = (o_sel_sig ? i_wb1_cyc : i_wb0_cyc) & ~i_rst;

always @(posedge i_clk) begin
    if(i_rst) begin
        o_sel_sig <= 1'b0;
    end else if(~o_wb_cyc & bus_req) begin
        o_sel_sig <= (i_wb0_cyc ? 1'b0 : 1'b1);
    end
end

endmodule
