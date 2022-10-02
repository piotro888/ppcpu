`define MAX_DIV 16
`define MAX_DIV_LOG 4

module clk_div (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    input i_rst,

    output o_clk,

    input [`MAX_DIV_LOG-1:0] div,
    input div_we,
    input clock_sel
);

// ADD 130 buff

reg [`MAX_DIV-1:0] cnt;
reg [`MAX_DIV_LOG-1:0] curr_div, next_div_buff;
reg next_div_val;

reg res_clk;
assign o_clk = (clock_sel_r ? res_clk : i_clk);

always @(posedge i_clk) begin
    if (~cnt[curr_div]) begin
        cnt <= cnt + `MAX_DIV'b1;
    end else begin
        cnt <= `MAX_DIV'b0;
    end
end

always @(posedge i_clk) begin
    if (cnt[curr_div])
        res_clk <= ~res_clk;
end

always @(posedge i_clk) begin
    if(i_rst) begin
        curr_div <= `MAX_DIV_LOG'b110;
        next_div_val <= 1'b0;
    end else begin
        if(cnt[curr_div] & next_div_val) begin
            curr_div <= next_div_buff;
            next_div_val <= 1'b0;
        end
        if (div_we) begin
            next_div_buff <= div;
            next_div_val <= 1'b1;
        end
    end
end

reg clock_sel_r;
always @(posedge i_clk)
    clock_sel_r <= clock_sel;

endmodule