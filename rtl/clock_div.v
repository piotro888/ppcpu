`define MAX_DIV 16
`define MAX_DIV_LOG 4

module clock_div (
    input i_clk,
    input i_rst,

    output reg o_clk,

    input [`MAX_DIV_LOG-1:0] div,
    input div_we
);

// ADD 130 buff

reg [`MAX_DIV-1:0] cnt;
reg [`MAX_DIV_LOG-1:0] curr_div, next_div_buff;
reg next_div_val;

always @(posedge i_clk) begin
    if (~cnt[curr_div]) begin
        cnt <= cnt + `MAX_DIV'b1;
    end else begin
        cnt <= `MAX_DIV'b0;
    end
end

always @(posedge i_clk) begin
    if (cnt[curr_div])
        o_clk <= ~o_clk;
end

always @(posedge i_clk) begin
    if(i_rst) begin
        curr_div <= `MAX_DIV_LOG'b010;
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

endmodule