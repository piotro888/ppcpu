module timer (
    input i_clk,
    input i_rst,
    output reg irq,

    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output [15:0] wb_o_dat
);

reg [15:0] timer_cnt;

reg [15:0] pre_div_cnt;
reg [3:0] clk_div;

reg [15:0] reset_val;

always @(posedge i_clk) begin
    if(i_rst) begin
        timer_cnt <= 16'b0;
        pre_div_cnt <= 16'b0;
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b0)) begin
        timer_cnt <= wb_i_dat;
    end else begin
        pre_div_cnt <= pre_div_cnt + 16'b1;
        
        if(pre_div_cnt >= (1<<clk_div) - 16'b1) begin
            timer_cnt <= timer_cnt + 16'b1;
            pre_div_cnt <= 16'b0;

            if(timer_cnt == 16'hffff)
                timer_cnt <= reset_val;
        end
    end
end

always @(posedge i_clk) begin
    irq <= (timer_cnt == 16'hffff);
end

always @(posedge i_clk) begin
    if(i_rst) begin
        reset_val <= 16'b0;
        clk_div <= 4'b0;
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b1)) begin
        clk_div <= wb_i_dat[3:0];
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b10)) begin
        reset_val <= wb_i_dat;
    end
end

assign wb_o_dat = timer_cnt;
assign wb_ack = (wb_cyc & wb_stb);

endmodule
