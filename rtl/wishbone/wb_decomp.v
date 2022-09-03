`include "config.v"

module wb_decomp (
    input i_clk,
    input i_rst,

    output reg wb_cyc,
    output reg wb_stb,
    output reg [`WB_ADDR_W-1:0] wb_adr,
    output reg [`RW-1:0] wb_o_dat,
    input [`RW-1:0] wb_i_dat,
    output reg wb_we,
    output reg [1:0] wb_sel,
    input wb_ack,
    input wb_err,

    inout [`RW-1:0] cw_io,
    input cw_req,
    input cw_dir,
    output cw_ack,
    output cw_err
);
reg [`RW-1:0] cw_io_o;
assign cw_io = (~cw_dir ? `RW'bzzzzzzzzzzzzzzzz : cw_io_o);
wire [`RW-1:0] cw_io_i = cw_io;

`define SW 4
`define S_IDLE `SW'b0
`define S_HDR_1 `SW'b1
`define S_DATA_R `SW'b10
`define S_DATA_W `SW'b11
`define S_DATA_W_WD `SW'b100
reg [`SW-1:0] state;

reg [`WB_ADDR_W-1:0] l_wb_adr;
assign wb_adr = l_wb_adr + {20'b0, burst_cnt};

`define MAX_BRST_LOG 3
reg [`MAX_BRST_LOG-1:0] burst_end, burst_cnt;

`define DATA_WRITE_DELAY 3
`define DATA_WDEL_LOG 2
reg [`DATA_WDEL_LOG-1:0] wdel_cnt;

reg wr_1wait;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
        cw_ack <= 1'b0;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        wdel_cnt <= `DATA_WDEL_LOG'b0;
    end else begin
        case (state) 
            default: begin
                cw_ack <= 1'b0;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                if (cw_req & cw_io_i[0]) begin
                    state <= `S_HDR_1;
                    l_wb_adr[`WB_ADDR_W-1:`RW] <= cw_io_i[`RW-1:8];
                    wb_we <= cw_io_i[3];
                    wb_sel <= cw_io_i[1:0];
                    burst_end <= (~(|cw_io[7:4]) ? 3'd0 : (cw_io[4] ? 3'd7 : 3'd3));
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    cw_ack <= 1'b1;
                end
            end
            `S_HDR_1: begin
                state <= (wb_we ? `S_DATA_W : `S_DATA_R);
                l_wb_adr[`RW-1:0] <= cw_io_i;
                cw_ack <= 1'b0;
                if (~wb_we) begin
                    wb_cyc <= 1'b1;
                    wb_stb <= 1'b1;
                end
                wr_1wait <= 1'b1;
            end
            `S_DATA_R: begin
                cw_ack <= 1'b0;
                if (wb_ack && burst_cnt != burst_end) begin
                    cw_ack <= 1'b1;
                    burst_cnt <= burst_cnt + `MAX_BRST_LOG'b1;
                    cw_io_o <= wb_i_dat;
                end else if (wb_ack && burst_cnt == burst_end) begin
                    cw_ack <= 1'b1;
                    state <= `S_IDLE;
                    cw_io_o <= wb_i_dat;
                end
            end
            `S_DATA_W: begin
                if(wr_1wait) begin
                    wb_cyc <= 1'b1;
                    wb_stb <= 1'b1;
                    wr_1wait <= 1'b0;
                    wb_o_dat <= cw_io_i;
                end

                cw_ack <= 1'b0;
                if (wb_ack && burst_cnt != burst_end) begin
                    cw_ack <= 1'b1;
                    wb_stb <= 1'b0;
                    state <= `S_DATA_W_WD;
                    burst_cnt <= burst_cnt + `MAX_BRST_LOG'b1;
                end else if (wb_ack && burst_cnt == burst_end) begin
                    cw_ack <= 1'b1;
                    state <= `S_IDLE;
                end
            end
            `S_DATA_W_WD: begin
                cw_ack <= 1'b0;
                if(wdel_cnt == `DATA_WRITE_DELAY) begin
                    state <= `S_DATA_W;
                    wdel_cnt <= `DATA_WDEL_LOG'b0;
                    wb_stb <= 1'b1;
                    wb_o_dat <= cw_io;
                end else begin
                    wdel_cnt <= wdel_cnt + `DATA_WDEL_LOG'b1;
                end
            end

        endcase
    end
end

endmodule