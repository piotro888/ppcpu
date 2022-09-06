`include "config.v"

module wb_compressor (
    input i_clk,
    input i_rst,

    input wb_cyc,
    input wb_stb,
    input [`WB_ADDR_W-1:0] wb_adr,
    input [`RW-1:0] wb_o_dat,
    output reg [`RW-1:0] wb_i_dat,
    input wb_we,
    input [1:0] wb_sel,
    input wb_8_burst, wb_4_burst,
    output wb_ack,
    output wb_err,

    inout reg [`RW-1:0] cw_io,
    output cw_req,
    output cw_dir,
    input cw_ack,
    input cw_err
);

reg [`RW-1:0] cw_io_o;
assign cw_io = (cw_dir ? `RW'bzzzzzzzzzzzzzzzz : cw_io_o);
wire [`RW-1:0] cw_io_i = cw_io;

`define SW 4
`define S_IDLE `SW'b0
`define S_HDR_1 `SW'b1
`define S_WACK `SW'b10
`define S_DATA_R `SW'b11
`define S_DATA_W `SW'b101
`define S_DATA_WT `SW'b111
`define S_WC `SW'b100
reg [`SW-1:0] state;

wire [3:0] cyc_type = (wb_8_burst ? 4'b001 : (wb_4_burst ? 4'b010 : 4'b000));
wire [`RW-1:0] header_0 = {wb_adr[`WB_ADDR_W-1:`RW], cyc_type, wb_we, wb_sel, 1'b1}; 

reg [`WB_ADDR_W-1:0] l_wb_adr;
wire [`WB_ADDR_W-1:0] wb_exp_adr = l_wb_adr + {20'b0, burst_cnt};
reg [`WB_ADDR_W-1:0] ack_exp_adr;

reg l_we;

`define MAX_BRST_LOG 3
reg [`MAX_BRST_LOG-1:0] burst_end, burst_cnt;

wire xfer_ack = ((cw_ack | cw_err) && wb_cyc && wb_stb);
wire ignored_addr = wb_adr < `WB_ADDR_W'h002000;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
        wb_ack <= 1'b0;
        cw_req <= 1'b0;
    end else begin
        case (state) 
            default: begin
                if (wb_cyc & wb_stb & ~ignored_addr) begin
                    state <= `S_HDR_1;
                    cw_io_o <= header_0;
                    cw_dir <= 1'b0;
                    cw_req <= 1'b1;
                    l_we <= wb_we;
                    burst_end <= (wb_8_burst ? 3'd7 : (wb_4_burst ? 3'd3 : 3'b0));

                    burst_cnt <= `MAX_BRST_LOG'b0;
                end
            end
            `S_HDR_1: begin
                state <= `S_WACK;
                cw_req <= 1'b0;
                cw_io_o <= wb_adr[`RW-1:0];
            end
            `S_WACK: begin
                if (cw_ack)
                    state <= (l_we ? `S_DATA_W : `S_DATA_R);
                cw_io_o <= wb_o_dat;
            end
            `S_DATA_R: begin
                cw_dir <= 1'b1;
                cw_io_o <= wb_o_dat;
                
                wb_ack <= 1'b0;
                wb_err <= 1'b0;

                if (xfer_ack && burst_cnt != burst_end) begin
                    burst_cnt <= burst_cnt + 1'b1;
                    wb_ack <= cw_ack; // It is good idea to break combinational path at end of IC
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    ack_exp_adr <= wb_exp_adr;
                end else if (xfer_ack && burst_cnt == burst_end) begin
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    state <= `S_WC;
                    cw_dir <= ~cw_dir;
                    ack_exp_adr <= wb_exp_adr;
                end
            end
            `S_DATA_W: begin
                cw_dir <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                cw_req <= 1'b0;

                if (xfer_ack && burst_cnt != burst_end) begin
                    burst_cnt <= burst_cnt + 1'b1;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    ack_exp_adr <= wb_exp_adr;
                    state <= `S_DATA_WT;
                end else if (xfer_ack && burst_cnt == burst_end) begin
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    state <= `S_WC;
                    cw_dir <= ~cw_dir;
                    ack_exp_adr <= wb_exp_adr;
                end
            end
            `S_DATA_WT: begin
                cw_req <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                // wait for new data and send sync signal
                // continous data delivery with clock sychronizers is not possible, due to ACK delay
                // it causes no problem with read burst, beacause wishbone communication is unidirectional then
                if (wb_cyc & wb_stb & ~wb_ack) begin // wait for stb reassert when new burst data is delivered
                    cw_req <= 1'b1;
                    cw_io_o <= wb_o_dat;
                    state <= `S_DATA_W;
                end
            end
            `S_WC: begin
                cw_dir <= 1'b0;
                cw_req <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                state <= `S_IDLE;
            end
        endcase
    end
end

`undef S_DATA_R
`undef S_DATA_W

endmodule
