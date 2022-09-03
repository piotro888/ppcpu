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
`define S_DATA `SW'b11
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

wire xfer_ack = (cw_ack && wb_cyc && wb_stb);

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
        wb_ack <= 1'b0;
        cw_req <= 1'b0;
    end else begin
        case (state) 
            default: begin
                if (wb_cyc & wb_stb) begin
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
                    state <= `S_DATA;
                cw_io_o <= wb_o_dat;
            end
            `S_DATA: begin
                cw_dir <= ~l_we;
                cw_io_o <= wb_o_dat;

                // if (wb_ack && wb_adr != ack_exp_adr) 
                //     wb_err <= 1'b1;
                
                wb_ack <= 1'b0;

                if (xfer_ack && burst_cnt != burst_end) begin
                    burst_cnt <= burst_cnt + 1'b1;
                    wb_ack <= 1'b1; // It is good idea to break combinational path at end of IC
                    wb_i_dat <= cw_io_i;
                    ack_exp_adr <= wb_exp_adr;
                end else if (xfer_ack && burst_cnt == burst_end) begin
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    wb_ack <= 1'b1;
                    wb_i_dat <= cw_io_i;
                    state <= `S_WC;
                    cw_dir <= ~cw_dir;
                    ack_exp_adr <= wb_exp_adr;
                end

                // TODO: HANDLE ERR (and emit err if wbadr diff from exp when ack enabled)
            end
            `S_WC: begin
                cw_dir <= 1'b0;
                wb_ack <= 1'b0;
                state <= `S_IDLE;
            end
        endcase
    end
end


endmodule
/*

`define MAX_DIV 16
`define MAX_DIV_LOG 4

module clock_div (
    input i_clk,
    input i_rst,

    output reg o_clk,

    input [`MAX_DIV_LOG-1:0] div,
    input div_we
);


reg [`MAX_DIV-1:0] cnter;
reg [`MAX_DIV_LOG-1:0] curr_div;

always @(posedge i_clk) begin
    if (i_rst) begin
        cnter <= `MAX_DIV'b0;
    end else if (~cnter[curr_div]) begin
        cnter <= cnter + `MAX_DIV'b1;
    end else begin
        cnter <= `MAX_DIV'b0;
    end
end

always @(posedge i_clk) begin
    if(i_rst)
        o_clk <= 1'b0;
    else begin
        if (cnter[curr_div])
            o_clk <= ~o_clk;
    end
end

always @(posedge i_clk) begin
    if(i_rst)
        curr_div <= `MAX_DIV_LOG'b1111;
    else if (div_we)
        curr_div <= div;
end

endmodule

*/