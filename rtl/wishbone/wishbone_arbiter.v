`include "config.v"

module wishbone_priority_arbiter (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input i_wb0_cyc,
    input i_wb1_cyc,

    output o_wb_cyc,
    output reg o_sel_sig,

    input wb0_stb,
    input [`WB_ADDR_W-1:0] wb0_adr,
    input [`RW-1:0] wb0_o_dat,
    input wb0_we,
    input [1:0] wb0_sel,
    input wb0_8_burst, wb0_4_burst,
    output wb0_ack,
    output wb0_err,

    input wb1_stb,
    input [`WB_ADDR_W-1:0] wb1_adr,
    input [`RW-1:0] wb1_o_dat,
    input wb1_we,
    input [1:0] wb1_sel,
    input wb1_8_burst, wb1_4_burst,
    output wb1_ack,
    output wb1_err,

    output owb_stb,
    output [`WB_ADDR_W-1:0] owb_adr,
    output [`RW-1:0] owb_o_dat,
    output owb_we,
    output [1:0] owb_sel,
    input owb_ack,
    input owb_err,
    output owb_8_burst, owb_4_burst
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

always @(*) begin
    if(~o_sel_sig) begin
        owb_stb =  wb0_stb;
        owb_o_dat =  wb0_o_dat;
        owb_adr =  wb0_adr;
        owb_we =  wb0_we;
        owb_sel =  wb0_sel;
        wb0_ack = owb_ack;
        wb0_err = owb_err;
        wb1_ack = 1'b0;
        wb1_err = 1'b0;
        owb_4_burst =  wb0_4_burst;
        owb_8_burst = wb0_8_burst;
    end else begin
        owb_stb =  wb1_stb;
        owb_o_dat =  wb1_o_dat;
        owb_adr =  wb1_adr;
        owb_we =  wb1_we;
        owb_sel =  wb1_sel;
        wb1_ack = owb_ack;
        wb1_err = owb_err;
        wb0_ack = 1'b0;
        wb0_err = 1'b0;
        owb_4_burst =  wb1_4_burst;
        owb_8_burst =  wb1_8_burst;
    end
end

endmodule
