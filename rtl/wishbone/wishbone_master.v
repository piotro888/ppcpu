`include "config.v"

`define WB_DATA_W 16 
`define WB_ADDR_W 24
`define WB_SEL_BITS 2

module wishbone_master (
    input i_clk,
    input i_rst,

    // Cpu interface
    input [`WB_ADDR_W-1:0] i_mem_addr, 
    input [`RW-1:0] i_mem_data,
    output reg [`RW-1:0] o_mem_data,
    input i_mem_req, i_mem_we,
    output reg o_mem_ack,
    input i_mem_next,

    // Wishbone interface
    output reg wb_cyc,
    output reg wb_stb,
    output reg [`WB_DATA_W-1:0] wb_o_dat,
    input [`WB_DATA_W-1:0] wb_i_dat,
    output reg [`WB_ADDR_W-1:0]  wb_adr,
    output reg wb_we,
    input wb_ack,
    input wb_err,
    input wb_rty,
    output reg [`WB_SEL_BITS-1:0] wb_sel
);

assign wb_sel = 2'b11;

always @(posedge i_clk) begin
    if (i_rst) begin
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        o_mem_ack <= 1'b0;
    end else if (i_mem_req & ~wb_cyc & ~o_mem_ack) begin // decide bus rules (is ~o_mem_ack needed -> speed; next was intended to use here or below?)
        wb_cyc <= 1'b1;
        wb_stb <= 1'b1;
        wb_adr <= i_mem_addr;
        wb_o_dat <= (i_mem_we ? i_mem_data : `WB_DATA_W'b0);
        wb_we <= i_mem_we;
        o_mem_ack <= 1'b0;
    end else if (wb_cyc & wb_stb & wb_ack & i_mem_next & i_mem_req) begin
        wb_adr <= i_mem_addr;
        o_mem_ack <= 1'b1;
        o_mem_data <= (wb_we ? `RW'b0 : wb_i_dat);
    end else if (wb_cyc & wb_stb & wb_ack) begin 
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        o_mem_ack <= 1'b1;
        o_mem_data <= (wb_we ? `RW'b0 : wb_i_dat);
        // TODO: handle bus err/rty
    end else begin
        o_mem_ack <= 1'b0;
    end
end

endmodule
