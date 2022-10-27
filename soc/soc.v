// Connects ppcpu core with pcpu peripherals for use on original pcpu dev board


`include "config.v"

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module soc (
    input wire i_clk,
    input wire i_rst,

    output wire dr_dqml, dr_dqmh,
  	output wire dr_cs_n, dr_cas_n, dr_ras_n, dr_we_n, dr_cke,
   	output wire [1:0] dr_ba,
	output wire  [12:0] dr_a,
   	inout [15:0] dr_dq,
	output wire dr_clk,

    input i_irq,
    
    output [3:0] pc_leds,
    output ser_clk,
    output ser_data,

    input uart_rx,
    output uart_tx
);

reg por_n = 1'b0;
reg [10:0] por_cnt = 'b0;
always @(posedge d_clk) begin
    por_cnt <= por_cnt + 'b1;
    if(&por_cnt)
        por_n <= 1'b1;
    if(~i_rst)
        por_n <= 1'b0;
end

wire cw_clk;
wire [`RW-1:0] cw_io_i, cw_io_o;
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;

wire [15:0] dbg_pc, dbg_r0;
//assign pc_leds = {wb_we, wb_ack, sdram_req, sdram_req_active};
assign pc_leds = dbg_pc[3:0];

reg [4:0] clk_div;
wire d_clk = clk_div[0];
wire d_rst = ~(i_rst & por_n);
always @(posedge i_clk) begin
    clk_div <= clk_div + 5'b1;
end

top_cw top_cw (
    .i_clk(d_clk),
    .i_rst(d_rst),
    .cw_clk(cw_clk),
    .cw_io_i(cw_io_i),
    .cw_io_o(cw_io_o),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),
    .i_irq(i_irq),
    .dbg_pc(dbg_pc),
    .dbg_r0(dbg_r0)
);

wire wb_cyc;
wire wb_stb;
wire [`WB_DATA_W-1:0] wb_o_dat;
reg [`WB_DATA_W-1:0] wb_i_dat;
wire [`WB_ADDR_W-1:0]  wb_adr;
wire wb_we;
reg wb_ack;
wire wb_err;
wire [`WB_SEL_BITS-1:0] wb_sel;

wb_decomp wb_decomp (
    .i_clk(cw_clk),
    .i_rst(d_rst),

    .cw_io_i(cw_io_o),
    .cw_io_o(cw_io_i),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb),
    .wb_o_dat(wb_o_dat),
    .wb_i_dat(wb_i_dat),
    .wb_adr(wb_adr),
    .wb_we(wb_we),
    .wb_ack(wb_ack),
    .wb_err(wb_err),
    .wb_sel(wb_sel)
);

wire [`RW-1:0] sdram_data_out;
assign sdram_data_out = data_out[15:0];

wire [31:0] data_out;

reg sdram_req;
reg sdram_req_active;

// Sdram interface to wishbone adapter
always @(posedge cw_clk) begin
    if(d_rst) begin
        sdram_req <= 1'b0;
        sdram_req_active <= 1'b0;
    end else if (sdram_req & c_cack & wb_we) begin
        sdram_req <= 1'b0;
        sdram_req_active <= 1'b0;
    end else if (sdram_req & c_cack & c_read_ready & ~wb_we) begin
        sdram_req <= 1'b0;
        sdram_req_active <= 1'b0;
    end else if (sdram_req & c_cack & ~wb_we) begin
        sdram_req <= 1'b0;
    end else if (sdram_req_active & c_read_ready & ~wb_we) begin
        sdram_req_active <= 1'b0;
    end else if ((wb_adr >= 24'h100004) & (wb_adr < 24'hffe000) & wb_cyc & wb_stb & ~sdram_req_active) begin
        sdram_req <= 1'b1;
        sdram_req_active <= 1'b1;
    end
end

wire sdram_ack = sdram_req_active & ((c_read_ready & ~wb_we) | (c_cack & wb_we));

reg prev_stb;
always @(posedge i_clk) begin
    prev_stb <= wb_stb;
end

wire c_busy, c_read_ready, c_cack;
sdram sdram (
    .clk(i_clk),
    .srclk(cw_clk),
    .c_addr(wb_adr),
    .c_data_in(wb_o_dat),
    .c_data_out(data_out),
    .c_addr_sel(wb_sel),
    .c_read_req(sdram_req & ~wb_we),
    .c_write_req(sdram_req & wb_we),
    .c_busy(c_busy),
    .c_read_ready(c_read_ready),
    .c_cack(c_cack),

    .dr_dqml(dr_dqml), .dr_dqmh(dr_dqmh),
    .dr_cs_n(dr_cs_n), .dr_cas_n(dr_cas_n), .dr_ras_n(dr_ras_n), .dr_we_n(dr_we_n), .dr_cke(dr_cke),
    .dr_ba(dr_ba),
    .dr_a(dr_a),
    .dr_dq(dr_dq)
);

assign dr_clk = ~i_clk; // ram controller depends on setting edges half cycle before ram

wire [`RW-1:0] rom_data;
soc_rom soc_rom (
    .in_addr(wb_adr),
    .out_data(rom_data)
);

wire [`WB_DATA_W-1:0] uart_wb_i_dat;
wire uart_wb_ack;
uart uart (
    .i_clk(cw_clk),
    .i_full_clk(i_clk),
    .i_rst(d_rst),

    .tx(uart_tx),
    .rx(uart_rx),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= 24'h100000 && wb_adr <= 24'h100002)),
    .wb_adr(wb_adr - 24'h100000),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(uart_wb_i_dat),
    .wb_ack(uart_wb_ack)
);

always @(*) begin
    if (wb_adr >= 24'h100000 && wb_adr <= 24'h100002) begin
        wb_i_dat = uart_wb_i_dat;
        wb_ack = wb_cyc & wb_stb;
    end else if (wb_adr < 24'hffe000) begin
        wb_i_dat = sdram_data_out;
        wb_ack = sdram_ack;
    end else begin
        wb_i_dat = rom_data;
        wb_ack = wb_cyc & wb_stb;
    end
end

serialout r0_leds (
    .clk(d_clk),
    .data(dbg_r0[7:0]),
    .sclk(ser_clk),
    .sdata(ser_data)
);

endmodule
