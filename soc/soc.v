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

    input uart_rx, uart2_rx,
    output uart_tx, uart2_tx,

    inout scl,
    inout sda,

    input ps_clk, ps_data,

    output spi_sck,
    output spi_mosi,
    input spi_miso,
    output [1:0] spi_ss,

    output vga_hsync, vga_vsync,
    output [2:0] vga_r, vga_g,
    output [1:0] vga_b
);

reg por_n = 1'b0;
reg [15:0] por_cnt = 'b0;
always @(posedge d_clk) begin
    por_cnt <= por_cnt + 'b1;
    if(&por_cnt)
        por_n <= 1'b1;
    if(~i_rst)
        por_n <= 1'b0;
end

wire [15:0] dbg_pc, dbg_r0;
//assign pc_leds = {wb_we, wb_ack, sdram_req, sdram_req_active};
//assign pc_leds = dbg_pc[3:0];
assign pc_leds = regled;
reg regled;
reg prev_irq;
always @(posedge d_clk) begin
    prev_irq <= timer_irq;
    regled <= regled ^ (timer_irq ^ prev_irq && timer_irq == 1'b1);
end

reg [4:0] clk_div;
wire d_clk = clk_div[0];
wire d_rst = ~(i_rst & por_n);
always @(posedge i_clk) begin
    clk_div <= clk_div + 5'b1;
end

`define MPRJ_IO_PADS 38
wire [`MPRJ_IO_PADS-1:0] m_io_in;
wire [`MPRJ_IO_PADS-1:0] m_io_out;
wire [`MPRJ_IO_PADS-1:0] m_io_oeb;

top top (
    .m_io_in(m_io_in),
    .m_io_out(m_io_out),
    .m_io_oeb(m_io_oeb),
    .mgt_wb_clk_i(d_clk),
    .mgt_wb_rst_i(d_rst),
    .mgt_wb_cyc_i(1'b0),
    .mgt_wb_stb_i(1'b0),
    .la_oenb({128{1'b1}}),
    .mgt_wb_we_i(1'b0),
    .mgt_wb_dat_i('b0),
    .mgt_wb_sel_i('b0),
    .mgt_wb_adr_i('b0),
    .la_data_in('b0),
    .mgt_wb_ack_o(ignored_wb_ack),
    .mgt_wb_dat_o(ignored_wb_dat_o),
    .la_data_out(ignored_data_out),
    .irq(ignored_irq),
    .dbg_r0(dbg_r0),
    .dbg_pc(dbg_pc)
);

wire [31:0] ignored_wb_dat_o;
wire ignored_wb_ack;
wire [127:0] ignored_data_out;
wire [2:0] ignored_irq;

// pins to cw bus
wire [`RW-1:0] cw_io_i;
wire [`RW-1:0] cw_io_o;
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;
wire cw_clk;
wire cw_rst;

localparam CW_PIN_OFF=8;
assign cw_req = m_io_out[CW_PIN_OFF+0];
assign cw_dir = m_io_out[CW_PIN_OFF+1];
assign cw_io_o = m_io_out[CW_PIN_OFF+17:CW_PIN_OFF+2];
assign m_io_in[CW_PIN_OFF+17:CW_PIN_OFF+2] = cw_io_i;
assign m_io_in[CW_PIN_OFF+18] = cw_ack;
assign m_io_in[CW_PIN_OFF+19] = cw_err;
assign cw_clk = m_io_out[CW_PIN_OFF+20];
assign cw_rst = m_io_out[CW_PIN_OFF+21];
assign m_io_in[CW_PIN_OFF+22] = m_irq;
assign m_io_in[CW_PIN_OFF+23] = 1'b0; // split clock

wire wb_cyc;
wire wb_stb;
wire [`WB_DATA_W-1:0] wb_o_dat;
reg [`WB_DATA_W-1:0] wb_i_dat;
wire [`WB_ADDR_W-1:0]  wb_adr;
wire wb_we;
reg wb_ack;
reg wb_err;
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

/*
 * Address map
 */

localparam UART_BASE =  24'h002000;
localparam UART_END =   24'h002003;

localparam UART2_BASE = 24'h002004;
localparam UART2_END =  24'h002007;

localparam TIMER_BASE = 24'h002008;
localparam TIMER_END =  24'h00200a;

localparam IRQC_BASE =  24'h00200c;
localparam IRQC_END =   24'h00200e;

localparam SPI_BASE =  24'h002010;
localparam SPI_END =   24'h002014;

localparam KBD_BASE =  24'h002020;
localparam KBD_END =   24'h002020;

localparam I2C_BASE =  24'h002026;
localparam I2C_END =   24'h00202a;

localparam VGA_BASE =  24'h003000;
localparam VGA_END =   24'h006002;

localparam SDRAM_BASE = 24'h100000; 
localparam SDRAM_END =  24'hffdfff;

localparam ROM_BASE =   24'hffe000;
localparam ROM_END =    24'hffffff;


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
    end else if ((wb_adr >= SDRAM_BASE) & (wb_adr <= SDRAM_END) & wb_cyc & wb_stb & ~sdram_req_active) begin
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
    .clk(cw_clk),
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

assign dr_clk = ~cw_clk;//i_clk; // ram controller depends on setting edges half cycle before ram

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
    .wb_stb(wb_stb & ((wb_adr >= UART_BASE) && (wb_adr <= UART_END))),
    .wb_adr(wb_adr - UART_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(uart_wb_i_dat),
    .wb_ack(uart_wb_ack)
);

wire [`WB_DATA_W-1:0] uart2_wb_i_dat;
wire uart2_wb_ack;
uart2 uart2 (
    .i_clk(cw_clk),
    .i_full_clk(i_clk),
    .i_rst(d_rst),

    .tx(uart2_tx),
    .rx(uart2_rx),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & ((wb_adr >= UART2_BASE) && (wb_adr <= UART2_END))),
    .wb_adr(wb_adr - UART2_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(uart2_wb_i_dat),
    .wb_ack(uart2_wb_ack)
);

wire timer_irq, timer_wb_ack;
wire [`WB_DATA_W-1:0] timer_wb_i_dat;
timer timer (
    .i_clk(cw_clk),
    .i_rst(d_rst),
    
    .irq(timer_irq),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= TIMER_BASE && wb_adr <= TIMER_END)),
    .wb_adr(wb_adr - TIMER_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(timer_wb_i_dat),
    .wb_ack(timer_wb_ack)
);

wire ps2_irq;
wire m_irq, irqc_wb_ack;
wire [`WB_DATA_W-1:0] irqc_wb_i_dat;
irq_ctrl irq_ctrl (
    .i_clk(cw_clk),
    .i_rst(d_rst),

    .o_irq(m_irq),
    .i_irq({14'b0, ps2_irq, timer_irq}),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= IRQC_BASE && wb_adr <= IRQC_END)),
    .wb_adr(wb_adr - IRQC_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(irqc_wb_i_dat),
    .wb_ack(irqc_wb_ack)
);

wire spi_wb_ack;
wire [`WB_DATA_W-1:0] spi_wb_i_dat;
spi spi (
    .i_clk(cw_clk),
    .i_rst(d_rst),

    .spi_sck(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_ss(spi_ss),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= SPI_BASE && wb_adr <= SPI_END)),
    .wb_adr(wb_adr - SPI_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(spi_wb_i_dat),
    .wb_ack(spi_wb_ack)
);

wire vga_wb_ack;
vga vga (
    .raw_clk(i_clk),
    .cpu_clk(cw_clk),

    .hsync(vga_vsync),
    .vsync(vga_hsync),
    .r(vga_r),
    .g(vga_g),
    .b(vga_b),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= VGA_BASE && wb_adr <= VGA_END)),
    .wb_adr(wb_adr - VGA_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_ack(vga_wb_ack)
);

wire [7:0] ps2_wb_i_dat;
ps2_kbd ps2_kbd (
    .clk(cw_clk),
    
    .ps_clk(ps_clk),
    .ps_data(ps_data),

    .scancode_sync(ps2_wb_i_dat),
    .irq_sync(ps2_irq)
);

wire i2c_wb_ack;
wire [`WB_DATA_W-1:0] i2c_wb_i_dat;

wire o_sda, d_sda;
wire o_scl;
wire i_scl = scl;
assign scl = o_scl ? 1'bZ : 1'b0;
assign sda = d_sda ? 1'bZ : o_sda;

i2c i2c (
    .i_clk(cw_clk),
    .i_full_clk(i_clk),
    .i_rst(d_rst),

    .i_scl(i_scl),
    .o_scl(o_scl),
    .i_sda(sda),
    .o_sda(o_sda),
    .d_sda(d_sda),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= I2C_BASE && wb_adr <= I2C_END)),
    .wb_adr(wb_adr - I2C_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(i2c_wb_i_dat),
    .wb_ack(i2c_wb_ack)
);

always @(*) begin
    if (wb_adr >= SDRAM_BASE) begin
        if ((wb_adr >= SDRAM_BASE) && (wb_adr <= SDRAM_END)) begin
            wb_i_dat = sdram_data_out;
            wb_ack = sdram_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= ROM_BASE) && (wb_adr <= ROM_END)) begin
            wb_i_dat = rom_data;
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else begin
            wb_i_dat = 16'b0;
            wb_ack = 1'b0;
            wb_err = 1'b1;
        end
    end else begin
        if ((wb_adr >= UART_BASE) && (wb_adr <= UART_END)) begin
            wb_i_dat = uart_wb_i_dat;
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else if ((wb_adr >= UART2_BASE) && (wb_adr <= UART2_END)) begin
            wb_i_dat = uart2_wb_i_dat;
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else if ((wb_adr >= TIMER_BASE) && (wb_adr <= TIMER_END)) begin
            wb_i_dat = timer_wb_i_dat;
            wb_ack = timer_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= IRQC_BASE) && (wb_adr <= IRQC_END)) begin
            wb_i_dat = irqc_wb_i_dat;
            wb_ack = irqc_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= SPI_BASE) && (wb_adr <= SPI_END)) begin
            wb_i_dat = spi_wb_i_dat;
            wb_ack = spi_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= KBD_BASE) && (wb_adr <= KBD_END)) begin
            wb_i_dat = {8'b0, ps2_wb_i_dat};
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else if ((wb_adr >= I2C_BASE) && (wb_adr <= I2C_END)) begin
            wb_i_dat = i2c_wb_i_dat;
            wb_ack = i2c_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= VGA_BASE) && (wb_adr <= VGA_END)) begin
            wb_i_dat = 16'b0;
            wb_ack = vga_wb_ack;
            wb_err = 1'b0;
        end else begin
            wb_i_dat = 16'b0;
            wb_ack = 1'b0;
            wb_err = 1'b1;
        end
    end
end

serialout r0_leds (
    .clk(d_clk),
    .data(dbg_r0[7:0]),
    .sclk(ser_clk),
    .sdata(ser_data)
);

endmodule
