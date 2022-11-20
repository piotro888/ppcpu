module spi (
    input i_clk,
    input i_rst,

    output spi_sck,
    output reg spi_mosi,
    input spi_miso,
    output reg [1:0] spi_ss,

    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat
);

// MSB first
wire CPOL = mode_reg[5];
wire CPHA = mode_reg[4];

reg spi_sck_s;
assign spi_sck = spi_sck_s ^ (~CPOL);

wire spi_start_req;
reg  req_in_progress;
reg  req_end_xor;
reg  req_step;
reg [2:0] req_bit;
reg [7:0] tx_byte, rx_byte;

always @(posedge spi_logic_clk) begin
    if (i_rst) begin
        spi_sck_s <= 1'b1;
        spi_mosi <= 1'b1;
        
        req_step <= 1'b0;
        req_end_xor <= 1'b0;
    end else if (req_in_progress & ~req_step) begin
        spi_sck_s <= CPHA;
        rx_byte[req_bit] <= spi_miso;
        
        req_bit <= req_bit - 3'b1;
        req_step <= 1'b1;
        req_in_progress <= (req_bit != 3'b0);
        req_end_xor <= req_end_xor^(req_bit == 3'b0);
    end else if (req_in_progress & req_step) begin
        spi_sck_s <= ~CPHA;
        spi_mosi <= tx_byte[req_bit];

        req_step <= 1'b0;
    end else if (spi_start_req) begin
        spi_sck_s <= ~CPHA;
        spi_mosi <= tx_byte[7];

        req_step <= 1'b0;
        req_bit <= 3'd7;
        req_in_progress <= 1'b1;
    end else begin
        spi_sck_s <= 1'b1;
        spi_mosi <= 1'b1;
    end
end

reg xfer_start_flag;
reg spi_busy;
always @(posedge i_clk) begin
    if (i_rst) begin
        xfer_start_flag <= 1'b0;
        spi_busy <= 1'b0;
		mode_reg <= 6'b000110;
        spi_ss <= 2'b11;
    end else begin
        if (spi_req_end)
            spi_busy <= 1'b0;

        if (wb_cyc & wb_stb & wb_we & (wb_adr == 24'b0)) begin
            xfer_start_flag <= ~xfer_start_flag;
            tx_byte <= wb_i_dat[7:0];
            spi_busy <= 1'b1;
        end else if (wb_cyc & wb_stb & wb_we & (wb_adr == 24'h3)) begin
            mode_reg <= wb_i_dat[5:0];
        end else if (wb_cyc & wb_stb & wb_we & (wb_adr == 24'h4)) begin
            spi_ss <= wb_i_dat[1:0];
        end
    end
end

always @(*) begin
    if (wb_adr == 24'h1)
        wb_o_dat = {8'b0, rx_byte};
    else if (wb_adr == 24'h2)
        wb_o_dat = {15'b0, spi_busy};
    else if (wb_adr == 24'h3)
        wb_o_dat = {10'b0, mode_reg};
    else if (wb_adr == 24'h4)
        wb_o_dat = {15'b0, spi_ss};
    else
        wb_o_dat = 16'b0;
end


reg [5:0] mode_reg;

wire [3:0] clk_div = mode_reg[3:0];
reg [9:0] clk_cnt;
assign spi_logic_clk = clk_cnt[clk_div];
always @(posedge i_clk) begin
    if (i_rst)
        clk_cnt <= 1'b0;
    else
        clk_cnt <= clk_cnt + 1'b1;
end

assign wb_ack = wb_cyc & wb_stb;

// CDC

reg [2:0] d_xfer_xor_sync;
always @(posedge spi_logic_clk) begin
    if(i_rst) begin
        d_xfer_xor_sync[2:0] <= 3'b0;
    end else begin
        d_xfer_xor_sync[0] <= xfer_start_flag;
        d_xfer_xor_sync[1] <= d_xfer_xor_sync[0];
        d_xfer_xor_sync[2] <= d_xfer_xor_sync[1];
    end
end
assign spi_start_req = (d_xfer_xor_sync[1] ^ d_xfer_xor_sync[2]) & ~i_rst;

reg [2:0] d_req_end_sync;
always @(posedge i_clk) begin
    if(i_rst) begin
        d_req_end_sync[2:0] <= 3'b0;
    end else begin
        d_req_end_sync[0] <= req_end_xor;
        d_req_end_sync[1] <= d_req_end_sync[0];
        d_req_end_sync[2] <= d_req_end_sync[1];
    end
end
wire spi_req_end = (d_req_end_sync[1] ^ d_req_end_sync[2]) & ~i_rst;

endmodule