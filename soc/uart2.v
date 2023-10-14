module uart2 (
    input i_clk,
    input i_full_clk,
    input i_rst,

    input rx,
    output reg tx,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat
);

localparam BAUD_RATE = 9600;
localparam CLOCK_FREQ = 50_000_000;
localparam UART_CLOCK_DIV = CLOCK_FREQ/(BAUD_RATE*2);
localparam TX_BUFF_SIZE = 8;

reg uart_clk = 1'b0;
reg [20 :0] uart_clk_cnt = 10'b0;
always @(posedge i_full_clk) begin
    uart_clk_cnt <= uart_clk_cnt + 6'b1;

    if (uart_clk_cnt == UART_CLOCK_DIV) begin
        uart_clk_cnt <= 10'b0;
        uart_clk <= ~uart_clk;
    end
end


// TRANSMIT

reg [1:0] tx_state;
wire tx_ready = (tx_state == 2'b0);
wire tx_start = tx_data_avail;
reg [7:0] tx_data;
reg [2:0] tx_data_cnt;
reg [10:0] hold_cnt;

always @(posedge uart_clk) begin
    if (i_rst) begin
        tx <= 1'b1;        
        tx_state <= 2'b0;
    end else if ((tx_state == 2'b0) & tx_start) begin
        tx <= 1'b0; // start bit
        tx_data_cnt <= 3'b0;
        tx_state <= 2'b1; 
    end else if (tx_state == 2'b1) begin
        tx <= tx_data[tx_data_cnt];
        tx_data_cnt <= tx_data_cnt + 1'b1;
        if (&tx_data_cnt)
            tx_state <= 2'b10;
    end else if (tx_state == 2'b10) begin
        tx <= 1'b1; // stop bit
        tx_state <= 2'b11;
        hold_cnt <= 0;
    end else if (tx_state == 2'b11) begin
        hold_cnt <= hold_cnt + 1;
        if (hold_cnt == 8) begin
              tx_state <= 2'b0;
        end
        tx <= 1'b1;
    end
end

reg [7:0] tx_fifo [TX_BUFF_SIZE-1:0];
reg [2:0] tx_write_ptr, tx_read_ptr;
wire tx_data_avail = |(tx_write_ptr^tx_read_ptr);
wire tx_full = (tx_write_ptr+3'b1) == tx_read_ptr;

always @(posedge uart_clk) begin
    if (i_rst) begin
        tx_read_ptr <= 3'b0;
    end else if (tx_ready & tx_data_avail) begin
        tx_read_ptr <= tx_read_ptr + 3'b1;
        tx_data <= tx_fifo[tx_read_ptr];
    end
end

reg [2:0] tx_prev_data;
wire tx_empty_irq = (tx_prev_data != 3'b0) && ((tx_write_ptr-tx_read_ptr) == 3'b0);

always @(posedge i_clk) begin
    if (i_rst) begin
        tx_write_ptr <= 3'b0;
    end else begin 
        tx_prev_data <= (tx_write_ptr-tx_read_ptr);
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'h2) begin
            tx_write_ptr <= tx_write_ptr + 3'b1;
            tx_fifo[tx_write_ptr] <= wb_i_dat[7:0]; 
        end
    end
end

assign wb_ack = wb_cyc & wb_stb;
always @* begin
    if (wb_adr == 24'h0) begin
        wb_o_dat = {14'b0, ~tx_full, 1'b0};
    end else begin
        wb_o_dat = 16'b0;
    end
end

endmodule
