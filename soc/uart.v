module uart (
    input i_clk,
    input i_full_clk,
    input i_rst,

    input rx,
    output reg tx,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat,
    //output  dbg_trig
);

localparam BAUD_RATE = 115200;
localparam OVERSAMPLE = 8;
localparam OVERSAMPLE_LOG = 3;
localparam CLOCK_FREQ = 50_000_000;
localparam UART_CLOCK_DIV = CLOCK_FREQ/(BAUD_RATE*2);
localparam OSMPL_CLOCK_DIV = CLOCK_FREQ/(BAUD_RATE*OVERSAMPLE*2);
localparam RX_BUFF_SIZE = 8;
localparam TX_BUFF_SIZE = 8;

reg uart_os_clk = 1'b0;
reg [5:0] uart_os_clk_cnt = 6'b0;
reg uart_clk = 1'b0;
reg [9:0] uart_clk_cnt = 10'b0;
always @(posedge i_full_clk) begin
    uart_os_clk_cnt <= uart_os_clk_cnt + 6'b1;
    uart_clk_cnt <= uart_clk_cnt + 6'b1;

    if (uart_os_clk_cnt == 27) begin
        uart_os_clk_cnt <= 6'b0;
        uart_os_clk <= ~uart_os_clk;
    end

    if (uart_clk_cnt == UART_CLOCK_DIV) begin
        uart_clk_cnt <= 10'b0;
        uart_clk <= ~uart_clk;
    end
end

// RECEIVE
// reg rx_active = 1'b0, rx_stop = 1'b0;
// reg [OVERSAMPLE_LOG:0] rx_os_cnt;
reg [7:0] rx_result;
reg [2:0] rx_res_bit;

//wire rx_submit = (rx_stop & (rx_os_cnt == 0));
wire rx_submit = (sub_clk_cnt == 4'b0111) && (state == 4'b1001);
//assign dbg_trig = rx_submit & ~rx;

// I DONT KNOW WHY but this code from old cpu works just fine,
// but preetier new code that seems to behave in the same way misses the stop bits. WHY???
reg [3:0] state = 4'b0;
reg [3:0] sub_clk_cnt = 4'b0;
//reg r1_trig = 1'b0;
always @(posedge uart_os_clk) begin
    case (state)
        4'b0: begin
            // if start bit
            if(rx == 1'b0) begin
                state <= 1'b1;
            end
        end
        4'b1001: begin
            if(sub_clk_cnt == 4'b0111) begin
                sub_clk_cnt <= 4'b0;
                state <= 4'b0;
                //r1_trig <= ~r1_trig;
            end else begin
                sub_clk_cnt <= sub_clk_cnt+4'b1;
            end
        end
        default: begin // default read bit
            if((state != 4'b1 && sub_clk_cnt == 4'b0111) || sub_clk_cnt == 4'b1010) begin //delay first clock by one and half
                rx_result[state-4'b1] <= rx;
                state <= state+4'b1;
                sub_clk_cnt <= 4'b0;
                //r1_trig <= ~r1_trig;
            end else begin
                sub_clk_cnt <= sub_clk_cnt+4'b1;
            end
        end
    endcase
end

// reg r2_trig = 1'b0;
// always @(posedge uart_os_clk) begin
//     if (i_rst) begin
//         rx_active <= 1'b0;
//         rx_os_cnt <= 'b0;
//         rx_stop <= 1'b0;
//     end else begin 
//         if (~rx_active & ~rx) begin
//             rx_active <= 1'b1;
//             rx_res_bit <= 'b0;
//             rx_os_cnt <= 4'b1010;
//         end else if (rx_stop & (rx_os_cnt == 0)) begin
//             rx_active <= 1'b0;
//             rx_stop <= 1'b0;
//             r2_trig <= ~r2_trig;
//             //dbg_trig <= ~dbg_trig;
//         end else if (rx_active & ~rx_stop & (rx_os_cnt == 0)) begin
//             rx_os_cnt <= 4'b0111;
//             r2_trig <= ~r2_trig;
//             //rx_result[rx_res_bit] <= rx;
//             rx_res_bit <= rx_res_bit + 1'b1;
//             rx_stop <= (rx_res_bit == 3'b111);
//             //dbg_trig <= ~dbg_trig;
//         end else if (rx_active) begin
//             rx_os_cnt <= rx_os_cnt - 1'b1;
//         end
//     end
// end


reg [7:0] rx_fifo [RX_BUFF_SIZE-1:0];
reg [2:0] rx_write_ptr, rx_read_ptr;

always @(posedge uart_os_clk) begin
    if (i_rst) begin
        rx_write_ptr <= 3'b0;
    end else if (rx_submit) begin
        rx_fifo[rx_write_ptr] <= rx_result;
        rx_write_ptr <= rx_write_ptr + 3'b1;
    end
end

wire rx_data_available = |(rx_read_ptr ^ rx_write_ptr);
reg [2:0] rx_prev_data;
wire rx_irq = (rx_prev_data == 3'b0 & ((rx_write_ptr-rx_read_ptr) != 3'b0)); 

always @(posedge i_clk) begin
    if (i_rst) begin
        rx_read_ptr <= 3'b0;
    end else begin
        rx_prev_data <= rx_write_ptr-rx_read_ptr;
        if (wb_cyc & wb_stb & ~wb_we & wb_adr == 24'h1 & rx_data_available) begin
            rx_read_ptr <= rx_read_ptr + 3'b1;
            rx_prev_data <= (rx_write_ptr-rx_read_ptr-3'b1);
        end
    end
end

// TRANSMIT

reg [1:0] tx_state;
wire tx_ready = (tx_state == 2'b0);
wire tx_start = tx_data_avail;
reg [7:0] tx_data;
reg [2:0] tx_data_cnt;

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
        tx_state <= 2'b0;
    end
end

reg [7:0] tx_fifo [RX_BUFF_SIZE-1:0];
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
        wb_o_dat = {14'b0, ~tx_full, rx_data_available};
    end else if (wb_adr == 24'h1) begin
        wb_o_dat = rx_fifo[rx_read_ptr];
    end else begin
        wb_o_dat = 16'b0;
    end
end

endmodule
