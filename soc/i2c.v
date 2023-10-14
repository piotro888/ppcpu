module i2c(
    input i_clk,
    input i_full_clk,
    input i_rst,

    output o_scl,
    input i_scl,
    output reg o_sda,
    input i_sda,
    output reg d_sda,


    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output [15:0] wb_o_dat
);

localparam WB_I2C_ADDR = 2'b0;
localparam WB_I2C_REG = 2'b01;
localparam WB_I2C_DATA = 2'b10;
localparam WB_I2C_STATE = 2'b11;

reg [7:0] req_addr, req_reg, req_i_data;
reg start_flag, start_flag_slow;
reg busy_flag, busy_flag_slow;

reg [7:0] data_read;
reg [7:0] data_read_sync;

reg [9:0] clock_cnt;
wire i2c_clock = clock_cnt[9]; // ~ 50 kHz - Arduino I2C cant keep up for longer time with 100 kHz clock
wire i2c_ctl_clock = clock_cnt[8]; // faster control clock to control state both on falling and rising edge
always @(posedge i_full_clk) begin
    clock_cnt <= clock_cnt + 10'b1;
end

reg clock_inhibit, clock_low;

assign o_scl = clock_inhibit ? clock_low  : i2c_clock;

reg [4:0] state;
reg [4:0] cnt;
reg req_error;

always @(negedge i2c_ctl_clock) begin
    if (i_rst) begin
        state <= 5'b0;
        cnt <= 5'b0;
        start_flag_slow <= 1'b0;
        busy_flag_slow <= 1'b0;
        clock_inhibit <= 1'b1;
        req_error <= 1'b0;
        data_read <= 16'b0;

        o_sda <= 1'b1;
        d_sda <= 1'b0;
        clock_low <= 1'b1;
    end else begin
        case (state)
        default: begin end
        5'b0: begin
            o_sda <= 1'b1;
            d_sda <= 1'b0;
            
            clock_inhibit <= 1'b1;
            clock_low <= 1'b1;
            if ((start_flag ^ start_flag_slow) & ~i2c_clock) begin
                o_sda <= 1'b0; // START CONDITION scl & !sda 
                state <= 5'b1;
                cnt <= 5'b0;
                req_error <= 1'b0;
                start_flag_slow <= start_flag_slow ^ 1'b1;
            end
        end
        5'b1: begin
            if (i2c_clock) begin
                d_sda <= 1'b0;
                clock_low <= 1'b0;
            end else begin
                d_sda <= 1'b0;
                clock_low <= 1'b1;
                clock_inhibit <= 1'b0; // first bit should be set after scl falling edge with sda low

                o_sda <= req_addr[5'd7 - cnt];
                cnt <= cnt + 5'b1;
                if (cnt == 5'd7) begin
                    state <= 5'b10;
                    o_sda <= 1'b0; // override last bit to always write in first request
                end
            end
        end
        5'b10: begin
            if (~i2c_clock) begin
                d_sda <= 1'b1; // input ack request
                cnt <= 5'b0;
                state <= 5'b11;
            end
        end
        5'b11: begin
            if (i2c_clock) begin
                d_sda <= 1'b1;
                if (i_sda == 1'b0) begin // sample ACK at rising edge
                    state <= 5'b100;
                    cnt <= 5'b0;
                end else begin
                    req_error <= 1'b1;
                    state <= 5'b111;
                end
            end
        end
        5'b100: begin
            if (state == 5'b100 & ~i2c_clock) begin
                d_sda <= 1'b0;
                o_sda <= req_reg[5'd7 - cnt];
                cnt <= cnt + 5'b1;
                if (cnt == 5'd7) begin
                    state <= 5'b101;
                end
            end
        end
        5'b101: begin
            if (~i2c_clock) begin
                d_sda <= 1'b1; // input ack request
                cnt <= 5'b0;
                state <= 5'b110;
            end
        end 
        5'b110: begin
            if (i2c_clock) begin
                d_sda <= 1'b1;
                if (i_sda == 1'b0) begin // sample ACK at rising edge
                    if (req_addr[0]) begin
                        state <= 5'b10000; // read
                    end else begin
                        state <= 5'b1001; // write
                    end
                end else begin
                    req_error <= 1'b1;
                    state <= 5'b111;
                end
            end
        end
        5'b111: begin
            d_sda <= 1'b0;
            o_sda <= 1'b1;
            clock_inhibit <= 1'b1;
            clock_low <= 1'b0;
            cnt <= 5'b0;
            if (~i2c_clock) begin
                state <= 5'b1000;
            end
        end
        5'b1000: begin
            if (cnt == 5'b0 && i2c_clock) begin // STOP
                d_sda <= 1'b0;
                o_sda <= 1'b0;
                clock_inhibit <= 1'b1;
                clock_low <= 1'b0;
                state <= 5'b1000;
            end else begin // STOP
                d_sda <= 1'b0;
                o_sda <= 1'b0;
                clock_inhibit <= 1'b1;
                clock_low <= 1'b1;
                cnt <= cnt + 5'b1;
                if (cnt == 5'd3) begin
                    state <= 5'b0;
                    busy_flag_slow <= busy_flag_slow ^ 1'b1;
                end
            end
        end
        5'b1001: begin
            if (~i2c_clock) begin
                d_sda <= 1'b0;
                o_sda <= req_i_data[5'd7 - cnt];
                cnt <= cnt + 5'b1;
                if (cnt == 5'd7) begin
                    state <= 5'b1010;
                end
            end
        end
        5'b1010: begin
            if (~i2c_clock) begin
                d_sda <= 1'b1; // input ack request
                cnt <= 5'b0;
                state <= 5'b1011;
            end
        end
        5'b1011: begin
            if (i2c_clock) begin
                d_sda <= 1'b1;
                if (i_sda == 1'b0) begin // sample ACK at rising edge
                    state <= 5'b111;
                end else begin
                    req_error <= 1'b1;
                    state <= 5'b111;
                end
            end
        end
        5'b10000: begin
            if (~i2c_clock) begin // read - resend start signal
                d_sda <= 1'b0;
                o_sda <= 1'b1;
                state <= 5'b10001;
            end
        end
        5'b10001: begin
            if (i2c_clock) begin
                d_sda <= 1'b0;
                clock_inhibit <= 1'b1;
                clock_low <= 1'b1;
            end else begin
                d_sda <= 1'b0;
                o_sda <= 1'b0;
                state <= 5'b10010;
                cnt <= 5'b0;
            end
        end
        5'b10010: begin
            if (~i2c_clock) begin // read - send address (2nd time)
                clock_inhibit <= 1'b0; 
                d_sda <= 1'b0;
                o_sda <= req_addr[5'd7 - cnt];
                cnt <= cnt + 5'b1;
                if (cnt == 5'd7) begin
                    state <= 5'b10100;
                end
            end
        end
        5'b10100: begin
            if (~i2c_clock) begin
                d_sda <= 1'b1; // input ack request
                cnt <= 5'b0;
                state <= 5'b10101;
            end 
        end
        5'b10101: begin
            if (i2c_clock) begin
                d_sda <= 1'b1;
                if (i_sda == 1'b0) begin // sample ACK at rising edge
                    state <= 5'b10111;
                end else begin
                    req_error <= 1'b1;
                    state <= 5'b111;
                end
            end
        end
        5'b10111: begin
            if (~i2c_clock) begin // sample data from device
                d_sda <= 1'b1;
                // detect clock streching (clock 1 state is pull-up, clock here should be 1, 
                // but device can pull it low if it is busy and we need to wait for response).
                if (i_scl) begin 
                    data_read[5'd7-cnt] <= i_sda;
                    cnt <= cnt + 5'b1;
                    if (cnt == 5'd7) begin
                        state <= 5'b11000;
                    end
                end
            end
        end
        5'b11000: begin
            if (~i2c_clock) begin
                d_sda <= 1'b0;
                o_sda <= 1'b1; // NACK byte read = end of transmision
                state <= 5'b11001;
            end
        end
        5'b11001: begin
            state <= 5'b111; // stop after clock edge
        end
    endcase
    end
end

reg req_err_sync, req_busy_sync;
always @(posedge i_clk) begin
    if (i_rst) begin
        req_addr <= 8'b0;
        req_reg <= 8'b0;
        req_i_data <= 8'b0;
        start_flag <= 1'b0;
        busy_flag <= 1'b0;
        data_read_sync <= 8'b0;
        req_err_sync <= 1'b0;
        req_busy_sync <= 1'b0;
    end else if (wb_cyc & wb_stb & (wb_adr == WB_I2C_ADDR) & wb_we) begin
        req_addr <= wb_i_dat[7:0];
    end else if (wb_cyc & wb_stb & (wb_adr == WB_I2C_REG) & wb_we) begin
        req_reg <= wb_i_dat[7:0];
    end else if (wb_cyc & wb_stb & (wb_adr == WB_I2C_DATA) & wb_we) begin
        req_i_data <= wb_i_dat[7:0];
    end else if (wb_cyc & wb_stb & (wb_adr == WB_I2C_STATE) & wb_we) begin
        start_flag <= start_flag ^ 1'b1;
        busy_flag <= busy_flag ^ 1'b1;
    end

    if (!i_rst) begin
        data_read_sync <= data_read;
        req_err_sync <= req_error;
        req_busy_sync <= busy_flag ^ busy_flag_slow;
    end
end

assign wb_o_dat = ((wb_adr == WB_I2C_STATE) ? {14'b0, req_err_sync, req_busy_sync} : data_read_sync);
assign wb_ack = wb_cyc & wb_stb;

endmodule
