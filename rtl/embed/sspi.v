module sspi (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    
    input i_clk,
    input i_rst,

    input spi_clk,
    input spi_mosi,
    output reg spi_miso,

    output reg wb_cyc,
    output reg wb_stb,
    output [23:0]  wb_adr,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat,
    output reg wb_we,
    output [1:0] wb_sel,
    input wb_ack,
    input wb_err
);

reg [2:0] sy_clk;
wire sclk_edge = (sy_clk[2] ^ sy_clk[1]) & ~sy_clk[2];
always @(posedge i_clk) begin
    if (i_rst) sy_clk <= 3'b0;
    else sy_clk <= {sy_clk[1], sy_clk[0], spi_clk};
end

reg [23:0] req_addr;
reg [15:0] req_data;
reg [15:0] res_data;
reg resp_err;

reg [4:0] bit_cnt;
reg [3:0] state;
localparam STATE_IDLE = 4'b0;
localparam STATE_ADDR = 4'b1;
localparam STATE_RW = 4'b10;
localparam STATE_WRITE_DAT = 4'b11;
localparam STATE_WRITE_WB = 4'b100;
localparam STATE_WB_RESP = 4'b101;
localparam STATE_READ_WB = 4'b110;
localparam STATE_READ_DAT = 4'b111;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= STATE_IDLE;
        bit_cnt <= 'b0;
        spi_miso <= 1'b1;
    end else if (sclk_edge) begin
        if (state == STATE_IDLE) begin
            spi_miso <= 1'b1;
            if (~spi_mosi)
                state <= STATE_ADDR; 
        end else if (state == STATE_ADDR) begin
            req_addr[bit_cnt] <= spi_mosi; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd23) begin
                state <= STATE_RW;
                bit_cnt <= 5'b0;
            end
        end else if (state == STATE_RW) begin
            state <= (spi_mosi ? STATE_WRITE_DAT : STATE_READ_WB);
        end else if (state == STATE_WRITE_DAT) begin
            req_data[bit_cnt[3:0]] <= spi_mosi; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd15) begin
                state <= STATE_WRITE_WB;
                bit_cnt <= 5'b0;
            end
        end else if (state == STATE_WRITE_WB) begin
            wb_cyc <= 1'b1;
            wb_stb <= 1'b1;
            wb_we <= 1'b1;

            wb_adr <= req_addr;
            wb_o_dat <= req_data;
            
            if (wb_ack | wb_err) begin
                state <= STATE_WB_RESP;
                bit_cnt <= 5'b0;

                resp_err <= wb_err;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                
                spi_miso <= 1'b0; // end of wait
            end
        end else if (state == STATE_WB_RESP) begin
            spi_miso <= resp_err;
            state <= STATE_IDLE;
        end else if (state == STATE_READ_WB) begin
            wb_cyc <= 1'b1;
            wb_stb <= 1'b1;
            wb_we <= 1'b0;
            wb_adr <= req_addr;

            if (wb_ack | wb_err) begin
                state <= STATE_READ_DAT;
                bit_cnt <= 5'b0;

                res_data <= wb_i_dat;
                resp_err <= wb_err;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                
                spi_miso <= 1'b0; // end of wait
            end
        end else if (state == STATE_READ_DAT) begin
            spi_miso <= res_data[bit_cnt[3:0]]; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd15) begin
                state <= STATE_WB_RESP;
                bit_cnt <= 5'b0;
            end
        end
    end
end

assign wb_sel = 2'b11;

endmodule