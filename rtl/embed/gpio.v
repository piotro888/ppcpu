`include "config.v"

module gpio #(parameter N = 4) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat,

    output reg [N-1:0] gpio_out,
    input [N-1:0] gpio_in,
    output reg [N-1:0] gpio_dir
);

localparam GPIO_READ = `WB_ADDR_W'h001010;
localparam GPIO_WRITE = `WB_ADDR_W'h001011;
localparam GPIO_DIR = `WB_ADDR_W'h001012;

always @(posedge i_clk) begin
    if (i_rst) begin
        gpio_dir <= {N{1'b1}}; // default inputs
        gpio_out <= {N{1'b0}};
    end else if (wb_cyc & wb_stb & wb_we & (wb_adr == GPIO_WRITE)) begin
        gpio_out <= wb_i_dat[N-1:0];
    end else if (wb_cyc & wb_stb & wb_we & (wb_adr == GPIO_DIR)) begin
        gpio_dir <= wb_i_dat[N-1:0];
    end
end

always @* begin
    wb_o_dat = 16'b0;
    case (wb_adr)
        default: 
            wb_o_dat[N-1:0] = gpio_in;
        GPIO_WRITE:
            wb_o_dat[N-1:0] = gpio_out;
        GPIO_DIR:
            wb_o_dat[N-1:0] = gpio_dir; 
    endcase
end
assign wb_ack = wb_cyc & wb_stb;

endmodule