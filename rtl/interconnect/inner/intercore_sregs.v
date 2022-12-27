`include "config.v"

`define CORES 2

module intercore_sregs (
    input i_clk,
    input i_rst,

    input [`RW-1:0] c0_sr_bus_addr,
    input [`RW-1:0] c0_sr_bus_data_o,
    output [`RW-1:0] c0_sr_bus_data_i,
    input c0_sr_bus_we,

    input [`RW-1:0] c1_sr_bus_addr,
    input [`RW-1:0] c1_sr_bus_data_o,
    output [`RW-1:0] c1_sr_bus_data_i,
    input c1_sr_bus_we,

    output c0_disable,
    output c0_core_int,

    output c1_disable,
    output c1_core_int
);

`define SREG_ICINT_SET `RW'b1001
`define SREG_ICINT_RESET `RW'b1010
`define SREG_ICDISABLE `RW'b1011

wire [`CORES-1:0] ic_irq_set_read_w;
assign ic_irq_set_read_w[0] = (c0_sr_bus_addr == `SREG_ICINT_SET) && c0_sr_bus_we;
assign ic_irq_set_read_w[1] = (c1_sr_bus_addr == `SREG_ICINT_SET) && c1_sr_bus_we;

wire [`CORES-1:0] ic_irq_reset_w;
assign ic_irq_reset_w[0] = (c0_sr_bus_addr == `SREG_ICINT_RESET) && c0_sr_bus_we;
assign ic_irq_reset_w[1] = (c1_sr_bus_addr == `SREG_ICINT_RESET) && c1_sr_bus_we;

wire [`RW-1:0] sr_bus_dat [`CORES-1:0];
assign sr_bus_dat[0] = c0_sr_bus_data_o;
assign sr_bus_dat[1] = c1_sr_bus_data_o;

assign c0_sr_bus_data_i = {14'b0, ic_irq_state};
assign c1_sr_bus_data_i = {14'b0, ic_irq_state};

reg [`CORES-1:0] ic_irq_state;

always @(posedge i_clk) begin
    if (i_rst) begin
        ic_irq_state = 2'b0;
    end else begin
        if (ic_irq_reset_w[0])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_reset_w[1])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[1][`CORES-1:0];
        
        if (ic_irq_set_read_w[0])
            ic_irq_state = ic_irq_state | sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_set_read_w[1])
            ic_irq_state = ic_irq_state | sr_bus_dat[1][`CORES-1:0];
    end
end

wire c0_core_hold_we = (c0_sr_bus_addr == `SREG_ICDISABLE) && c0_sr_bus_we;

assign c0_disable = 1'b0;
assign c0_core_int = ic_irq_state[0];
assign c1_core_int = ic_irq_state[1];

reg core1_disable;
assign c1_disable = core1_disable;
always @(posedge i_clk) begin
    if (i_rst) begin
        core1_disable <= 1'b1;
    end else begin
        if (c0_core_hold_we)
            core1_disable <= sr_bus_dat[0][1];
    end
end

endmodule