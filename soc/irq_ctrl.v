// Interrrupt controller
// Takes multiple edge triggered interrupts as inputs (each able to clear and disable)
// and outputs one master level interrupt

module irq_ctrl (
    input i_clk,
    input i_rst,

    output o_irq,

    input [15:0] i_irq,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output [15:0] wb_o_dat
);

assign o_irq = |(irq_active & irq_mask);

reg [15:0] irq_active;
reg [15:0] irq_mask;

always @(posedge i_clk) begin
    if (i_rst) begin
        irq_active = 16'b0;
        irq_mask = 16'h0000;
    end else begin
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'b1)
            irq_active = irq_active & ~wb_i_dat;
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'b10)
            irq_mask = wb_i_dat;
        
        irq_active = irq_active | ((prev_i_irq ^ i_irq) & i_irq);
    end
end

reg [15:0] prev_i_irq;
always @(posedge i_clk) begin
    if (i_rst)
        prev_i_irq <= 16'b0;
    else
        prev_i_irq <= i_irq;
end

assign wb_ack = wb_cyc & wb_stb;
assign wb_o_dat = (wb_adr == 24'b10 ? irq_mask : irq_active & irq_mask); 

endmodule