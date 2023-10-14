module ps2_kbd (
    input wire clk,
    
    input wire ps_clk,
    input wire ps_data,

    output reg [7:0] scancode_sync,
    output reg irq_sync
);

reg [7:0] scancode;
reg irq;

initial irq = 1'b0;
initial scancode = 8'hFF;
reg [3:0] bit = 4'b0;
reg [10:0] code_shift;
 
wire db_clk;
wire db_dat;
debouncer clkdb(clk, ps_clk, db_clk);
debouncer datdb(clk, ps_data, db_dat);

always @(negedge db_clk) begin
    code_shift = {db_dat, code_shift[10:1]};
end

wire frame_valid = (~code_shift[0] & code_shift[10] & (^code_shift[9:1]));

// ~800 us -> 20ns*40000 -> 15b / 1us*800 -> 10b
reg [14:0] frame_time; // +3 idx speed gain
reg [1:0] irq_time;
always @(posedge clk) begin
    if(|frame_time || (~(|frame_time) && db_clk == 1'b0))
        frame_time <= frame_time + 10'b1;

    if(&frame_time)
        if(frame_valid) begin
            scancode <= code_shift[9:1];
            irq <= 1'b1;
        end

    if(irq)
        irq_time <= irq_time + 4'b1;

    if(&irq_time) begin
        irq_time <= 4'b0;
        irq <= 1'b0;
    end
end

always @(posedge clk) begin
    scancode_sync <= scancode;
    irq_sync <= irq;
end

endmodule

module debouncer (
    input wire clk,
    input wire in,
    output reg out
);
initial out = 1'b0;

// clock syncing
reg in_sync_0;  always @(posedge clk) in_sync_0 <= in;
reg in_sync_1;  always @(posedge clk) in_sync_1 <= in_sync_0;

// debouncing
reg [3:0] state_time = 4'b0;
always @(posedge clk) begin
    if(in_sync_1 != out) begin
        state_time <= state_time + 4'b1;
        if(&state_time)
            out <= ~out;
    end else
        state_time <= 4'b0;
end

endmodule