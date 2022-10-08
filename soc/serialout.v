module serialout (
    input wire clk,
    input wire [7:0] data,
    output wire sclk,
    output reg sdata,

	input wire sdatain,
	output wire sdata_pl,
	output reg [7:0] btout
);

reg[22:0] clk_cnt = 0;
always @(posedge clk) begin
    clk_cnt <= clk_cnt+1;
end

reg [3:0] ser_bit = 4'b0;

wire ref_clk = clk_cnt[20];
wire ser_clk = clk_cnt[10];

reg tx = 0, rt = 0;

assign sclk = ser_clk & tx;
assign sdata_pl = tx;

initial btout <= 8'b0;

always @(negedge ser_clk) begin
    case (ser_bit) 
		0: begin
			sdata <= data[0];
			ser_bit <= 1;
			tx <= 1;
			btout[0] <= sdatain;
		end
		1: begin
			sdata <= data[1];
			ser_bit <= 2;
			btout[1] <= sdatain;
		end
		2: begin
			sdata <= data[2];
			ser_bit <= 3;
			btout[2] <= sdatain;
		end
		3: begin
			sdata <= data[3];
			ser_bit <= 4;
			btout[3] <= sdatain;
		end
		4: begin
			sdata <= data[4];
			ser_bit <= 5;
			btout[4] <= sdatain;
		end
		5: begin
			sdata <= data[5];
			ser_bit <= 6;
			btout[5] <= sdatain;
		end
		6: begin
			sdata <= data[6];
			ser_bit <= 7;
			btout[6] <= sdatain;
		end
		7: begin
			sdata <= data[7];
			ser_bit <= 8;
			btout[7] <= sdatain;
		end
		8: begin
			tx <= 0;
			if (ref_clk && ~rt) begin
				ser_bit <= 0;
				rt <= 1;
			end else if (~ref_clk) begin
				rt <= 0;
			end
		end
	endcase
    
end

endmodule