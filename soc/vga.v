module vga (
	input wire raw_clk,
	input wire cpu_clk,

	output reg hsync, vsync,
	output reg [2:0] r, g,
	output reg [1:0] b,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat
);

reg pclk;

initial begin
	pclk = 0;
	r = 0;
	g = 0;
	b = 0;
end

reg [14:0] addrb;
wire [15:0]  qb;

vgram vgram(
	wb_i_dat,
	addrb,
	raw_clk,
	wb_adr,
	cpu_clk,
	wb_stb & wb_cyc & wb_we,
	qb
);
assign wb_ack = wb_stb & wb_cyc & wb_we;

always @(posedge raw_clk) begin
	pclk <= ~pclk;
end

reg [10:0] hcnt, vcnt;

reg [7:0] vga_config = 8'b01;
reg [7:0] fast_scroll_line = 8'b0;
reg [7:0] x_char_pos = 8'b0, y_char_pos = 8'b0;

wire [60:0] char_rom_out;
wire [8:0] char_rom_in = qb[7:0];

charrom charom(
	char_rom_in,
	~raw_clk,
	char_rom_out
);

reg [7:0] char_bit;

always @(posedge pclk) begin
	if(hcnt >= 640 || vcnt >= 480) begin 
		r = 3'b000;
		g = 3'b000;
		b = 2'b00;
	end else if(vga_config[3:0] == 4'b0001) begin // text mode [106x48] (char 6*10)
		if(hcnt < 636) begin // skip 107th char (cropped)
			char_bit = x_char_pos + y_char_pos*6;
			if(char_rom_out[char_bit]) begin
				r = qb[8] ? (qb[11] ?  3'b111 :3'b011) : 3'b000;
				g = qb[9] ? (qb[11] ?  3'b111 :3'b011) : 3'b000;
				b = qb[10] ? (qb[11] ?  2'b11 :2'b10) : 2'b00;
			end else begin
				r = qb[12] ? (qb[15] ?  3'b111 :3'b011) : 3'b000;
				g = qb[13] ? (qb[15] ?  3'b111 :3'b011) : 3'b000;
				b = qb[14] ? (qb[15] ?  2'b11 :2'b10) : 2'b000;
			end
			if(x_char_pos == 5) begin
				addrb = addrb+1;
				x_char_pos = 0;
			end else 
				x_char_pos = x_char_pos+1;
		end else begin
			r = 3'b000;
			g = 3'b000;
			b = 2'b00;
		end
	end else begin // 8-bit color 160x120
		if(hcnt[2]) begin 
			r = qb[10:8];
			g = qb[13:11];
			b = qb[15:14];
		end else begin
			r = qb[2:0];
			g = qb[5:3];
			b = qb[7:6];
		end
		if (&hcnt[2:0])
			addrb = addrb+1;
	end
	
	if(hcnt >= 656 && hcnt <= 752)
		hsync = 1'b0;
	else
		hsync = 1'b1;
	
	if(vcnt >= 490 && vcnt <= 492)
		vsync = 1'b0;
	else
		vsync = 1'b1;
	
	hcnt = hcnt+1;
	if(hcnt == 801) begin
		hcnt = 0;
		if (vga_config[3:0] == 4'b0001) begin
			if(y_char_pos == 9) begin
				y_char_pos = 0;
				x_char_pos = 0;
			end else begin
				x_char_pos = 0;
				y_char_pos = y_char_pos+1;
				addrb = addrb-106;
			end
		end else if((~(&vcnt[1:0])) && vcnt <= 480) 
			addrb = addrb - 80;
		vcnt = vcnt+1; 
	end
	if(vcnt > 480) begin
		if(vga_config[3:0] == 4'b0001)
			addrb = fast_scroll_line*106;
		else
			addrb = 0;
	
		x_char_pos = 0;
		y_char_pos = 0;
	end
	if(vcnt == 526) begin
		vcnt = 0;
	end
	if(vga_config[3:0] == 4'b0001 && addrb >= 106*48 && y_char_pos >= 9) begin
		addrb = 0;
	end
end

always @(posedge cpu_clk) begin
	if(wb_stb & wb_cyc & wb_we) begin
		if(wb_adr == 15'h3000) begin // write to settings reg at address 0x4000
			vga_config <= wb_i_dat[7:0];
		end else if(wb_adr == 15'h3001) begin
			fast_scroll_line <= wb_i_dat[7:0];
		end
	end
end

endmodule
