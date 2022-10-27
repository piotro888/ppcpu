// SDRAM driver from pcpu repo

module sdram (
    input wire clk,
    // CPU
    input wire [23:0] c_addr,
    input wire [15:0] c_data_in,
    input wire [1:0] c_addr_sel,
    output reg [31:0] c_data_out,
    input wire c_read_req, c_write_req,
    output reg c_busy, c_read_ready, c_cack,
    // SDRAM
    output reg dr_dqml, dr_dqmh,
    output wire dr_cs_n, dr_cas_n, dr_ras_n, dr_we_n, dr_cke,
    output reg [1:0] dr_ba,
    output reg  [12:0] dr_a,
    inout [15:0] dr_dq,
   input wire srclk
);

localparam CMD_NOP = 3'b111;
localparam CMD_ACTIVE = 3'b011;
localparam CMD_READ = 3'b101;
localparam CMD_WRITE = 3'b100;
localparam CMD_PRECH = 3'b010;
localparam CMD_AREFR = 3'b001;
localparam CMD_LREG = 3'b000;

reg [2:0] ram_cmd = CMD_NOP;
assign {dr_ras_n, dr_cas_n, dr_we_n} = ram_cmd;
assign dr_cke = 1'b1;
assign dr_cs_n = 1'b0;

localparam STATE_INIT_BEGIN = 4'b0000;
localparam STATE_INIT_PRECALL = 4'b0001;
localparam STATE_INIT_AUTOREF1 = 4'b0010;
localparam STATE_INIT_AUTOREF2 = 4'b0011;
localparam STATE_INIT_REGPROG = 4'b0100;
localparam STATE_IDLE = 4'b0101;
localparam STATE_REFR = 4'b0110;
localparam STATE_READ = 4'b0111;
localparam STATE_CASREAD = 4'b1000;
localparam STATE_WRITE = 4'b1001;
localparam STATE_WAIT = 4'b1111;
reg [3:0] state = STATE_INIT_BEGIN;

reg [15:0]  wait_reg;
reg [3:0]  wait_next_state;

reg [15:0] dr_dq_reg;
reg dr_dq_oe = 1'b0;
assign dr_dq = (dr_dq_oe ? dr_dq_reg : 16'bz);

// refr 7.183 us -> 355 clock -8/10/15 PRECHARGE ALL BEFORE RESET
reg [8:0] autorefr_cnt = 9'd55;

initial c_busy = 1'b1;
initial c_read_ready = 1'b0;

reg [23:0] i_addr;
reg [15:0] i_data_in;
reg [1:0] i_wr_mask;

reg [23:0] c_r_addr, c_w_addr; // to save addr at edge of bus

reg sync_xory_read = 1'b0;
always @(posedge srclk) begin
   if(c_read_req & ~c_busy) begin
      sync_xory_read <= sync_xory_read^1;
        c_r_addr <= c_addr;
    end
end
reg sync_x_read_done = 1'b0;
wire pulse_c_read = (sync_xory_read ^ sync_x_read_done) & ~c_read_ready;

reg sync_xory_write = 1'b0;
always @(posedge srclk) begin
   if(c_write_req & ~c_busy) begin
      sync_xory_write <= sync_xory_write^1;
        c_w_addr <= c_addr;
    end
end

always @(posedge srclk) begin
    c_cack <= 1'b0;
    if((c_read_req & ~c_busy) | (c_write_req & ~c_busy)) begin
        c_cack <= 1'b1;
    end
end
reg sync_x_write_done = 1'b0;
wire pulse_c_write = sync_xory_write ^ sync_x_write_done;

reg prev_srclk = 1'b0;
// 50 Mhz -> 20 ns
// RP 18ns RFC 60ns
always @(posedge clk) begin
    //if (state != STATE_WAIT) begin
        {dr_dqml, dr_dqmh} <= 2'b00; dr_dq_oe <= 1'b0; dr_a <= 13'b0; dr_ba <= 2'b0;
    //end

    prev_srclk <= srclk;
    if((prev_srclk ^ srclk) & srclk) // THIS WAS THE ISSUE!!!! READ_REDY WAS NOT RESET!!!
      c_read_ready <= 1'b0;
    
    case (state)
        STATE_INIT_BEGIN: begin
            ram_cmd <= CMD_NOP;
            state <= STATE_WAIT;
            wait_next_state <= STATE_INIT_PRECALL;
            //wait_reg <= 16'd5000; 
            wait_reg <= 16'd10; 
        end
        STATE_INIT_PRECALL: begin
            ram_cmd <= CMD_PRECH;
            dr_a[10] <= 1'b1; // all banks
            state <= STATE_WAIT;
            wait_next_state <= STATE_INIT_AUTOREF1;
            wait_reg <= 16'd1;
        end
        STATE_INIT_AUTOREF1: begin
            ram_cmd <= CMD_AREFR;
            state <= STATE_WAIT;
            wait_next_state  <= STATE_INIT_AUTOREF2;
            wait_reg <= 16'd4; // 80 ns
        end
        STATE_INIT_AUTOREF2: begin
            ram_cmd <= CMD_AREFR;
            state <= STATE_WAIT;
            wait_next_state  <= STATE_INIT_REGPROG;
            wait_reg <= 16'd4; // 80 ns
        end
        STATE_INIT_REGPROG: begin
            ram_cmd <= CMD_LREG;
            dr_a <= 13'b0001000100000;
            //dr_a <= 13'b0001000010000;
            // CAS 2 BURST R1 W1 SEQ
            // CAS 1 CORRECT? DATASHEET
            dr_ba <= 2'b00;
            state <= STATE_WAIT;
            wait_next_state  <= STATE_IDLE;
            wait_reg <= 16'd4; // 80 ns
        end
        STATE_IDLE: begin
           // c_busy <= 1'b1; //default if not staying in idle
            if(pulse_c_read) begin
                ram_cmd <= CMD_ACTIVE; //CHECK IF NOT ACTIVATED ALREADY
                //STORE PROG AND DATA IN DIFFERENT BANKS TO NOT PRECHARGE EVERY COMMAND
                //8192 rows x 512 col x 16 bit x 4 banks
                // 13 b     +  9 b    + 16 b   + 2b
                i_addr <= c_r_addr;
                i_data_in <= c_data_in;
                dr_ba <= {c_r_addr[23:22]}; // lower max ram addr and set banks to msb???
                dr_a <= c_r_addr[21:9];
                state <= STATE_WAIT;
                wait_next_state <= STATE_READ;
                wait_reg <= 16'd1;
                c_busy <= 1'b1;
                sync_x_read_done <= sync_x_read_done^1;
            end else if(pulse_c_write) begin
                ram_cmd <= CMD_ACTIVE;
                i_addr <= c_w_addr;
                i_data_in <= c_data_in;
                i_wr_mask <= ~c_addr_sel;
                dr_ba <= {c_w_addr[23:22]};
                dr_a <= c_w_addr[21:9];
                state <= STATE_WAIT;
                wait_next_state <= STATE_WRITE;
                wait_reg <= 16'd1;
                c_busy <= 1'b1;
                sync_x_write_done <= sync_x_write_done^1;
            end else if(~(|autorefr_cnt)) begin
                ram_cmd <= CMD_PRECH;
                dr_a[10] <= 1'b1; //ALL BANKS
                state <= STATE_WAIT;
                wait_next_state <= STATE_REFR;
                wait_reg <= 16'd1;
                c_busy <= 1'b1;
            end else begin
                ram_cmd <= CMD_NOP;
                state <= STATE_IDLE;
                c_busy <= 1'b0;
            end
        end
        STATE_WRITE: begin
            ram_cmd <= CMD_WRITE;
            {dr_dqmh, dr_dqml} <= i_wr_mask;
            dr_ba <= {i_addr[23:22]};
            dr_a[8:0] <= i_addr[8:0];
            dr_a[9] <= 1'b0; dr_a[12:11] <= 2'b0;
            dr_a[10] <= 1'b1; //auto precharge
            dr_dq_reg <= i_data_in;
            dr_dq_oe <= 1'b1;
            state <= STATE_WAIT;
            wait_next_state <= STATE_IDLE;
            wait_reg <= 16'd1;
        end
        STATE_REFR: begin
            ram_cmd <= CMD_AREFR;
            state <= STATE_WAIT;
            wait_next_state <= STATE_IDLE;
            wait_reg <= 16'd4;
            autorefr_cnt <= 9'd355;
        end
        STATE_READ: begin
            ram_cmd <= CMD_READ;
            {dr_dqml, dr_dqmh} <= 2'b00;
            //don't use burst but two subseq reads for instr
            dr_ba <= {i_addr[23:22]};
            dr_a[8:0] <= i_addr[8:0];
            dr_a[9] <= 1'b0; dr_a[12:11] <= 2'b0;
            dr_a[10] <= 1'b1; //auto precharge
            state <= STATE_WAIT;
            wait_next_state <= STATE_CASREAD;
            wait_reg <= 16'd1;
        end
        STATE_CASREAD: begin
            state <= STATE_IDLE; //no wait needed
            ram_cmd <= CMD_NOP;
            c_data_out <= {16'b0, dr_dq};
            c_read_ready <= 1'b1;
            c_busy <= 1'b0;
        end
        default: begin // STATE_WAIT
            ram_cmd <= CMD_NOP;
            if(wait_reg == 16'b01) begin
                state <= wait_next_state;
                if (wait_next_state == STATE_IDLE) c_busy <= 1'b0;
                else c_busy <= 1'b1;
            end
            wait_reg <= wait_reg-1;
        end
    endcase

    if(|autorefr_cnt) autorefr_cnt <= autorefr_cnt-1;
end
    
endmodule