`include "config.v"

module wb_cross_clk (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input clk_m,
    input clk_s,
    input m_rst,
    input s_rst,

    input m_wb_cyc,
    input m_wb_stb,
    input [`WB_ADDR_W-1:0] m_wb_adr,
    input [`RW-1:0] m_wb_o_dat,
    output [`RW-1:0] m_wb_i_dat,
    input m_wb_we,
    input [1:0] m_wb_sel,
    input m_wb_8_burst, m_wb_4_burst,
    output m_wb_ack,
    output m_wb_err,

    output s_wb_cyc,
    output s_wb_stb,
    output [`WB_ADDR_W-1:0] s_wb_adr,
    output [`RW-1:0] s_wb_o_dat,
    input [`RW-1:0] s_wb_i_dat,
    output s_wb_we,
    output [1:0] s_wb_sel,
    output s_wb_8_burst, s_wb_4_burst,
    input s_wb_ack,
    input s_wb_err
);

reg prev_stb, prev_ack;
always @(posedge clk_m) begin
    if (m_rst) begin
        prev_stb <= 1'b0;
        prev_ack <= 1'b0;
    end else begin
        prev_stb <= m_wb_stb;
        prev_ack <= (msy_flag_ack |  msy_flag_err);
    end
end
wire m_new_req = m_wb_stb & (~prev_stb | prev_ack) & ~ignored_addr;
wire ignored_addr = m_wb_adr < `WB_ADDR_W'h002000;

`define MAX_BURST_LOG 4
reg [`MAX_BURST_LOG-1:0] m_burst_cnt;

reg m_new_req_flag;
always @(posedge clk_m) begin
    if (m_rst) begin
        m_new_req_flag <= 1'b0;
        m_burst_cnt <= `MAX_BURST_LOG'b0;
    end else if (m_new_req & ~(|m_burst_cnt)) begin
        m_new_req_flag <= ~m_new_req_flag;
        if (m_wb_we)
            m_burst_cnt <= `MAX_BURST_LOG'd1; // burst write is not possible, as new data must be transmitted after each ack, which causes delay
                                              // although burst signal is passed for later optimization of compressed bus write
        else
            m_burst_cnt <= (m_wb_8_burst ? `MAX_BURST_LOG'd8 : (m_wb_4_burst ? `MAX_BURST_LOG'd4 : `MAX_BURST_LOG'd1));
    end else if (msy_flag_ack | msy_flag_err) begin
        m_burst_cnt <= m_burst_cnt - `MAX_BURST_LOG'b1;
    end
end

reg ack_next_hold;
reg [`MAX_BURST_LOG-1:0] s_burst_cnt;
wire burst_in_progress = (|s_burst_cnt);

always @(posedge clk_s) begin
    if (s_rst) begin
        ack_next_hold <= 1'b0;
        s_burst_cnt <= `MAX_BURST_LOG'b0;
    end else if ((s_wb_ack | s_wb_err) & burst_in_progress) begin
        s_burst_cnt <= s_burst_cnt - `MAX_BURST_LOG'b1;
        ack_next_hold <= 1'b0;
    end else if ((s_wb_ack | s_wb_err) & ~burst_in_progress) begin
        ack_next_hold <= 1'b1;
    end else if (ssy_newreq) begin
        ack_next_hold <= 1'b0;
        if (m_wb_we)
            s_burst_cnt <= 'b0;
        else
            s_burst_cnt <= (s_wb_8_burst ? `MAX_BURST_LOG'd7 : (s_wb_4_burst ? `MAX_BURST_LOG'd3 : `MAX_BURST_LOG'd0));
    end
end

assign s_wb_stb = s_wb_cyc & ~ack_next_hold;

// XOR strobe setup
reg ack_xor_flag;
reg err_xor_flag;
always @(posedge clk_s) begin
    if(s_rst) begin
        ack_xor_flag <= 1'b0;
        err_xor_flag <= 1'b0;
    end else begin
        if(s_wb_ack)
            ack_xor_flag <= ~ack_xor_flag;
        if(s_wb_err)
            err_xor_flag <= ~err_xor_flag;
    end
end

reg prev_xor_ack, prev_xor_err;
always @(posedge clk_m) begin
    if(m_rst) begin
        prev_xor_ack <= 1'b0;
        prev_xor_err <= 1'b0;
    end else begin
        prev_xor_ack <= msy_xor_ack;
        prev_xor_err <= msy_xor_err;
    end
end
wire msy_flag_ack = prev_xor_ack ^ msy_xor_ack;
wire msy_flag_err = prev_xor_err ^ msy_xor_err;

reg prev_xor_newreq;
always @(posedge clk_s) begin
    if(s_rst)
        prev_xor_newreq <= 1'b0;
    else
        prev_xor_newreq <= ssy_flag_newreq; 
end
wire ssy_newreq = prev_xor_newreq ^ ssy_flag_newreq;

// S->M ff sync
`define SM_SYNC_W 2+16
wire [`SM_SYNC_W-1:0] smsync1;
ff_mb_sync #(.DATA_W(`SM_SYNC_W)) s_m_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .src_clk(clk_s),
    .dst_clk(clk_m),
    .src_rst(s_rst),
    .dst_rst(m_rst),
    .i_data({s_wb_i_dat, err_xor_flag^s_wb_err, ack_xor_flag^s_wb_ack}),
    .o_data(smsync1),
    .i_xfer_req(s_wb_ack | s_wb_err) // NOTE: CLOCK MUST BE DIVIDED BY >4 TO NOT VIOLATE 3 CYCLE DELAY (to dst clock) 
);

assign m_wb_i_dat = smsync1[17:2];
assign m_wb_ack = msy_flag_ack; 
assign m_wb_err = msy_flag_err; 

wire msy_xor_ack = smsync1[0];
wire msy_xor_err = smsync1[1];

// M->S ff sync
`define MS_SYNC_W 1+24+16+1+2+2+1
wire [`MS_SYNC_W-1:0] mssync1;

ff_mb_sync #(.DATA_W(`MS_SYNC_W)) m_s_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .src_clk(clk_m),
    .dst_clk(clk_s),
    .src_rst(m_rst),
    .dst_rst(s_rst),
    .i_data({m_wb_cyc, m_wb_adr, m_wb_o_dat, m_wb_we, m_wb_sel, m_wb_4_burst, m_wb_8_burst, ~m_new_req_flag}),
    .o_data(mssync1),
    .i_xfer_req(m_new_req & ~(|m_burst_cnt))
);

wire ssy_flag_newreq = mssync1[0]; 
assign s_wb_8_burst = mssync1[1];
assign s_wb_4_burst = mssync1[2];
assign s_wb_sel = mssync1[4:3];
assign s_wb_we = mssync1[5];
assign s_wb_o_dat = mssync1[21:6];
assign s_wb_adr = mssync1[45:22];
assign s_wb_cyc = mssync1[46];

endmodule
