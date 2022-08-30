`include "config.v"

module dcache (
    input i_clk,
    input i_rst,

    input mem_req,
    input mem_we,
    output mem_ack,
    input [`RW-1:0] mem_addr,
    input [`RW-1:0] mem_i_data,
    output reg [`RW-1:0] mem_o_data,
    input mem_cache_flush,
    input [1:0] mem_sel,

    // output interface
    output reg wb_cyc,
    output reg wb_stb,
    input [`RW-1:0] wb_i_dat,
    output [`RW-1:0] wb_o_dat,
    output [`RW-1:0]  wb_adr,
    output reg wb_we,
    output [1:0] wb_sel,
    input wb_ack

    // TODO: Multicore MSI cache protocol
);

`define TAG_SIZE 7
`define LINE_SIZE 64
// 7b tag + 64b line + 2b valid dirty
`define ENTRY_SIZE 73
`define CACHE_ASSOC 4
`define CACHE_ASSOC_W 2
`define CACHE_ENTR_N 64
`define CACHE_OFF_W 2

`define CACHE_IDX_WIDTH 6
`define CACHE_IDXES 64

`define VALID_BIT 0
`define DIRTY_BIT 1

`define SW 3
`define S_IDLE `SW'b0
`define S_CREAD `SW'b1
`define S_MISS_RD `SW'b10
`define S_MISS_WR `SW'b11
`define S_RQ_WR `SW'b100
`define S_RQ_WR_WAIT `SW'b101
reg [`SW-1:0] state;

wire [`ENTRY_SIZE-1:0] cache_mem_in;
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
wire [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

wire [`CACHE_IDX_WIDTH-1:0] cache_idx = mem_addr[7:2];
wire [`CACHE_OFF_W-1:0] cache_offset = mem_addr[1:0];
wire [`TAG_SIZE-1:0] cache_compare_tag = mem_addr[14:8];

genvar i;
generate
    for (i=0; i<`CACHE_ASSOC; i=i+1) begin : cache_mem
        dcache_mem #(.AW(`CACHE_IDX_WIDTH), .AS(`CACHE_IDXES), .DW(`ENTRY_SIZE)) mem (
            .i_clk(i_clk), .i_rst(i_rst | mem_cache_flush), .i_addr(cache_idx), .i_data(cache_mem_in),
            .o_data(cache_out[i]), .i_we(cache_we[i]));
        assign cache_hit[i] = (cache_out[i][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == cache_compare_tag) && cache_out[i][`VALID_BIT]; 
    end
endgenerate

wire cache_ghit = (state == `S_CREAD) && (|cache_hit);
wire cache_gmiss = (state == `S_CREAD) && ~(|cache_hit);

wire mem_op_end = wb_cyc & wb_stb & wb_ack & (&line_burst_cnt);
wire mem_fetch_end = (state == `S_MISS_RD) && mem_op_end;
wire write_to_cache = (state == `S_RQ_WR);

assign mem_ack = (state == `S_CREAD && ~mem_we && cache_ghit) | (mem_fetch_end && ~mem_we) | (state == `S_RQ_WR);

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
    end else if (state == `S_IDLE && mem_req) begin
        state <= `S_CREAD;
    end else if (state == `S_CREAD && cache_ghit && ~mem_we) begin
        state <= `S_IDLE;
    end else if (state == `S_CREAD && cache_ghit && mem_we) begin
        state <= `S_RQ_WR;
    end else if (state == `S_CREAD && cache_gmiss && ~entry_dirty) begin
        state <= `S_MISS_RD;
    end else if (state == `S_CREAD && cache_gmiss && entry_dirty) begin
        state <= `S_MISS_WR;
    end else if (state == `S_MISS_RD && mem_fetch_end && ~mem_we) begin
        state <= `S_IDLE;
    end else if (state == `S_MISS_RD && mem_fetch_end && mem_we) begin
        state <= `S_RQ_WR_WAIT;
    end else if (state == `S_RQ_WR_WAIT) begin
        state <= `S_RQ_WR;
    end else if (state == `S_MISS_WR && mem_op_end) begin
        state <= `S_MISS_RD;
    end else if (state == `S_RQ_WR) begin
        state <= `S_IDLE;
    end
end

wire wb_sel_adr_source = (state == `S_MISS_WR) | (state == `S_CREAD && cache_gmiss && entry_dirty);

assign wb_adr = (wb_sel_adr_source ? {old_entry_addr[15:2], line_burst_cnt} : {1'b0, mem_addr[14:2], line_burst_cnt});
assign wb_sel = 2'b11;

wire [`CACHE_ASSOC_W-1:0] write_sel_idx = 2'b0;

assign cache_we[0] = mem_fetch_end | write_to_cache;
assign cache_mem_in = (write_to_cache ? cache_update_entry : {cache_compare_tag, pre_assembled_line, 2'b01});

wire entry_dirty = &cache_out[write_sel_idx][`DIRTY_BIT:`VALID_BIT];
wire [`RW-1:0] old_entry_addr = {1'b0, write_source_entry[`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE], mem_addr[`CACHE_OFF_W+`CACHE_IDX_WIDTH-1:0]};

reg [`LINE_SIZE-1:0] line_collect;
wire [`LINE_SIZE-1:0] pre_assembled_line = {wb_i_dat, line_collect[47:0]};

reg [`CACHE_OFF_W-1:0] line_burst_cnt;
always @(posedge i_clk) begin
    if (i_rst) begin
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
    end else if(cache_gmiss || (state == `S_MISS_WR && mem_op_end)) begin
        line_burst_cnt <= 2'b0;
        wb_cyc <= 1'b1;
        wb_stb <= 1'b1;
        wb_we <= entry_dirty & ~mem_op_end;
    end else if (mem_op_end) begin
        line_burst_cnt <= 2'b0;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
    end else if (wb_cyc & wb_stb & wb_ack) begin
        line_burst_cnt <= line_burst_cnt + 1'b1;
    end
end

wire [`ENTRY_SIZE-1:0] write_source_entry;
always @(posedge i_clk) begin
    case (line_burst_cnt)
        default: line_collect[15:0] <= wb_i_dat;
        2'b01: line_collect[31:16] <= wb_i_dat;
        2'b10: line_collect[47:32] <= wb_i_dat;
        2'b11: line_collect[63:48] <= wb_i_dat;
    endcase
end

reg [`ENTRY_SIZE-1:0] cache_hit_entry;
always @* begin
    case (cache_hit)
        default: cache_hit_entry = cache_out[0];
        4'b0010: cache_hit_entry = cache_out[1];
        4'b0100: cache_hit_entry = cache_out[2];
        4'b1000: cache_hit_entry = cache_out[3];
    endcase
end

wire [`ENTRY_SIZE-1:0] entry_out = (mem_fetch_end ? {`TAG_SIZE'b0, pre_assembled_line, 2'b00} : cache_hit_entry);

always @* begin
    case (cache_offset)
        default: mem_o_data = entry_out[17:2];
        2'b01: mem_o_data = entry_out[33:18];
        2'b10: mem_o_data = entry_out[49:34];
        2'b11: mem_o_data = entry_out[65:50];
    endcase
end

wire [`ENTRY_SIZE-1:0] write_source_entry = cache_out[write_sel_idx];
always @* begin
    case (line_burst_cnt)
        default: wb_o_dat = write_source_entry[17:2];
        2'b01: wb_o_dat = write_source_entry[33:18];
        2'b10: wb_o_dat = write_source_entry[49:34];
        2'b11: wb_o_dat = write_source_entry[65:50];
    endcase
end

reg [`ENTRY_SIZE-1:0] cache_update_entry;
always @* begin
    case ({mem_addr[1:0], mem_sel})
        default: cache_update_entry = {write_source_entry[72:18], mem_i_data, 2'b11};
        4'b0001: cache_update_entry = {write_source_entry[72:10], mem_i_data[7:0], 2'b11};
        4'b0010: cache_update_entry = {write_source_entry[72:18], mem_i_data[15:8], write_source_entry[9:2], 2'b11};
        4'b0111: cache_update_entry = {write_source_entry[72:34], mem_i_data, write_source_entry[17:2], 2'b11};
        4'b0101: cache_update_entry = {write_source_entry[72:26], mem_i_data[7:0], write_source_entry[17:2], 2'b11};
        4'b0110: cache_update_entry = {write_source_entry[72:34], mem_i_data[15:8], write_source_entry[25:2], 2'b11};
        4'b1011: cache_update_entry = {write_source_entry[72:50], mem_i_data, write_source_entry[33:2], 2'b11};
        4'b1001: cache_update_entry = {write_source_entry[72:42], mem_i_data[7:0], write_source_entry[33:2], 2'b11};
        4'b1010: cache_update_entry = {write_source_entry[72:50], mem_i_data[15:8], write_source_entry[41:2], 2'b11};
        4'b1111: cache_update_entry = {write_source_entry[72:66], mem_i_data, write_source_entry[49:2], 2'b11};
        4'b1101: cache_update_entry = {write_source_entry[72:58], mem_i_data[7:0], write_source_entry[49:2], 2'b11};
        4'b1110: cache_update_entry = {write_source_entry[72:66], mem_i_data[15:8], write_source_entry[57:2], 2'b11};
    endcase
end

// TODO: check if addr is cacheable from MMU

endmodule

module dcache_mem #(parameter AW = 2, parameter AS = 4, parameter DW = 16)(
    input i_clk,
    input i_rst,

    input [AW-1:0] i_addr,
    input [DW-1:0] i_data,
    output reg [DW-1:0] o_data,
    input i_we
);

reg [DW-1:0] mem [AS-1:0];

always @(posedge i_clk) begin
    if (i_rst) begin
        for (integer row = 0; row < AS; row = row+1) begin
            mem[row][`DIRTY_BIT:`VALID_BIT] <= 2'b00;
        end
    end else begin
        if(i_we)
            mem[i_addr] <= i_data;
        o_data <= mem[i_addr];
    end
end

endmodule
