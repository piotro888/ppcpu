`include "config.v"

module icache (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,
    input i_rst,

    input mem_req,
    output mem_ack,
    input [`RW-1:0] mem_addr,
    output reg [`I_SIZE-1:0] mem_data,
    input mem_ppl_submit,
    input mem_cache_flush,

    // output interface
    output reg wb_cyc,
    output reg wb_stb,
    input [`RW-1:0] wb_i_dat,
    output [`RW-1:0]  wb_adr,
    output wb_we,
    input wb_ack,
    output [1:0] wb_sel,
    input wb_err
);

assign wb_sel = 2'b11;
assign wb_we = 1'b0;

`define TAG_SIZE 9
`define LINE_SIZE 128
// 9b tag + 128b line + 1b valid
`define ENTRY_SIZE 138
`define CACHE_ASSOC 1
`define CACHE_ENTR_N 32
`define CACHE_SETS_N 8
`define CACHE_OFF_W 2

`define CACHE_IDX_WIDTH 5
`define CACHE_IDXES 32


wire [`TAG_SIZE-1:0] compare_tag = cache_read_addr[15:7];
wire [`TAG_SIZE-1:0] write_tag = cache_write_addr[15:7];
wire [`CACHE_IDX_WIDTH-1:0] read_index = (submit_pending ?  submit_pending_addr[6:2] : mem_addr[6:2]);
wire [`CACHE_IDX_WIDTH-1:0] wire_index = cache_write_addr[6:2];
wire [`CACHE_OFF_W-1:0] compare_off = cache_read_addr[1:0];
wire [`CACHE_OFF_W-1:0] write_off = cache_write_addr[1:0];

wire [`ENTRY_SIZE-1:0] cache_mem_in = cache_write_entry;
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

reg [`RW-1:0] cache_read_addr, cache_write_addr;
reg cache_read_valid;
wire cache_write_valid = wb_cyc & wb_stb;
reg prev_write_compl;

wire [`CACHE_IDX_WIDTH-1:0] cache_addr = ((|cache_we) ? wire_index : read_index);

`ifndef USE_OC_RAM

icache_ram mem (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_addr(cache_addr), .i_data(cache_mem_in),
    .o_data(cache_out[0]), .i_we(cache_we[0])
);

`else

ocram_icache mem (
    .clock(i_clk), .address(cache_addr), .data(cache_mem_in),
    .q(cache_out[0]), .wren(cache_we[0])
);

`endif

assign cache_hit[0] = (cache_out[0][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == compare_tag) && valid_bits[cache_addr]; 

reg [`CACHE_ENTR_N-1:0] valid_bits;
always @(posedge i_clk) begin
    if (i_rst | mem_cache_flush) begin
        valid_bits <= `CACHE_ENTR_N'b0;
    end else if (cache_we[0]) begin
        valid_bits[cache_addr] <= cache_mem_in[0];
    end
end

assign mem_ack = cache_ghit | mem_fetch_end;

wire cache_miss = cache_read_valid & (~(|cache_hit) | flush_prev_cycle);
wire cache_ghit = cache_read_valid & ((|cache_hit) & ~flush_prev_cycle);

always @(posedge i_clk)
    cache_read_addr <= (submit_pending ? submit_pending_addr : mem_addr);

always @(posedge i_clk) begin
    if(cache_read_valid)
        cache_write_addr <= cache_read_addr;
end

always @(posedge i_clk)
    prev_write_compl <= |cache_we;

reg submit_pending;
reg [`RW-1:0] submit_pending_addr;
always @(posedge i_clk) begin
    if(i_rst)
        submit_pending <= 1'b0;
    else if (mem_ppl_submit & ~accept_ok) begin
        submit_pending <= 1'b1;
        submit_pending_addr <= mem_addr;
    end else if (accept_ok)
        submit_pending <= 1'b0;
end

wire accept_ok = mem_req & ~cache_write_valid & ~cache_miss;

reg flush_prev_cycle;
always @(posedge i_clk)
    flush_prev_cycle <= mem_cache_flush; // feedback after invalidating memortm takes 2 cycles; use this to force miss signal after 1 cycle

always @(posedge i_clk) begin
    if(i_rst)
        cache_read_valid <= 1'b0;
    else
        cache_read_valid <= accept_ok & (submit_pending | mem_ppl_submit);
end

wire mem_fetch_end = wb_cyc & wb_stb & (wb_ack | wb_err) & (&line_burst_cnt);

assign wb_adr = {cache_write_addr[14:2], line_burst_cnt};

reg [`LINE_SIZE-1:0] line_collect;
reg [`CACHE_OFF_W:0] line_burst_cnt;
reg invalidate_bus_err;
always @(posedge i_clk) begin
    if (i_rst) begin
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        invalidate_bus_err <= 1'b0;
    end else if (mem_fetch_end) begin
        line_burst_cnt <= 3'b0;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        invalidate_bus_err <= invalidate_bus_err | wb_err;
    end else if (wb_cyc & wb_stb & (wb_ack | wb_err)) begin
        line_burst_cnt <= line_burst_cnt + 1'b1;
        invalidate_bus_err <= invalidate_bus_err | wb_err;
    end else if(cache_miss) begin
        line_burst_cnt <= 3'b0;
        wb_cyc <= 1'b1;
        wb_stb <= 1'b1;
        invalidate_bus_err <= 1'b0;
    end
end

reg invalidate_cache_update; // don't write pending memory request to cache after flush (fetch unit invalidates this request)
always @(posedge i_clk) begin
    if (i_rst) begin
        invalidate_cache_update <= 1'b0;
    end else if (mem_fetch_end) begin
        invalidate_cache_update <= 1'b0;
    end else if (mem_cache_flush & (cache_write_valid | cache_miss)) begin
        invalidate_cache_update <= 1'b1;
    end
end

wire invalidate_bus_err_w = (mem_fetch_end & wb_err) | invalidate_bus_err;

wire [`LINE_SIZE-1:0] pre_assembled_line = {wb_i_dat, line_collect[111:0]};
wire [`ENTRY_SIZE-1:0] cache_write_entry = {write_tag, pre_assembled_line, 1'b1};
wire cache_we_en = mem_fetch_end & ~invalidate_cache_update & ~invalidate_bus_err_w;
always @* begin
    cache_we[0] = cache_we_en;
end


always @(posedge i_clk) begin
    case (line_burst_cnt)
        default: line_collect[15:0] <= wb_i_dat;
        3'b001: line_collect[31:16] <= wb_i_dat;
        3'b010: line_collect[47:32] <= wb_i_dat;
        3'b011: line_collect[63:48] <= wb_i_dat;
        3'b100: line_collect[79:64] <= wb_i_dat;
        3'b101: line_collect[95:80] <= wb_i_dat;
        3'b110: line_collect[111:96] <= wb_i_dat;
        3'b111: line_collect[127:112] <= wb_i_dat;
    endcase
end

reg [`ENTRY_SIZE-1:0] cache_hit_entry;

wire [`ENTRY_SIZE-1:0] entry_out = (mem_fetch_end ? {`TAG_SIZE'b0, pre_assembled_line, 1'b0} : cache_hit_entry);

always @* begin
    cache_hit_entry = cache_out[0];
end

wire [`CACHE_OFF_W-1:0] offset_out = (mem_fetch_end ? write_off : compare_off);
always @* begin
    case (offset_out)
        default: mem_data = entry_out[32:1];
        2'b01: mem_data = entry_out[64:33];
        2'b10: mem_data = entry_out[96:65];
        2'b11: mem_data = entry_out[128:97];
    endcase
end

endmodule

`undef TAG_SIZE
`undef LINE_SIZE
`undef ENTRY_SIZE
`undef CACHE_ASSOC
`undef CACHE_ENTR_N
`undef CACHE_SETS_N
`undef CACHE_OFF_W
`undef CACHE_IDX_WIDTH
`undef CACHE_IDXES
`undef SW