`include "config.v"

module icache (
    input i_clk,
    input i_rst,

    input imem_req,
    output reg imem_ack,
    input [`RW-1:0] imem_addr,
    output reg [`I_SIZE-1:0] imem_data,
    input imem_next, // this signal is like pre-next and allows submitting new request at ack edge (in comb domain)
    input imem_sync_next, // this signal is std bus fetch, that allows submitting new request if ack is high

    output omem_req,
    output [`RW-1:0] omem_addr,
    input [`I_SIZE-1:0] omem_data,
    input omem_ack,
    output omem_burst4
);

`define TAG_SIZE 9
`define LINE_SIZE 128
// 9b tag + 128b line + 1b valid
`define ENTRY_SIZE 138
`define CACHE_ASSOC 4
`define CACHE_ENTR_N 32
`define CACHE_SETS_N 8
`define CACHE_OFF_W 2

`define CACHE_IDX_WIDTH 5
`define CACHE_IDXES 32


wire [`CACHE_IDX_WIDTH-1:0] cache_index = (cache_addr_source ? l_req_addr[6:2] : imem_addr[6:2]);
wire [`TAG_SIZE-1:0] cache_tag = (cache_addr_source ? l_req_addr[15:7] : imem_addr[15:7]);
wire [`CACHE_OFF_W-1:0] cache_off = (cache_addr_source ? l_req_addr[1:0] : imem_addr[1:0]);

wire [`ENTRY_SIZE-1:0] cache_mem_in = {l_req_addr[15:7], line_collect, 1'b1};
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

genvar i;
generate
    for (i=0; i<`CACHE_ASSOC; i=i+1) begin : cache_mem
        cache_mem #(.AW(`CACHE_IDX_WIDTH), .AS(`CACHE_IDXES), .DW(`ENTRY_SIZE)) mem (
            .i_clk(i_clk), .i_addr(cache_index), .i_data(cache_mem_in),
            .o_data(cache_out[i]), .i_we(cache_we[i]));
        assign cache_hit[i] = (cache_out[i][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == l_req_addr[15:7]) && cache_out[i][0]; 
    end
endgenerate

`define SW 3
`define STATE_IDLE `SW'b0
`define STATE_CACHE_READ `SW'b1
`define STATE_MEM_READ `SW'b10
`define STATE_WR_WAIT `SW'b11
`define STATE_AW_NEXT `SW'b100
reg [`SW-1:0] state;

reg [`RW-1:0] l_req_addr;
reg [`CACHE_OFF_W-1:0] burst_cnt;
reg [`LINE_SIZE-1:0] line_collect;
reg [`CACHE_OFF_W-1:0] out_cache_off;

wire cache_addr_source = |cache_we; 

assign omem_burst4 = 1'b1;
assign omem_addr = {l_req_addr[`RW-1:`CACHE_OFF_W], burst_cnt};

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `STATE_IDLE;
        imem_ack <= 1'b0;
        omem_req <= 1'b0;
        cache_we <= `CACHE_ASSOC'b0;
    end else begin
    case (state)
        default: begin // idle
            imem_ack <= 1'b0;
            
            if(imem_req & ~imem_ack | (imem_req & imem_ack & imem_sync_next)) begin
                state <= `STATE_CACHE_READ;
                l_req_addr <= imem_addr;
            end
        end
        `STATE_CACHE_READ: begin
            cache_we[0] <= 1'b0;
            if (|cache_hit) begin
                imem_ack <= 1'b1;
                mentry_out <= entry_out;
                state <= `STATE_IDLE;
                out_cache_off <= l_req_addr[1:0];
                if(imem_req & imem_next) begin
                    l_req_addr <= imem_addr;
                    state <= `STATE_CACHE_READ;
                end
            end else begin
                // cache miss
                imem_ack <= 1'b0;
                omem_req <= 1'b1;
                state <= `STATE_MEM_READ;
                burst_cnt <= `CACHE_OFF_W'b0;
            end
        end
        `STATE_MEM_READ: begin
            if (omem_ack) begin
                burst_cnt <= burst_cnt + `CACHE_OFF_W'b1;
                case (burst_cnt)
                    default: line_collect[31:0] <= omem_data;
                    2'b1: line_collect[63:32] <= omem_data;
                    2'b10: line_collect[95:64] <= omem_data;
                    2'b11: line_collect[127:96] <= omem_data;
                endcase

                if(&burst_cnt) begin
                    cache_we[0] <= 1'b1; // TODO: Cache assoc
                    state <= `STATE_WR_WAIT;
                    omem_req <= 1'b0;
                end
            end
        end
        `STATE_WR_WAIT: begin
            cache_we <= `CACHE_ASSOC'b0;
            mentry_out <= cache_mem_in;
            out_cache_off <= l_req_addr[1:0];
            imem_ack <= 1'b1;
            state <= `STATE_IDLE;
            if(imem_req & imem_next)
                state <= `STATE_AW_NEXT;
            // `next` not hanlded here, because a 1 cycle delay from cache write is needed
        end
        `STATE_AW_NEXT: begin
            imem_ack <= 1'b0;
            state <= `STATE_CACHE_READ;
            l_req_addr <= imem_addr;
        end
    endcase
    end
end

reg [`ENTRY_SIZE-1:0] entry_out;
reg [`ENTRY_SIZE-1:0] mentry_out;

always @* begin
    case (cache_hit)
        default:
            entry_out = cache_out[0];
        4'b0010:
            entry_out = cache_out[1];
        4'b0100:
            entry_out = cache_out[2];
        4'b1000:
            entry_out = cache_out[3];
    endcase
end

always @* begin
    case (out_cache_off)
        default:
            imem_data = mentry_out[32:1];
        2'b01:
            imem_data = mentry_out[64:33];
        2'b10:
            imem_data = mentry_out[96:65];
        2'b11:
            imem_data = mentry_out[128:97];
    endcase
end 


endmodule

module cache_mem #(parameter AW = 2, parameter AS = 4, parameter DW = 16)(
    input i_clk,
    input [AW-1:0] i_addr,
    input [DW-1:0] i_data,
    output reg [DW-1:0] o_data,
    input i_we
);

reg [DW-1:0] mem [AS-1:0];

always @(posedge i_clk) begin
    if(i_we)
        mem[i_addr] <= i_data;
    o_data <= mem[i_addr];
end

endmodule
