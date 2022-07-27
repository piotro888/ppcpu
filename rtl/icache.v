`include "config.v"

module icache (
    input i_clk,
    input i_rst,

    input imem_req,
    output reg imem_ack,
    input [`RW-1:0] imem_addr,
    output reg [`I_SIZE-1:0] imem_data,
    input imem_next,

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

wire [`ENTRY_SIZE-1:0] cache_mem_in = {cache_tag, line_collect, 1'b1};
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

genvar i;
generate
    for (i=0; i<`CACHE_ASSOC; i=i+1) begin : cache_mem
        cache_mem #(.AW(`CACHE_IDX_WIDTH), .AS(`CACHE_IDXES), .DW(`ENTRY_SIZE)) mem (
            .i_clk(i_clk), .i_addr(cache_index), .i_data(cache_mem_in),
            .o_data(cache_out[i]), .i_we(cache_we[i]));
        assign cache_hit[i] = (cache_out[i][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == cache_tag) && cache_out[i][0]; 
    end
endgenerate

`define SW 2
`define STATE_IDLE `SW'b0
`define STATE_CACHE_READ `SW'b1
`define STATE_MEM_READ `SW'b10
`define STATE_WR_WAIT `SW'b11
reg [`SW-1:0] state;

reg [`RW-1:0] l_req_addr;
reg [`CACHE_OFF_W-1:0] burst_cnt;
reg [`LINE_SIZE-1:0] line_collect;

reg cache_addr_source = 1'b0; 

assign omem_burst4 = 1'b1;
assign omem_addr = {imem_addr[`RW-1:`CACHE_OFF_W], `CACHE_OFF_W'b0};

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `STATE_IDLE;
        imem_ack <= 1'b0;
    end else begin
    case (state)
        default: begin // idle
            imem_ack <= 1'b0;
            if(imem_req)
                state <= `STATE_CACHE_READ;
        end
        `STATE_CACHE_READ: begin
            cache_we[0] <= 1'b0;
            if (|cache_hit) begin
                imem_ack <= 1'b1;
                mentry_out <= entry_out;
                state <= `STATE_IDLE;
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
            mentry_out <= cache_mem_in;
            imem_ack <= 1'b1;
            state <= `STATE_IDLE;
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
    case (cache_off)
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
    if(i_we) // begin
        mem[i_addr] <= i_data;
        //o_data <= i_data;
    //end else begin
        o_data <= mem[i_addr];
    //end
end

endmodule


/*
`include "config.v"

module icache (
    input i_clk,
    input i_rst,

    input imem_req,
    output imem_ack,
    input [`RW-1:0] imem_addr,
    output reg [`I_SIZE-1:0] imem_data,
    input imem_next,

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

wire [`CACHE_IDX_WIDTH-1:0] cache_index = imem_addr[6:2];
wire [`TAG_SIZE-1:0] cache_tag = imem_addr[15:7];
wire [`CACHE_OFF_W-1:0] cache_off = imem_addr[1:0];

wire [`ENTRY_SIZE-1:0] cache_mem_in;
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

genvar i;
generate
    for (i=0; i<`CACHE_ASSOC; i=i+1) begin : cache_mem
        cache_mem #(.AW(`CACHE_IDX_WIDTH), .AS(`CACHE_IDXES), .DW(`ENTRY_SIZE)) mem (
            .i_clk(i_clk), .i_addr(cache_index), .i_data(cache_mem_in),
            .o_data(cache_out[i]), .i_we(cache_we[i]));
        assign cache_hit[i] = (cache_out[i][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == cache_tag) && cache_out[i][0]; 
    end
endgenerate

wire g_cache_hit = |cache_hit;

wire new_req = (imem_ack & imem_next) | (imem_req & ~prev_imem_req);

reg cache_resp_valid, prev_imem_req;
always @(posedge i_clk) begin
    if(i_rst) begin
        cache_resp_valid <= 1'b0;
        prev_imem_req <= 1'b0;
    end else begin
        cache_resp_valid <= new_req;
        prev_imem_req <= imem_req;
    end
end

reg [`RW-1:0] latch_req_addr;
always @(posedge i_clk) begin
    if(imem_req & ~i_rst) begin
        latch_req_addr <= imem_addr;
    end
end

assign omem_addr = {latch_req_addr[`RW-1:`CACHE_OFF_W], `CACHE_OFF_W'b0};

wire [1:0] cache_replace_set = 2'b0; // TODO: LRU

reg omem_req_active;

assign omem_req = omem_req_active | (cache_resp_valid & ~g_cache_hit);
assign omem_burst4 = 1'b1;
reg [`LINE_SIZE-1:0] line_collect;
reg [1:0] line_seg;
assign cache_mem_in = {latch_req_addr[15:7], line_collect, 1'b1};
wire line_ready = omem_req & omem_ack & (&line_seg);
reg test = cache_resp_valid & ~g_cache_hit;
always @(posedge i_clk) begin
    if (i_rst) begin
        omem_req_active <= 1'b0;
        line_collect <= `LINE_SIZE'b0;
        line_seg <= 2'b0;
    end else if (line_ready) begin
        line_seg <= 2'b0;
        line_collect[127:96] <= omem_data;
        omem_req_active <= 1'b0;
    end else if (omem_req & omem_ack) begin
        line_seg <= line_seg + 2'b1;
        case (line_seg)
            default: line_collect[31:0] <= omem_data;
            2'b1: line_collect[63:32] <= omem_data;
            2'b10: line_collect[95:64] <= omem_data;
        endcase
    end else if (cache_resp_valid & ~g_cache_hit) begin
        omem_req_active <= 1'b1;        
    end
end

always @(posedge i_clk) begin
    if(i_rst) begin
        cache_we <= `CACHE_ASSOC'b0;
    end else if (line_ready) begin
        cache_we[cache_replace_set] <= 1'b1;
    end else begin
        cache_we <= `CACHE_ASSOC'b0;
    end
end

assign imem_ack = (cache_resp_valid & g_cache_hit) | (|cache_we);

reg [`ENTRY_SIZE-1:0] entry_out;
always @* begin
    if (|cache_we) begin
        entry_out = {`TAG_SIZE'b0, line_collect, 1'b1};
    end else begin
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
end

always @* begin
    case (cache_off)
        default:
            imem_data = entry_out[32:1];
        2'b01:
            imem_data = entry_out[64:33];
        2'b10:
            imem_data = entry_out[96:65];
        2'b11:
            imem_data = entry_out[128:97];
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
    if(i_we) begin
        mem[i_addr] <= i_data;
        o_data <= i_data;
    end else begin
        o_data <= mem[i_addr];
    end
end

endmodule
*/