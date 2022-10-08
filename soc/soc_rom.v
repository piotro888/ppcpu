`include "config.v"

module soc_rom (
    input [`WB_ADDR_W-1:0] in_addr,
    output reg [`RW-1:0] out_data
);

always @* begin
    // if (in_addr == `WB_ADDR_W'hffe000) out_data = `RW'h0000; // simple counter
    // else if (in_addr == `WB_ADDR_W'hffe001) out_data = `RW'h0000;
    // else if (in_addr == `WB_ADDR_W'hffe002) out_data = `RW'h0004;
    // else if (in_addr == `WB_ADDR_W'hffe003) out_data = `RW'h0001;
    // else if (in_addr == `WB_ADDR_W'hffe004) out_data = `RW'h0008;
    // else if (in_addr == `WB_ADDR_W'hffe005) out_data = `RW'h0001;
    // else if (in_addr == `WB_ADDR_W'hffe006) out_data = `RW'h000e;
    // else if (in_addr == `WB_ADDR_W'hffe007) out_data = `RW'h0002;
        if (in_addr == `WB_ADDR_W'hffe000) out_data = `RW'h0000; // conter with memory access
    else if (in_addr == `WB_ADDR_W'hffe001) out_data = `RW'h0000;
    else if (in_addr == `WB_ADDR_W'hffe002) out_data = `RW'h0004;
    else if (in_addr == `WB_ADDR_W'hffe003) out_data = `RW'h0000;
    else if (in_addr == `WB_ADDR_W'hffe004) out_data = `RW'h0005;
    else if (in_addr == `WB_ADDR_W'hffe005) out_data = `RW'h0000;
    else if (in_addr == `WB_ADDR_W'hffe006) out_data = `RW'h0004;
    else if (in_addr == `WB_ADDR_W'hffe007) out_data = `RW'h0000;
    else if (in_addr == `WB_ADDR_W'hffe008) out_data = `RW'h0002;
    else if (in_addr == `WB_ADDR_W'hffe009) out_data = `RW'h0000;
    else if (in_addr == `WB_ADDR_W'hffe00a) out_data = `RW'h0008;
    else if (in_addr == `WB_ADDR_W'hffe00b) out_data = `RW'h0002;
    else if (in_addr == `WB_ADDR_W'hffe00c) out_data = `RW'h000e;
    else if (in_addr == `WB_ADDR_W'hffe00d) out_data = `RW'h0002;
    else out_data = `RW'b0;
end

endmodule