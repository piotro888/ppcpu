`include "config.v"

module execute #(parameter CORENO = 0, INT_VEC = 1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    // Pipeline control singnals
    output o_ready,
    input i_submit,
    output reg o_flush,
    input i_flush,

    input [`RW-1:0] i_imm,
    input i_jmp_predict,

    // Execution control singals
    input c_pc_inc, c_pc_ie,
    input c_r_bus_imm,
    input [`ALU_MODE_W-1:0] c_alu_mode,
    input c_alu_carry_en, c_alu_flags_ie,
    input [`REGNO_LOG-1:0] c_l_reg_sel, c_r_reg_sel, 
    input [`REGNO-1:0] c_rf_ie,
    input [`JUMP_CODE_W-1:0] c_jump_cond_code,
    input c_mem_access, c_mem_we, c_mem_width,
    input [1:0] c_used_operands,
    input c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt, c_sys, c_mem_long,

    // Signals to fetch stage to handle mispredictions
    output o_pc_update,
    output [`RW-1:0] o_exec_pc,

    // Debug outputs
    output [`RW-1:0] dbg_r0, dbg_pc,

    // Pipeline next stage
    output reg [`RW-1:0] o_data,
    output reg [`RW-1:0] o_addr,
    output reg [`REGNO-1:0] o_reg_ie,
    output reg o_mem_access, o_mem_we, o_mem_width,
    output reg o_submit,
    input i_next_ready,
    input [`REGNO-1:0] i_reg_ie,
    input [`RW-1:0] i_reg_data,

    input i_irq,
    output o_c_instr_page,
    output reg o_c_data_page,
    output [`RW-1:0] sr_bus_addr, sr_bus_data_o,
    output sr_bus_we,
    output reg o_icache_flush,
    input i_mem_exception,
    input i_core_int,
    input [`RW-1:0] i_core_int_sreg,
    output o_c_instr_long_mode,
    output reg o_mem_long_mode,
    output [7:0] o_instr_addr_high,
    output reg [7:0] o_mem_addr_high
);

reg next_ready_delayed;
// detect RAW pipeline hazard
wire raw_hazard = (
        (c_used_operands[0] & o_reg_ie[c_l_reg_sel]) |
        (c_used_operands[1] & o_reg_ie[c_r_reg_sel]) |
        (c_mem_long & sreg_long_ptr_en & c_used_operands[0] & o_reg_ie[c_l_reg_sel+1])
    ) & (o_submit | ~next_ready_delayed);
// hazard happens also in the first cycle when next_ready becomes high, delayed signal is used 

wire i_invalidate = i_flush | irq | pc_high_updated;
// hazard doesn't invalidate instructions, only holds it
wire hold_req = raw_hazard | alu_mul_busy;

wire i_valid = i_submit & ~i_invalidate;
reg hold_valid;

wire instr_valid = i_valid | (hold_valid & ~i_submit & ~i_invalidate);
wire exec_submit = i_next_ready & instr_valid & ~hold_req;

// don't update state when current instruction is not valid (flush or bubble)
assign o_ready = exec_submit | ~instr_valid;

// At IRQ, current instruction (and state update) is invalidated and pc is saved to sr, to
// continue execution from current instruction. Flush is requested on next cycle
wire irq = ((i_irq | i_core_int) & irq_en) | prev_sys | trap_exception | i_mem_exception;
// core_int is masked as externel event to not interrupt irq handler

always @(posedge i_clk) begin
    if(i_rst) begin
        hold_valid <= 1'b0;
    end else if (i_invalidate | exec_submit) begin
        hold_valid <= 1'b0;
    end else if (i_valid) begin
        hold_valid <= 1'b1;
    end
end

always @(posedge i_clk) begin
    if(i_rst) begin
        next_ready_delayed <= 1'b0;
    end else begin
        next_ready_delayed <= i_next_ready;
    end
end

// Internal buses
wire [`RW-1:0] reg_l_con, reg_r_con;
wire [`RW-1:0] alu_l_bus, alu_r_bus;
wire [`RW-1:0] alu_bus;

// Muxes definitions
assign alu_l_bus = reg_l_con;
assign alu_r_bus = (c_r_bus_imm ? i_imm : reg_r_con);
assign alu_bus = (mul_div_op ? alu_mul_res : alu_res);

// Component connects
wire [`RW-1:0] pc_val;
wire [`ALU_FLAG_CNT-1:0] alu_flags_d, alu_flags_q;
assign dbg_pc = pc_val;
assign o_pc_update = exec_submit;
assign o_exec_pc = pc_val;
wire [`RW-1:0] sreg_in = reg_r_con;
reg [`RW-1:0] sreg_out;
wire [`RW-1:0] dbg_reg_out;
wire pc_overflow;
wire [`RW-1:0] reg_l_high_con;
wire [`RW-1:0] alu_mul_res, alu_res;
wire alu_mul_busy;
wire mul_div_op = (c_alu_mode == `ALU_MODE_MUL) || (c_alu_mode == `ALU_MODE_DIV) || (c_alu_mode == `ALU_MODE_MOD);

// Submodules
rf rf(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(i_reg_data), .o_lout(reg_l_con),
    .o_rout(reg_r_con), .i_lout_sel(c_l_reg_sel), .i_rout_sel(c_r_reg_sel),
    .i_ie(i_reg_ie), .i_gie(1'b1), .o_l_high_out(reg_l_high_con),
    .dbg_r0(dbg_r0), .dbg_sel(dbg_reg_sel), .dbg_reg(dbg_reg_out));

alu alu(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_l(alu_l_bus), .i_r(alu_r_bus), .o_out(alu_res), .i_mode(c_alu_mode), 
    .o_flags(alu_flags_d), .i_carry(alu_flags_q[`ALU_FLAG_C] & c_alu_carry_en));

alu_mul_div alu_mul_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    .i_rst(i_rst),

    .i_a(alu_l_bus), .i_b(alu_r_bus), .o_d(alu_mul_res),
    .i_submit(mul_div_op & i_submit), .i_flush(i_invalidate),
    .o_busy(alu_mul_busy),
    .i_mul(c_alu_mode == `ALU_MODE_MUL),
    .i_div(c_alu_mode == `ALU_MODE_DIV),
    .i_mod(c_alu_mode == `ALU_MODE_MOD)
);

pc #(.INT_VEC(INT_VEC)) pc (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_bus(c_sreg_store | c_sreg_irt ? (c_sreg_irt ? sreg_out : sreg_in) : alu_bus),
    .i_c_pc_inc((c_pc_inc | (~jump_dec_en & jump_dec_valid)) & exec_submit), .i_c_pc_ie((c_pc_ie | (jump_dec_en & jump_dec_valid) | pc_write) & exec_submit),
    .o_pc(pc_val), .i_c_pc_irq(irq), .o_pc_ovf(pc_overflow));

// Cpu control registers
register  #(.N(`ALU_FLAG_CNT)) alu_flag_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d((alu_flags_sreg_ie ? sreg_in[`ALU_FLAG_CNT-1:0] : alu_flags_d)),
    .o_d(alu_flags_q), .i_ie((c_alu_flags_ie | alu_flags_sreg_ie) & exec_submit));

// JUMP DECODE
reg jump_dec_en;
wire jump_dec_valid = c_jump_cond_code[`JUMP_CODE_BIT_EN];

wire jump_mispredict = jump_dec_valid & (jump_dec_en ^ i_jmp_predict);
wire pc_write = (pc_sreg_ie & c_sreg_store) | c_sreg_irt;

always @(posedge i_clk) begin
    o_flush <= ((jump_mispredict | pc_write | flush_instr_mmu) & exec_submit) | irq | pc_high_updated; // invalidate itself and all previous stages at next cycle
end

`define JUMP_CODE_UNCOND`JUMP_CODE_W'b10000
`define JUMP_CODE_CARRY `JUMP_CODE_W'b10001
`define JUMP_CODE_EQUAL `JUMP_CODE_W'b10010
`define JUMP_CODE_LT    `JUMP_CODE_W'b10011
`define JUMP_CODE_GT    `JUMP_CODE_W'b10100
`define JUMP_CODE_LE    `JUMP_CODE_W'b10101
`define JUMP_CODE_GE    `JUMP_CODE_W'b10110
`define JUMP_CODE_NE    `JUMP_CODE_W'b10111
`define JUMP_CODE_OVF   `JUMP_CODE_W'b11000
`define JUMP_CODE_PAR   `JUMP_CODE_W'b11001
`define JUMP_CODE_GTU   `JUMP_CODE_W'b11010
`define JUMP_CODE_GEU   `JUMP_CODE_W'b11011
`define JUMP_CODE_LEU   `JUMP_CODE_W'b11100
always @(*) begin
    case (c_jump_cond_code[`JUMP_CODE_W-1:0])
        `JUMP_CODE_UNCOND:
            jump_dec_en = 1'b1;
        `JUMP_CODE_CARRY:
            jump_dec_en = alu_flags_q[`ALU_FLAG_C];
        `JUMP_CODE_EQUAL:
            jump_dec_en = alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_LT:
            jump_dec_en = alu_flags_q[`ALU_FLAG_N];
        `JUMP_CODE_GT:
            jump_dec_en = ~(alu_flags_q[`ALU_FLAG_N] | alu_flags_q[`ALU_FLAG_Z]);
        `JUMP_CODE_LE:
            jump_dec_en = alu_flags_q[`ALU_FLAG_N] | alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_GE:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_N];
        `JUMP_CODE_NE:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_OVF:
            jump_dec_en = alu_flags_q[`ALU_FLAG_O];
        `JUMP_CODE_PAR:
            jump_dec_en = alu_flags_q[`ALU_FLAG_P];
        `JUMP_CODE_GTU:
            jump_dec_en = ~(alu_flags_q[`ALU_FLAG_C] | alu_flags_q[`ALU_FLAG_Z]);
        `JUMP_CODE_GEU:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_C];
        `JUMP_CODE_LEU:
            jump_dec_en = alu_flags_q[`ALU_FLAG_C] | alu_flags_q[`ALU_FLAG_Z];
        default:
            jump_dec_en = 1'b0;
    endcase
end

// Forwarding to next pipeline stage
always @(posedge i_clk) begin
    if (i_rst) begin
        o_submit <= 1'b0;
    end else if (exec_submit) begin
        o_addr <= alu_bus;
        o_data <= (c_mem_access ? reg_r_con : 
            (c_sreg_load | c_sreg_jal_over ? sreg_out + (c_sreg_jal_over ? `RW'b1 : `RW'b0) 
                                            : alu_bus)); 
        o_reg_ie <= c_rf_ie;
        o_mem_access <= c_mem_access;
        o_mem_we <= c_mem_we;
        o_mem_width <= c_mem_width;
        o_mem_long_mode <= c_mem_long & sreg_long_ptr_en;
        o_mem_addr_high <= computed_mem_addr_high;
        o_submit <= 1'b1;
    end else begin
        o_submit <= 1'b0;
    end
end

reg prev_sys; // Execute sys instruction and trigger interrupt at next cycle to resume from next instruction
always @(posedge i_clk) begin
    if (i_rst) begin
        prev_sys <= 1'b0;
    end else if (c_sys & exec_submit) begin
        prev_sys <= 1'b1;
    end else begin
        prev_sys <= 1'b0;
    end
end

reg trap_exception;
always @(posedge i_clk) begin
    if (i_rst) begin
        trap_exception <= 1'b0;
    end else begin
        trap_exception <= trap_flag & exec_submit;
    end
end

reg [`RW-1:0] mem_stage_pc;
always @(posedge i_clk) begin
    if (i_rst)
        mem_stage_pc <= `RW'b0;
    else if (exec_submit)
        mem_stage_pc <= pc_val;
end

// Special registers
`define SREG_PC `RW'b0
`define SREG_PRIV_CTRL `RW'b1
`define SREG_JTR `RW'b10
`define SREG_IRQ_PC `RW'b11
`define SREG_ALU_FLAGS `RW'b100
`define SREG_IRQ_FLAGS `RW'b101
`define SREG_SCRATCH `RW'b110
`define SREG_CPUID `RW'b111
`define SREG_COREID `RW'b1000
`define SREG_MT_IRQ `RW'b1001 //,1010, 1011
`define SREG_PC_HIGH `RW'b1100
`define SREG_PC_HIGH_BUFF `RW'b1101

reg pc_sreg_ie, sreg_priv_control_ie, sreg_irq_pc_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie, sreg_pc_high_ie, sreg_pc_high_buff_ie;
wire [`RW-1:0] sreg_priv_control_out, sreg_irq_pc_out, sreg_scratch_out;
always @* begin
    {pc_sreg_ie, sreg_irq_pc_ie, sreg_priv_control_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie, sreg_pc_high_ie, sreg_pc_high_buff_ie} = 8'b0;
    case (i_imm)
        `SREG_PC: begin
            sreg_out = pc_val;
            pc_sreg_ie = c_sreg_store;
        end
        `SREG_PRIV_CTRL: begin
            sreg_out = sreg_priv_control_out;
            sreg_priv_control_ie = c_sreg_store;
        end
        `SREG_IRQ_PC: begin
            sreg_out = sreg_irq_pc_out;
            sreg_irq_pc_ie = c_sreg_store;
        end
        `SREG_JTR: begin
            sreg_out = {13'b0, sreg_jtr_out};
            sreg_jtr_ie = c_sreg_store & sreg_priv_mode;
        end
        `SREG_ALU_FLAGS: begin
            sreg_out = {11'b0, alu_flags_q};
            alu_flags_sreg_ie = c_sreg_store;
        end
        `SREG_IRQ_FLAGS: begin
            sreg_out = {11'b0, sreg_irq_flags_out};
        end
        `SREG_SCRATCH: begin
            sreg_out = sreg_scratch_out;
            sreg_scratch_ie = c_sreg_store;
        end
        `SREG_CPUID: begin
            sreg_out = 16'b1110_0000_0001_0001;
        end
        `SREG_COREID: begin
            sreg_out = CORENO;
        end
        `SREG_MT_IRQ: begin // write is handled in upper_core
            sreg_out = i_core_int_sreg;
        end
        `SREG_PC_HIGH: begin
            sreg_pc_high_ie = c_sreg_store;
            sreg_out = {8'b0, pc_high_out};
        end
        `SREG_PC_HIGH_BUFF: begin
            sreg_pc_high_buff_ie = c_sreg_store;
            sreg_out = {8'b0, pc_high_buff_out};
        end 
        default:
            sreg_out = 16'b0;
    endcase

    if(c_sreg_jal_over)
        sreg_out = pc_val;
    if(c_sreg_irt)
        sreg_out = sreg_irq_pc_out;
end

// Special registers control


wire [`RW-1:0] priv_in = (irq ? (`RW'b0001) : (c_sreg_irt ? (sreg_priv_control_out | `RW'h0004) : sreg_in)); // disable irq and paging flag on interrupt and re-enable on return
register #(.RESET_VAL(`RW'b0001)) sreg_priv_control (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(priv_in), .o_d(sreg_priv_control_out),
    .i_ie((((sreg_priv_control_ie & sreg_priv_mode) | c_sreg_irt) & exec_submit) | irq));

wire sreg_priv_mode = sreg_priv_control_out[0];
wire sreg_data_page = sreg_priv_control_out[1];
wire irq_en = sreg_priv_control_out[2];
wire sreg_long_ptr_en = sreg_priv_control_out[3];

register sreg_irq_pc (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_irq_pc_ie ? sreg_in : (i_mem_exception ? mem_stage_pc : pc_val)), .o_d(sreg_irq_pc_out), .i_ie(irq | (sreg_irq_pc_ie & exec_submit)));

wire [2:0] sreg_jtr_buff_o, sreg_jtr_out;
wire jtr_jump_en = (pc_sreg_ie | jump_dec_valid | c_sreg_irt) & exec_submit;
wire jtr_irqh_write = irq;
wire [2:0] jtr_buff_in = (irq ? 3'b000 : sreg_in[2:0]);
wire [2:0] jtr_in = (irq ? 3'b000 : sreg_jtr_buff_o);
register  #(.RESET_VAL((CORENO == 0 ? 3'b001 : 3'b000)), .N(3)) sreg_jtr_buff (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_buff_in), .o_d(sreg_jtr_buff_o), .i_ie(sreg_jtr_ie | jtr_irqh_write));
register  #(.RESET_VAL((CORENO == 0 ? 3'b001 : 3'b000)), .N(3)) sreg_jtr (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_in), .o_d(sreg_jtr_out), .i_ie(jtr_jump_en | jtr_irqh_write));
assign o_c_instr_page = sreg_jtr_out[0];
wire trap_flag = sreg_jtr_out[1];
wire long_pc_mode = sreg_jtr_out[2];

register sreg_scratch (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_in), .o_d(sreg_scratch_out), .i_ie(sreg_scratch_ie & exec_submit));

wire [4:0] sreg_irq_flags_in = {(i_core_int & irq_en), i_mem_exception, trap_exception, prev_sys, (i_irq & irq_en)};
wire [4:0] sreg_irq_flags_out;
register #(.N(5)) sreg_irq_flags (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_irq_flags_in), .o_d(sreg_irq_flags_out), .i_ie(irq));

wire immu_write = c_sreg_store & exec_submit & (sr_bus_addr >= `RW'h100 && sr_bus_addr < `RW'h100 + 16); // flush after write to mmu is executed
wire flush_instr_mmu = (immu_write & o_c_instr_page) | ((jtr_in[0] ^ sreg_jtr_out[0]) & (jtr_jump_en | jtr_irqh_write)) | pc_high_updated;
always @(posedge i_clk)
    o_icache_flush <= flush_instr_mmu & ~i_rst;

// Delays disable of data paging in case of interrupt. MEMWB stage is still executing and changing
// address during request would break commited result. In other cases special handling is not needed,
// becaue sregs are updated only when next stage is ready
always @(posedge i_clk) begin
    if (i_rst)
        o_c_data_page <= 1'b0;
    else if (i_next_ready)
        o_c_data_page <= sreg_data_page;
end

// Higher part of PC in long pointer mode
// long pc mode is disabled on interrupt and reabled via jtr at irt, bot registers can be recovered
wire [7:0] pc_high = (long_pc_mode ? pc_high_out : 'b0);

wire pc_high_ovf = pc_overflow & long_pc_mode & exec_submit;
wire pc_high_jtr = (pc_sreg_ie | jump_dec_valid | c_sreg_irt) & long_pc_mode & exec_submit;
// Update from buffer at jump or at pc overflow
wire [7:0] pc_high_in = (sreg_pc_high_ie ? sreg_in[7:0] : (pc_high_jtr ? pc_high_buff_out : pc_high_out+7'b1));
wire [7:0] pc_high_out;
register #(.N(8), .RESET_VAL(8'h80)) pc_high_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst),
    .i_d(pc_high_in),
    .o_d(pc_high_out),
    .i_ie((pc_high_ovf | pc_high_jtr | sreg_pc_high_ie) & exec_submit)
);

// Update buffer on function call (JAL) with current pointer
wire [7:0] pc_high_buff_in = (sreg_pc_high_buff_ie ? sreg_in[7:0] : pc_high);
wire [7:0] pc_high_buff_out;
register #(.N(8), .RESET_VAL(8'h80)) pc_high_buff_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst),
    .i_d(pc_high_buff_in),
    .o_d(pc_high_buff_out),
    .i_ie((sreg_pc_high_buff_ie | (c_sreg_jal_over & long_pc_mode)) & exec_submit)
);

wire pc_high_updated = |(prev_pc_high ^ pc_high); // flush pipeline and cache on update. current instuction is incorrect
reg [7:0] prev_pc_high;
always @(posedge i_clk) begin
    if (i_rst) prev_pc_high <= 'b0;
    else prev_pc_high <= pc_high;
end

assign o_c_instr_long_mode = long_pc_mode;
assign o_instr_addr_high = pc_high;

wire [7:0] computed_mem_addr_high = (~alu_flags_d[`ALU_FLAG_C] & alu_r_bus[`RW-1] ? reg_l_high_con[7:0]-8'b1 : 
    (alu_flags_d[`ALU_FLAG_C] ? reg_l_high_con[7:0]+8'b1 : reg_l_high_con[7:0]));

assign sr_bus_addr = i_imm;
assign sr_bus_we = c_sreg_store & sreg_priv_mode & exec_submit;
assign sr_bus_data_o = sreg_in;

assign dbg_out = {o_ready, pc_val, dbg_reg_out};

endmodule
