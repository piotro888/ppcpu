`include "config.v"

module execute (
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
    input c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt, c_sys,

    // Signals to fetch stage to handle mispredictions
    output o_pc_update,
    output [`RW-1:0] o_exec_pc,

    // Debug outputs
    output [`RW-1:0] dbg_r0, dbg_pc,

    // Pipeline next stage
    output reg [`RW-1:0] o_data,
    output reg[`RW-1:0] o_addr,
    output reg [`REGNO-1:0] o_reg_ie,
    output reg o_mem_access, o_mem_we, o_mem_width,
    output reg o_submit,
    input i_next_ready,
    input [`REGNO-1:0] i_reg_ie,
    input [`RW-1:0] i_reg_data,

    input i_irq,
    output o_c_instr_page, o_c_data_page,
    output [`RW-1:0] sr_bus_addr, sr_bus_data_o,
    output sr_bus_we,
    output reg o_icache_flush
);

reg next_ready_delayed;
// detect RAW pipeline hazard
wire raw_hazard = ((c_used_operands[0] & o_reg_ie[c_l_reg_sel]) |
    (c_used_operands[1] & o_reg_ie[c_r_reg_sel])) & (o_submit | ~next_ready_delayed);
// hazard happens also in the first cycle when next_ready becomes high, delayed signal is used 

wire i_invalidate = i_flush | irq;
// hazard doesn't invalidate instructions, only holds it
wire hold_req = raw_hazard;

wire i_valid = i_submit & ~i_invalidate;
reg hold_valid;

wire instr_valid = i_valid | (hold_valid & ~i_submit & ~i_invalidate);
wire exec_submit = i_next_ready & instr_valid & ~hold_req;

// don't update state when current instruction is not valid (flush or bubble)
assign o_ready = exec_submit | ~instr_valid;

// At IRQ, current instruction (and state update) is invalidated and pc is saved to sr, to
// continue execution from current instruction. Flush is requested on next cycle
wire irq = (i_irq & irq_en) | prev_sys;

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

// Component connects
wire [`RW-1:0] pc_val;
wire [`ALU_FLAG_CNT-1:0] alu_flags_d, alu_flags_q;
assign dbg_pc = pc_val;
assign o_pc_update = exec_submit;
assign o_exec_pc = pc_val;
wire [`RW-1:0] sreg_in = reg_r_con;
reg [`RW-1:0] sreg_out;

// Submodules
rf rf(.i_clk(i_clk), .i_rst(i_rst), .i_d(i_reg_data), .o_lout(reg_l_con),
    .o_rout(reg_r_con), .i_lout_sel(c_l_reg_sel), .i_rout_sel(c_r_reg_sel),
    .i_ie(i_reg_ie), .i_gie(1'b1), .dbg_r0(dbg_r0));

alu alu(.i_l(alu_l_bus), .i_r(alu_r_bus), .o_out(alu_bus), .i_mode(c_alu_mode), 
    .o_flags(alu_flags_d), .i_carry(alu_flags_q[`ALU_FLAG_C] & c_alu_carry_en));

pc pc(.i_clk(i_clk), .i_rst(i_rst), .i_bus(c_sreg_store | c_sreg_irt ? (c_sreg_irt ? sreg_out : sreg_in) : alu_bus),
    .i_c_pc_inc((c_pc_inc | (~jump_dec_en & jump_dec_valid)) & exec_submit), .i_c_pc_ie((c_pc_ie | (jump_dec_en & jump_dec_valid) | pc_write) & exec_submit),
    .o_pc(pc_val), .i_c_pc_irq(irq));

// Cpu control registers
register  #(.N(`ALU_FLAG_CNT)) alu_flag_reg (.i_clk(i_clk), .i_rst(i_rst), .i_d((alu_flags_sreg_ie ? sreg_in[`ALU_FLAG_CNT-1:0] : alu_flags_d)),
    .o_d(alu_flags_q), .i_ie((c_alu_flags_ie | alu_flags_sreg_ie) & exec_submit));

// JUMP DECODE
reg jump_dec_en;
wire jump_dec_valid = c_jump_cond_code[`JUMP_CODE_BIT_EN];

wire jump_mispredict = jump_dec_valid & (jump_dec_en ^ i_jmp_predict);
wire pc_write = (pc_sreg_ie & c_sreg_store) | c_sreg_irt;

always @(posedge i_clk) begin
    o_flush <= ((jump_mispredict | pc_write | flush_instr_mmu) & exec_submit) | irq; // invalidate itself and all previous stages at next cycle
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

// Special registers
`define SREG_PC `RW'b0
`define SREG_PRIV_CTRL `RW'b1
`define SREG_JTR `RW'b10
`define SREG_IRQ_PC `RW'b11
`define SREG_ALU_FLAGS `RW'b100
`define SREG_IRQ_FLAGS `RW'b101
`define SREG_SCRATCH `RW'b110

reg pc_sreg_ie, sreg_priv_control_ie, sreg_irq_pc_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie;
wire [`RW-1:0] sreg_priv_control_out, sreg_irq_pc_out, sreg_scratch_out;
always @* begin
    {pc_sreg_ie, sreg_irq_pc_ie, sreg_priv_control_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie} = 6'b0;
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
            sreg_out = {15'b0, sreg_jtr_out};
            sreg_jtr_ie = c_sreg_store;
        end
        `SREG_ALU_FLAGS: begin
            sreg_out = {11'b0, alu_flags_q};
            alu_flags_sreg_ie = c_sreg_store;
        end
        `SREG_IRQ_FLAGS: begin
            sreg_out = {15'b0, sreg_syscall};
        end
        `SREG_SCRATCH: begin
            sreg_out = sreg_scratch_out;
            sreg_scratch_ie = c_sreg_store;
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


wire [`RW-1:0] priv_in = (irq ? (sreg_priv_control_out & `RW'hfffb) : (c_sreg_irt ? (sreg_priv_control_out | `RW'h0004) : sreg_in)); // disable irq flag on interrupt and re-enable on return
register #(.RESET_VAL(`RW'b001)) sreg_priv_control (.i_clk(i_clk), .i_rst(i_rst), .i_d(priv_in), .o_d(sreg_priv_control_out),
    .i_ie((((sreg_priv_control_ie & sreg_priv_mode) | c_sreg_irt) & exec_submit) | irq));

wire irq_en = sreg_priv_control_out[2], sreg_priv_mode = sreg_priv_control_out[0];
assign o_c_data_page = sreg_priv_control_out[1];

register sreg_irq_pc (.i_clk(i_clk), .i_rst(i_rst), .i_d(pc_val), .o_d(sreg_irq_pc_out), .i_ie(irq | sreg_irq_pc_ie));

wire sreg_jtr_buff_o, sreg_jtr_out;
wire jtr_jump_en = (sreg_irq_pc_ie | jump_dec_valid | c_sreg_irt) & exec_submit;
wire jtr_irqh_write = irq;
wire jtr_buff_in = (irq ? 1'b0 : sreg_in[0]);
wire jtr_in = (irq ? 1'b0 : sreg_jtr_buff_o);
register  #(.RESET_VAL(1'b1), .N(1)) sreg_jtr_buff (.i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_buff_in), .o_d(sreg_jtr_buff_o), .i_ie(sreg_jtr_ie | jtr_irqh_write));
register  #(.RESET_VAL(1'b1), .N(1)) sreg_jtr (.i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_in), .o_d(sreg_jtr_out), .i_ie(jtr_jump_en | jtr_irqh_write));
assign o_c_instr_page = sreg_jtr_out;

register sreg_scratch (.i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_in), .o_d(sreg_scratch_out), .i_ie(sreg_scratch_ie & exec_submit));

wire sreg_syscall;
register #(.N(1)) sreg_irq_flags (.i_clk(i_clk), .i_rst(i_rst), .i_d(prev_sys), .o_d(sreg_syscall), .i_ie(irq));

wire immu_write = c_sreg_store & exec_submit & (sr_bus_addr >= `RW'h100 && sr_bus_addr < `RW'h100 + 16); // flush after write to mmu is executed
wire flush_instr_mmu = (immu_write & o_c_instr_page) | ((jtr_in ^ sreg_jtr_out) & (jtr_jump_en | jtr_irqh_write)); // & exec_submit
always @(posedge i_clk)
    o_icache_flush <= flush_instr_mmu & ~i_rst;

assign sr_bus_addr = i_imm;
assign sr_bus_we = c_sreg_store & exec_submit;
assign sr_bus_data_o = sreg_in;

endmodule

`include "alu.v"
`include "rf.v"
`include "pc.v"