`include "config.v"

module decode (
    input wire i_clk,
    input wire i_rst,

    input [15:0] i_instr_l,
    input [`I_SIZE-17:0] i_imm_pass,
    output reg [`I_SIZE-17:0] o_imm_pass,
    input i_jmp_pred_pass,
    output reg o_jmp_pred_pass,

    // Pipeline control
    input i_next_ready,
    input i_submit,
    output o_ready,
    output reg o_submit,
    input i_flush,

    output reg oc_pc_inc, oc_pc_ie,
    output reg oc_r_bus_imm,
    output reg [`ALU_MODE_W-1:0] oc_alu_mode,
    output reg oc_alu_carry_en, oc_alu_flags_ie,
    output reg [`REGNO_LOG-1:0] oc_l_reg_sel, oc_r_reg_sel, 
    output reg [`REGNO-1:0] oc_rf_ie,
    output reg [`JUMP_CODE_W-1:0] oc_jump_cond_code,
    output reg oc_mem_access, oc_mem_we,
    output reg [1:0] oc_used_operands,
    output reg oc_sreg_load, oc_sreg_store, oc_sreg_jal_over, oc_sreg_irt
);

// COMBINATIONAL INSTRUCTION DECODER

`define OPC_NOP 7'h00
`define OPC_MOV 7'h01
`define OPC_LDD 7'h02
`define OPC_LDO 7'h03
`define OPC_LDI 7'h04
`define OPC_STD 7'h05
`define OPC_STO 7'h06
`define OPC_ADD 7'h07
`define OPC_ADI 7'h08
`define OPC_ADC 7'h09
`define OPC_SUB 7'h0a
`define OPC_SUC 7'h0b
`define OPC_CMP 7'h0c
`define OPC_CMI 7'h0d
`define OPC_JMP 7'h0e
`define OPC_JAL 7'h0f
`define OPC_SRL 7'h10
`define OPC_SRS 7'h11
`define OPC_IRT 7'h1e

wire [6:0] opcode = i_instr_l[6:0];
wire [2:0] reg_dst = i_instr_l[9:7];
wire [2:0] reg_st = i_instr_l[12:10];
wire [2:0] reg_nd = i_instr_l[15:13];

// Comb output signals
reg c_pc_inc, c_pc_ie;
reg c_r_bus_imm;
reg [`ALU_MODE_W-1:0] c_alu_mode;
reg c_alu_carry_en, c_alu_flags_ie;
reg [`REGNO_LOG-1:0] c_l_reg_sel, c_r_reg_sel; 
reg [`REGNO-1:0] c_rf_ie;
reg [`JUMP_CODE_W-1:0] c_jump_cond_code;
reg c_mem_access, c_mem_we;
reg [1:0] c_used_operands;
reg c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt;

always @(*) begin
    // defaults
    c_pc_inc = 1'b1;
    {c_pc_ie, c_r_bus_imm, c_alu_carry_en, c_alu_flags_ie, c_mem_access,
        c_mem_we, c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt} = 10'b0;
    c_rf_ie = `REGNO'b0;
    c_alu_mode = `ALU_MODE_W'b0;
    c_l_reg_sel = `REGNO_LOG'b0;
    c_r_reg_sel = `REGNO_LOG'b0;
    c_jump_cond_code = `JUMP_CODE_W'b0;
    c_used_operands = 2'b0;
    
    case (opcode)
        `OPC_NOP: begin
        end
        `OPC_MOV: begin
            c_alu_mode = `ALU_MODE_L_PASS;
            c_l_reg_sel = reg_st;
            c_rf_ie[reg_dst] = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_LDD: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
        end
        `OPC_LDO: begin
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_LDI: begin
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_STD: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_STO: begin
            c_l_reg_sel = reg_nd;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ADD: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ADI: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_ADC: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_carry_en = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SUB: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SUC: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_r_bus_imm = 1'b1;
            c_alu_carry_en = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_CMP: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_CMI: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_JMP: begin
            // NOTE: Conditional jumps are decoded in execute stage
            // to not unnecessarily stall the pipeline each time prediction
            // is taken
            c_pc_inc = 1'b0;
            c_pc_ie = 1'b0;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_jump_cond_code = {1'b1, i_instr_l[10:7]};
        end
        `OPC_JAL: begin
            c_pc_ie = 1'b0;
            c_pc_inc = 1'b0;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_sreg_jal_over = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            // set jump code to unconditional jump, pred is set by fetch
            c_jump_cond_code = {1'b1, 4'b0};
        end
        `OPC_SRL: begin
            c_sreg_load = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_SRS: begin
            c_r_reg_sel = reg_st;
            c_sreg_store = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_IRT: begin
            c_sreg_irt = 1'b1;
            c_pc_ie = 1'b0;
            c_pc_inc = 1'b0;
        end
        default: begin
        end
    endcase
end


// PIPELINE WRAPPER
reg input_valid;

// ready signal must be combinational to be registered on time by previous stage
assign o_ready = i_next_ready;

always @(posedge i_clk) begin
    if(i_rst) begin
        o_submit <= 1'b0;
        input_valid <= 1'b0;
    end else begin
        // default bubble
        o_submit <= 1'b0;

        if (i_next_ready & (i_submit | input_valid) & ~i_flush) begin
            o_submit <= 1'b1;
            input_valid <= 1'b0;

            o_imm_pass <= i_imm_pass;
            o_jmp_pred_pass <= i_jmp_pred_pass;
            // Submit control signals
            oc_pc_inc <= c_pc_inc;
            oc_pc_ie <= c_pc_ie; 
            oc_r_bus_imm <= c_r_bus_imm;
            oc_alu_carry_en <= c_alu_carry_en;
            oc_alu_flags_ie <= c_alu_flags_ie;
            oc_rf_ie <= c_rf_ie;
            oc_alu_mode <= c_alu_mode;
            oc_l_reg_sel <= c_l_reg_sel;
            oc_r_reg_sel <= c_r_reg_sel;
            oc_jump_cond_code <= c_jump_cond_code;
            oc_mem_access <= c_mem_access;
            oc_mem_we <= c_mem_we;
            oc_used_operands <= c_used_operands;
            oc_sreg_load <= c_sreg_load;
            oc_sreg_store <= c_sreg_store;
            oc_sreg_jal_over <= c_sreg_jal_over;
            oc_sreg_irt <= c_sreg_irt;
        end

        if (i_submit & ~i_next_ready & ~i_flush) begin
            input_valid <= 1'b1; // don't overwrite buffer
        end

        if (i_flush) begin
            o_submit <= 1'b0;
            input_valid <= 1'b0;
        end 
    end
end


endmodule
