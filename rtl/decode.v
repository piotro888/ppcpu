`include "config.v"

module decode (
    input wire i_clk,
    input wire i_rst,

    input [15:0] i_instr_l,
    input [`I_SIZE-17:0] i_imm_pass,
    output reg [`I_SIZE-17:0] o_imm_pass,

    // Pipeline control
    input i_next_ready,
    output o_ready,
    output o_submit,

    output reg oc_pc_inc, oc_pc_ie,
    output reg oc_r_bus_imm,
    output reg [`ALU_MODE_W-1:0] oc_alu_mode,
    output reg oc_alu_carry_en, oc_alu_flags_ie,
    output reg [`REGNO_LOG-1:0] oc_l_reg_sel, oc_r_reg_sel, 
    output reg [`REGNO-1:0] oc_rf_ie
);

// COMBINATIONAL INSTRUCTION DECODER

`define OPC_NOP 7'h0
`define OPC_MOV 7'h1
`define OPC_LDI 7'h4
`define OPC_ADD 7'h7
`define OPC_ADI 7'h8
`define OPC_ADC 7'h9
`define OPC_SUB 7'ha
`define OPC_SUC 7'hb
`define OPC_CMP 7'hc
`define OPC_CMI 7'hd
`define OPC_JMP 7'he

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

always @(*) begin
    // defaults
    c_pc_inc = 1'b1;
    {c_pc_ie, c_r_bus_imm, c_alu_carry_en, c_alu_flags_ie} = 4'b0;
    c_rf_ie = `REGNO'b0;
    c_alu_mode = `ALU_MODE_W'b0;
    c_l_reg_sel = `REGNO_LOG'b0;
    c_r_reg_sel = `REGNO_LOG'b0;
    
    case (opcode)
        `OPC_NOP: begin
        end
        `OPC_MOV: begin
            c_alu_mode = `ALU_MODE_L_PASS;
            c_l_reg_sel = reg_st;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_LDI: begin
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_ADD: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_ADI: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_ADC: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_carry_en = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_SUB: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_SUC: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_r_bus_imm = 1'b1;
            c_alu_carry_en = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_CMP: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_flags_ie = 1'b1;
        end
        `OPC_CMI: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_flags_ie = 1'b1;
        end
        // NOTE: Conditional jumps should receive taken/not from fetch stage or decoded in ex.
        default: begin
        end
    endcase
end


// PIPELINE WRAPPER
always @(posedge i_clk) begin
    // pc reset values?
    if (i_next_ready & ~i_rst) begin
        o_submit <= 1'b1;

        o_imm_pass <= i_imm_pass;
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
    end else
        o_submit <= 1'b0;
    o_ready <= i_next_ready & ~i_rst;
end


endmodule