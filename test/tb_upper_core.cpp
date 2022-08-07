#include "Vupper_core.h"
#include "Vupper_core___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_TEST_TIME 200
uint64_t sim_time = 0;
uint64_t test_time = 0;

int req_time = 0;

typedef Vupper_core Vdut;

std::map <int, unsigned short> mem;

void tick(Vdut *dut, VerilatedVcdC* vcd) {
    do {
        dut->i_clk ^= 1;
        dut->eval();
        vcd->dump(sim_time++);
    } while(dut->i_clk);

    std::cout<<".";
}

void cpu_reset(Vdut* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
    req_time = 0;
    mem.clear();
}

void sim_wishbone_mem(Vdut* d) {
    if(d->wb_cyc && d->wb_stb && !d->wb_ack) {
        if (!req_time) {
            std::cout<<"newmemreq"<<'\n';
            req_time = (rand()%3)+1;
        }

        if(--req_time <= 0) {
            d->wb_ack = 1;
            if(d->wb_we) {
                mem[d->wb_adr] = d->wb_o_dat;
            } else {
                d->wb_i_dat = mem[d->wb_adr];
            }
            req_time = 0;
            std::cout<<"memreqf "<<std::hex<<d->wb_adr<<' '<<(int)d->wb_we<<' '<<(d->wb_we ? d->wb_o_dat : d->wb_i_dat)<<'\n';
        } else {
            d->wb_ack = 0;
        }
    } else {
        d->wb_ack = 0;
    }
}

void load_mem(std::vector<int>& instr) {
    for(int i=0; i<instr.size(); i++) {
        mem[0x800000 + 2*i] = instr[i] & 0xffff;
        mem[0x800000 + 2*i + 1] = instr[i] >> 16;
    }
}

bool test_interupts(Vdut* dut, VerilatedVcdC* vcd) {
    // This test is executed as currenly the most complex one
    // take listing from tb_core.cpp
    std::vector <int> instr = {
        0x0002000E,
        0x0011000E,
        0x00000084,
        0x000c0004,
        0x00010011,
        0x00000004,
        0x00100104,
        0x00010008,
        0x00020005,
        0x03840004,
        0x00020002,
        0x0000000D,
        0x000E030E,
        0x03200004,
        0x0000400C,
        0x0007018E,
        0x0016000E,
        0x00040190,
        0x00010488,
        0x00040C11,
        0x00000000,
        0x0000001E,
        0x00000000,
        0x00050004,
        0x01000011,
        0x0016000E};
    cpu_reset(dut, vcd);
    load_mem(instr);
    
    test_time = sim_time;
    int irqc = 0;
    while (sim_time-test_time < MAX_TEST_TIME*200) {
        sim_wishbone_mem(dut);

        if(rand() % 100 < 5 && !dut->i_irq && dut->dbg_pc != 0x14) {
            std::cout<<"irq\n";
            irqc++;
            dut->i_irq = 1;
        }

        tick(dut, vcd);
        
        if(dut->dbg_pc == 0x14) {
            std::cout<<"cli\n";
            dut->i_irq = 0; // cli
        }

        if(dut->dbg_pc >= instr.size()-1) {
            std::cout<<"end";
            break;
        }
    }

    int r0 = dut->dbg_r0;
    int r1 = dut->rootp->upper_core__DOT__core__DOT__execute__DOT__rf__DOT____Vcellout__rf_regs__BRA__1__KET____DOT__rf_reg__o_d;
    std::cout<<"r0: "<<r0<<" r1: "<<r1<<"  ex:"<<irqc<<'\n';
    if(r0 != 0x10 || r1 != irqc) {
        std::cout<<"test_interrupt failed\n";
        return false;
    }
    std::cout<<"test_interrupt passed\n";
    return true;
}

int main() {
    Vdut *dut = new Vdut;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    bool fail = false;
    srand(3);

    fail |= !test_interupts(dut, v_vcd);

    dut->final();
    v_vcd->close();
    delete dut;

    std::cout<<'\n';
    if(fail)
        std::cout<<"SOME TESTS FAILED\n";
    else
        std::cout<<"ALL TESTS PASSED\n";
    std::cout<<'\n';
    return fail;
}
