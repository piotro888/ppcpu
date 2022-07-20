#include "Vwishbone_arbiter.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

typedef Vwishbone_arbiter Vdut;

uint64_t sim_time = 0;

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
}

int main() {
    Vdut *dut = new Vdut;

    Verilated::traceEverOn(true);
    VerilatedVcdC *vcd = new VerilatedVcdC;
    dut->trace(vcd, 99);
    vcd->open("waveform.vcd");

    srand(4);

    cpu_reset(dut, vcd);
    bool prev_cyc = false;
    for(int i=0; i<100; i++) {
        int cyc0 = rand()&1;
        int cyc1 = rand()&1;
        dut->i_wb0_cyc = cyc0;
        dut->i_wb1_cyc = cyc1;
        tick(dut, vcd);

        if(dut->o_wb_cyc) { // granted
            if(!dut->o_sel_sig)
                assert(dut->i_wb0_cyc);
            else if(!prev_cyc)
                assert(dut->i_wb1_cyc && !dut->i_wb0_cyc);
            else
                assert(dut->i_wb1_cyc);
        }
        prev_cyc = dut->o_wb_cyc;   
    }

    dut->final();
    vcd->close();
    delete dut;

    std::cout<<"\nALL TESTS PASSED\n\n";
    return 0;
}
