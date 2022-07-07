#include "Vcore.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_SIM_TIME 200
uint64_t sim_time = 0;

int req_time = 0;
int i_seq = 1;

int instructions[] = {0, 0x00010004, 0x00020084, 0x00002007, 0x00000007, 0x00010008, 0x0000200A};

bool sim_mem(Vcore* d) {
    if(d->i_clk)
        return true;
    
    d->i_req_data_valid = 0;
    if(d->o_req_active && !d->i_req_data_valid) {      
        if(++req_time == 2) {
            req_time = 0;
            d->i_req_data_valid = 1;
            if (d->o_req_addr >= 7)
                return false;
            d->i_req_data = instructions[d->o_req_addr];
            std::cout<<"REQF "<<std::hex<<d->o_req_addr<<" r:"<<d->i_req_data<<"\n";
        }
    }
    return true;
}

void tick(Vcore *dut, VerilatedVcdC* vcd) {
    dut->i_clk ^= 1;
    dut->eval();

    vcd->dump(sim_time++);
    if(dut->i_clk)
        std::cout<<".";
}

int main() {
    Vcore *dut = new Vcore;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;


    while (sim_time < MAX_SIM_TIME) {
        // simulate external devices
        if(!sim_mem(dut))
            break;

        // clock tick
        tick(dut, v_vcd);

        if(!dut->i_clk)
            std::cout<<"r0:"<<dut->dbg_r0<<" pc:"<<dut->dbg_pc<<'\n';
    }

    dut->final();
    v_vcd->close();
    delete dut;

    std::cout<<'\n';
}
