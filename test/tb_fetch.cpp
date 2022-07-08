#include "Vfetch.h"
#include "Vfetch___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_SIM_TIME 200
uint64_t sim_time = 0;

int req_time = 0;
int i_seq = 1;

std::vector <int> res_ins;
int instructions[] = {0, 0xffff, 0x5000e, 0xfffe, 0xfffd, 0x8010e /*pred not taken*/, 0xfffa, 0x7000e, 0xfffb};
std::vector <int> exp_ins = {0, 0xffff, 0x5000e, 0x8010e, 0xfffa, 0x7000e, 0x7000e};

void sim_mem(Vfetch* d) {
    if(d->i_clk)
        return;
    
    d->i_req_data_valid = 0;
    if(d->o_req_active && !d->i_req_data_valid) {
        static int req_addr = -1;
        if(!req_time) {
            std::cout<<"NREQ "<<d->o_req_addr<<'\n';
            req_addr = d->o_req_addr;
        }
        
        if(++req_time == 3) {
            req_time = 0;
            d->i_req_data_valid = 1;
            assert(req_addr < 8);
            d->i_req_data = instructions[req_addr];
            std::cout<<"REQF "<<req_addr<<" r:"<<d->i_req_data<<"\n";
        }
    }
}

void sim_cpu(Vfetch* d) {
    if(d->i_clk)
        return;
    
    if(d->o_submit) {
        assert(d->i_next_ready);
        std::cout<<"PIPE "<<std::hex<<d->o_instr<<"(fetch_pc: "<<d->rootp->fetch__DOT__fetch_pc<<")\n";
        res_ins.push_back(d->o_instr);
    }

    if(!d->i_next_ready)
        std::cout<<"(NR)";

    d->i_next_ready = rand()%2;
}

void tick(Vfetch *dut, VerilatedVcdC* vcd) {
    dut->i_clk ^= 1;
    dut->eval();

    vcd->dump(sim_time++);
    if(dut->i_clk)
        std::cout<<".";
}

int main() {
    Vfetch *dut = new Vfetch;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;

    srand(1);
    dut->i_next_ready = 1;
    dut->i_flush = 0;

    while (sim_time < MAX_SIM_TIME) {
        // simulate external devices
        sim_mem(dut);
        sim_cpu(dut);
        // clock tick
        tick(dut, v_vcd);

        if(res_ins.size() == exp_ins.size())
            break;
    }

    dut->final();
    v_vcd->close();
    delete dut;

    std::cout<<'\n';
    // verify
    for(int i=0; i<exp_ins.size(); i++) {
        assert(res_ins.size() > i);
        assert(res_ins[i] == exp_ins[i]);
    }

    std::cout<<"\nfetch test passed\n";
}
