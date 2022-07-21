#include "Vmb_downconverter.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

typedef Vmb_downconverter Vdut;

uint64_t sim_time = 0;

int req_time = 0;
int req_addr = 0;
std::map <int, int> exp_data;

void settle(Vdut *d) {
    d->i_clk = 0;
    d->eval(); 
}

void tick(Vdut *d, VerilatedVcdC* vcd) {
    sim_time++;

    settle(d);
    vcd->dump(sim_time*10-1);

    d->i_clk = 1;
    d->eval();
    vcd->dump(sim_time*10);

    d->i_clk = 0;
    d->eval();
    vcd->dump(sim_time*10+5);

    vcd->flush();

    std::cout<<".";
}

void cpu_reset(Vdut* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
}

void sim_new_mem(Vdut* d) {
    d->d_req_data_valid = 0;
    if(d->d_req_active && (!d->d_req_data_valid || d->d_req_next)) {
        if (!req_time) {
            req_addr = d->d_req_addr;
            std::cout<<std::hex<<"new"<<' '<<req_addr<<'\n';
        }
        
        if(++req_time >= rand()%8) {
            req_time = 0;
            d->d_req_data_valid = 1;

            if(!exp_data.count(req_addr))
                exp_data[req_addr] = rand()%65535;

            d->d_req_data = exp_data[req_addr];
            std::cout<<std::hex<<"completed"<<' '<<req_addr<<' '<<d->d_req_data<<'\n';

            if(d->d_req_next && d->d_req_active) {
                req_addr = d->d_req_addr;
                req_time = 1;
                std::cout<<"next"<<req_addr;
            }
        }
    }
}


int main() {
    Vdut *d = new Vdut;

    Verilated::traceEverOn(true);
    VerilatedVcdC *vcd = new VerilatedVcdC;
    d->trace(vcd, 99);
    vcd->open("waveform.vcd");

    srand(5);

    cpu_reset(d, vcd);

    bool fail = false;
    for(int i=0; i<100; i++) {
        if(!d->u_req_active && !(rand() % 3)) {
            d->u_req_active = 1;
            d->u_req_addr = rand()%30000;
        }

        settle(d);
        sim_new_mem(d);

        settle(d);
        if(d->u_req_data_valid) {
            d->u_req_active = 0;
            int req_addr = (d->u_req_addr<<1);
            long long exp_instr = (exp_data[req_addr]) | ((long long) exp_data[req_addr+1]<<16ll);
            std::cout<<std::hex<<d->u_req_data<<' '<<exp_instr<<' '<<exp_data[req_addr]<<' '<<exp_data[req_addr+1]<<'\n';
            if(d->u_req_data != exp_instr) {
                std::cout<<"test failed\n";
                fail = true;
                break;
            }
        }

        tick(d, vcd);
    }

    d->final();
    vcd->close();
    delete d;
    
    if(!fail)
        std::cout<<"\nALL TESTS PASSED\n\n";
    return fail;
}
