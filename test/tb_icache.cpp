
#include "Vicache.h"
#include "Vicache___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

typedef Vicache Vdut;

uint64_t sim_time = 0;

int req_time = 0;
int req_addr = 0;
int burst_num = 0;

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
    req_time = 0;
}

void sim_mem(Vdut* d) {
    d->omem_ack = 0;
    if(d->omem_req) {
        if(!req_time) {
            req_time = (rand()%4)+1;
            req_addr = d->omem_addr;
            burst_num = 0;
        }
    
        if(--req_time <= 0) {
            assert(d->omem_burst4);
            req_time = (rand()%2)+1;
            d->omem_data = req_addr+burst_num;
            d->omem_ack = 1;
            std::cout<<"reqf"<<req_addr+burst_num<<'\n';
            if(++burst_num >= 4) {
                burst_num = 0;
                req_time = 0;
            }
        }
    }
}

int m_req_addr = 3;
int reqstime;
int main() {
    Vdut *d = new Vdut;

    Verilated::traceEverOn(true);
    VerilatedVcdC *vcd = new VerilatedVcdC;
    d->trace(vcd, 99);
    vcd->open("waveform.vcd");

    srand(5);

    cpu_reset(d, vcd);

    
    tick(d, vcd);
    for(int i=0; i<10000; i++) {
        bool first_cyc = false;
        if(d->imem_ack) {
            
            bool should_hit = (bool)(m_req_addr & 0x3);
            std::cout<<d->imem_data<<' '<<m_req_addr<<'\n';
            std::cout<<should_hit<<' '<<((bool)d->rootp->icache__DOT__cache_hit)<<'s'<<'\n';
            assert(d->imem_data == m_req_addr);
            if(should_hit)
                assert(sim_time-reqstime <= 4);
            else
                assert(sim_time-reqstime > 4);
        
            if(d->imem_next) {
                first_cyc = true;
                d->imem_req = 1;
                d->imem_addr = ++m_req_addr;
                reqstime = sim_time;
            } else {
                d->imem_req = 0;
            }

            d->imem_next = 0;
        }

        settle(d);

        if(d->imem_req && !d->imem_next && (rand()&2) && !first_cyc) {
            d->imem_next = 1;
            d->imem_addr = m_req_addr+1;
        }

        if(!d->imem_req && (rand()&1)) {
            first_cyc = true;
            d->imem_next = 0;
            d->imem_addr = ++m_req_addr;
            d->imem_req = 1;
            reqstime = sim_time;
        }

        tick(d, vcd);
        sim_mem(d);
        settle(d);
    }

    d->imem_req = 0;
    d->imem_next = 0;
    cpu_reset(d, vcd);
    tick(d, vcd);
    d->imem_req = 1;
    d->imem_next = 0;
    d->imem_addr = 4;
    do {
    tick(d, vcd);
    sim_mem(d);
    settle(d);
    d->imem_next = 1;
    d->imem_addr = 5;
    } while(!d->imem_ack);
    int req[] = {5, 6, 7, 4, 5};
    for(int i=0; i<5; i++) {
        d->imem_req = 1;
        d->imem_next = 1;
        d->imem_addr = req[i];
        tick(d, vcd);
        sim_mem(d);
        settle(d);
        if(i!=0) // delay after cache update
            assert(d->imem_ack);
    }
    d->imem_req = 1;
    d->imem_next = 0;
    d->imem_addr = 8;
    tick(d, vcd);
    sim_mem(d);
    settle(d);
    do {
    tick(d, vcd);
    sim_mem(d);
    settle(d);
    d->imem_next = 1;
    d->imem_addr = 9;
    } while(!d->imem_ack);
    for(int i=0; i<2; i++) {
        d->imem_req = 0;
        tick(d, vcd);
        sim_mem(d);
        settle(d);
    }

    int req2[] = {9, 6, 7, 8};
    for(int i=0; i<4; i++) {
        d->imem_req = 1;
        d->imem_next = 1;
        d->imem_addr = req2[i];
        tick(d, vcd);
        sim_mem(d);
        settle(d);
        if(i!=0) // delay after cache update
            assert(d->imem_ack);
    }

    for(int i=0; i<2; i++) {
        d->imem_req = 0;
        tick(d, vcd);
        sim_mem(d);
        settle(d);
        if(!i)
            assert(d->imem_ack);
    }

    d->final();
    vcd->close();
    delete d;
    
    std::cout<<"\nALL TESTS PASSED\n\n";
    return 0;
}
