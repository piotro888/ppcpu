#include "Vmemwb.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_SIM_TIME 200
uint64_t sim_time = 0;

int req_time = 0;
int i_seq = 1;

int mem_cell = 0;

void sim_mem(Vmemwb* d) {
    if(d->i_clk)
        return;
    
    d->i_mem_ack = 0;
    if(d->o_mem_req && !d->i_mem_ack) {
        static int req_data = -1, req_we = -1, req_addr = -1;
        if(!req_time) {
            req_addr = d->o_mem_addr;
            req_data = d->o_mem_data;
            req_we = d->o_mem_we;
        }
        
        if(++req_time >= rand()%3) {
            req_time = 0;
            d->i_mem_ack = 1;
            assert(req_addr == 10);
            if (req_we) {
                mem_cell = req_data; // in reality state would be busy for more cycles, but here
                              // we simulate that by delaying ack on random requests
            } else {
                d->i_mem_data = mem_cell;
            }
            std::cout<<"REQF "<<req_addr<<" "<<req_data<<" "<<req_we<<" "<<mem_cell<<"\n";
        }
    }
}

void tick(Vmemwb *dut, VerilatedVcdC* vcd) {
    do {
        dut->i_clk ^= 1;
        dut->eval();
        vcd->dump(sim_time++);
    } while(dut->i_clk);
    
    std::cout<<".";
    sim_mem(dut);

    if(sim_time > MAX_SIM_TIME) {
        dut->final();
        vcd->close();
        delete dut;
        assert(false);
    }
}

void cpu_reset(Vmemwb* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
}

void submit_request(Vmemwb* dut, VerilatedVcdC* vcd, int addr, int reg, int data, bool we, bool mem) {
    std::cout<<"submit";
    while (!dut->o_ready)
        tick(dut, vcd);
    
    dut->i_addr = addr;
    dut->i_dst_reg = reg;
    dut->i_data = data;
    dut->i_mem_access = mem;
    dut->i_reg_we = we && !mem;
    dut->i_mem_we = we && mem;
    dut->i_submit = 1;
    tick(dut, vcd);
    dut->i_submit = 0;
}

bool test_inc (Vmemwb* dut, VerilatedVcdC* vcd) {
    cpu_reset(dut, vcd);

    submit_request(dut, vcd, 0, 3, 1, 1, 0);
    assert(dut->o_reg_ie == 0x8);
    assert(dut->o_reg_data == 0x1);
    assert(dut->o_ready);
    assert(!dut->o_mem_req);
    submit_request(dut, vcd, 0, 3, 2, 1, 0);
    assert(dut->o_reg_ie == 0x8);
    assert(dut->o_reg_data == 0x2);
    assert(dut->o_ready);
    assert(!dut->o_mem_req);

    for(int i=0; i<10; i++) {
        int data = rand() % 65536;
        submit_request(dut, vcd, 10, 0, data, 1, 1);
        assert(!dut->o_reg_ie);
        submit_request(dut, vcd, 10, 2, 0, 0, 1);
        while(!dut->o_reg_ie)
            tick(dut, vcd);
        assert(dut->o_reg_ie == 0x4);
        assert(dut->o_reg_data == data);
        assert(dut->o_ready);
    }

    submit_request(dut, vcd, 10, 2, 0, 0, 0);
    assert(!dut->o_reg_ie);
    assert(dut->o_ready);

    return true;
}

int main() {
    Vmemwb *dut = new Vmemwb;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    bool fail = false;
    srand(3);

    fail |= !test_inc(dut, v_vcd);

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
