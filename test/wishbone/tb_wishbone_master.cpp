#include "Vwishbone_master.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_TEST_TIME 200
uint64_t sim_time = 0;

void tick(Vwishbone_master *dut, VerilatedVcdC* vcd) {
    do {
        dut->i_clk ^= 1;
        dut->eval();
        vcd->dump(sim_time++);
    } while(dut->i_clk);

    std::cout<<".";
}

void cpu_reset(Vwishbone_master* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
}

void do_request(Vwishbone_master* dut, VerilatedVcdC* vcd, int addr, int data, bool we) {
    dut->i_mem_addr = addr;
    dut->i_mem_data = data;
    dut->i_mem_we = we;
    dut->i_mem_req = 1;
    tick(dut, vcd);
    dut->i_mem_req = 0;
}

void wb_respond(Vwishbone_master* dut, VerilatedVcdC* vcd, int data) {
    assert(dut->wb_cyc);
    assert(dut->wb_stb);
    dut->wb_ack = 1;
    dut->wb_i_dat = data;
    tick(dut, vcd);
    dut->wb_ack = 0;
}

bool test_timing(Vwishbone_master* dut, VerilatedVcdC* vcd) {
    cpu_reset(dut, vcd);

    // check timing
    do_request(dut, vcd, 1, 0, 0);
    assert(dut->wb_cyc);
    assert(dut->wb_stb);
    assert(!dut->wb_we);
    assert(dut->wb_adr == 1);
    wb_respond(dut, vcd, 2);
    assert(dut->o_mem_ack);
    assert(dut->o_mem_data == 2);

    // test fetch_next
    do_request(dut, vcd, 2, 0, 0);
    dut->i_mem_next = 1;
    dut->i_mem_addr = 3;
    assert(dut->wb_adr == 2);
    wb_respond(dut, vcd, 1);
    dut->i_mem_next = 0;
    assert(dut->wb_adr == 3);
    assert(dut->o_mem_ack);
    assert(dut->o_mem_data == 1);
    wb_respond(dut, vcd, 2);
    assert(dut->o_mem_ack);
    assert(dut->o_mem_data == 2);
    assert(!dut->wb_cyc);
    tick(dut, vcd);
    assert(!dut->o_mem_ack);
    
    std::cout<<"\ntest_wisbone_timing passed\n";
    return true;
}

bool test_rand(Vwishbone_master* dut, VerilatedVcdC* vcd) {
    cpu_reset(dut, vcd);

    int exp_adr, exp_n_adr = 0;
    for(int i=0; i<100; i++) {
        if(!dut->wb_cyc) {
            exp_adr = rand()%65535;
            do_request(dut, vcd, exp_adr, 0, 0);
            if(!(rand()%10)) {
                exp_n_adr = (rand()%65534)+1;
                dut->i_mem_next = 1;
                dut->i_mem_addr = exp_n_adr;
                tick(dut, vcd);
            }
        } else {
            while(rand()%2) 
                tick(dut, vcd);
            
            int data = rand()%65535;
            assert(dut->wb_adr == exp_adr);
            wb_respond(dut, vcd, data);
            assert(dut->o_mem_ack);
            assert(dut->o_mem_data == data);
            if(exp_n_adr) {
                dut->i_mem_next = 0;
                while(rand()%2) 
                    tick(dut, vcd);
                int data = rand()%65535;
                assert(dut->wb_adr == exp_n_adr);
                wb_respond(dut, vcd, data);
                assert(dut->o_mem_ack);
                assert(dut->o_mem_data == data);
                exp_n_adr = 0;
            }
        }
    }
    
    std::cout<<"\ntest_wisbone_rand passed\n";
    return true;
}

int main() {
    Vwishbone_master *dut = new Vwishbone_master;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    bool fail = false;
    srand(4);

    fail |= !test_timing(dut, v_vcd);
    fail |= !test_rand(dut, v_vcd);

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
