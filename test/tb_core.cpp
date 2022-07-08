#include "Vcore.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_SIM_TIME 200
uint64_t sim_time = 0;

int req_time = 0;
int i_seq = 1;

bool sim_mem(Vcore* d, std::vector<int>& instr, bool randt=false) {
    if(d->i_clk)
        return true;
    
    d->i_req_data_valid = 0;
    if(d->o_req_active && !d->i_req_data_valid) {
        // TODO: randomize request times
        if(++req_time >= (randt ? (rand()%3)+1 : 2)) {
            req_time = 0;
            d->i_req_data_valid = 1;
            if (d->o_req_addr >= instr.size()) {
                d->i_req_data = 0; // NOP
            } else {
                d->i_req_data = instr[d->o_req_addr];
            }
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

void cpu_reset(Vcore* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
}

bool test_simple (Vcore* dut, VerilatedVcdC* v_vcd) {
    cpu_reset(dut, v_vcd);
    /* listing
     * ldi r0, 1
     * ldi r1, 2
     * add r0, r0, r1
     * add r0, r0, r0
     * adi r0, r0, 1
     * sub r0, r0, r1
     */
    std::vector <int> instr = {0, 0x00010004, 0x00020084, 0x00002007, 0x00000007, 0x00010008, 0x0000200A};
    while (sim_time < MAX_SIM_TIME) {
        // simulate external devices
        if(!sim_mem(dut, instr, true))
            break;

        // clock tick
        tick(dut, v_vcd);

        if(!dut->i_clk)
            std::cout<<"r0:"<<dut->dbg_r0<<" pc:"<<dut->dbg_pc<<'\n';
        
        if(!dut->i_clk && dut->dbg_pc >= instr.size())
            break;
    }

    if(dut->dbg_r0 != 5) {
        std::cout<<"test_simple failed\n";
        return false;
    }
    std::cout<<"test_simple passed\n";
    return true;
}

bool test_mispredict (Vcore* dut, VerilatedVcdC* v_vcd) {
    cpu_reset(dut, v_vcd);
    /* listing
    ldi r0, 1
    jmp b
    mp:
    ldi r0, 4
    jmp mp
    b:
    cmi r0, 1
    jne mp ; MISPREDICTION (pred taken)
    c:
    ldi r0, 8
    jmp c
    ldi r0, 5
    nop
    */

    std::vector <int> instr = {
    0x0000010004,
    0x000004000E,
    0x0000040004,
    0x000002000E,
    0x000001000D,
    0x000002038E,
    0x0000080004,
    0x000006000E,
    0x0000050004,
    0x0000000000};

    while (sim_time < MAX_SIM_TIME) {
        // simulate external devices
        if(!sim_mem(dut, instr))
            break;

        // clock tick
        tick(dut, v_vcd);

        if(!dut->i_clk)
            std::cout<<"r0:"<<dut->dbg_r0<<" pc:"<<dut->dbg_pc<<'\n';
        
        if(!dut->i_clk && dut->dbg_pc >= instr.size())
            break;
    }

    if(dut->dbg_r0 != 8) {
        std::cout<<"test_mispredict failed\n";
        return false;
    }
    std::cout<<"test_mispredict passed\n";
    return true;
}


int main() {
    Vcore *dut = new Vcore;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    bool fail = false;
    srand(2);

    fail |= !test_simple(dut, v_vcd);
    fail |= !test_mispredict(dut, v_vcd);

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
