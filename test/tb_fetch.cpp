#include "Vfetch.h"
#include "Vfetch___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_SIM_TIME 200
uint64_t sim_time = 0;

int req_time = 0;
int req_resp_time = 0;
int i_seq = 1;

std::vector <int> res_ins;
int instructions[] = {0, 0xffff, 0x5000e, 0x0000e, 0xfffd, 0x8010e /*pred not taken*/, 0xfffa, 0x7000e, 0xfffb};
std::vector <int> exp_ins = {0, 0xffff, 0x5000e, 0x8010e, 0xfffa, 0x7000e, 0x7000e};
int exp_by_idx[9] = {0xffff, 0x5000e, 0x8010e, 0x0, 0x8010e, 0xfffa, 0x7000e, 0xfffb, -1};
int next_exp_insn = -2;

void sim_mem(Vfetch* d) {
    if(d->i_clk)
        return;
    
    d->i_req_data_valid = 0;
    if(d->o_req_active && !d->i_req_data_valid) {
        static int req_addr = -1;
        if(!req_time) {
            std::cout<<"NREQ "<<d->o_req_addr<<'\n';
            req_addr = d->o_req_addr;
            req_resp_time = (rand()%3) + 1;
        }
        
        if(++req_time >= req_resp_time) {
            req_time = 0;
            d->i_req_data_valid = 1;
            assert(req_addr < 8);
            d->i_req_data = instructions[req_addr];
            std::cout<<"REQF "<<req_addr<<" r:"<<d->i_req_data<<"\n";
        }
    }
}

bool first_flush = true;
int flush_rand_delay_by = 0;
void sim_cpu(Vfetch* d, bool flush_test=false) {
    if(d->i_clk)
        return;
    
    if(d->o_submit) {
        assert(d->i_next_ready);
        std::cout<<"PIPE "<<std::hex<<d->o_instr<<"(fetch_pc: "<<d->rootp->fetch__DOT__fetch_pc<<")\n";
        res_ins.push_back(d->o_instr);
        if(flush_test) {
            std::cout<<d->o_instr<<' '<<next_exp_insn<<"<<"<<'\n';
            flush_rand_delay_by = sim_time + (rand()%6);
            assert(d->o_instr == next_exp_insn || next_exp_insn < 0);
        }
    }

    if(flush_test && sim_time <= flush_rand_delay_by) {
        flush_rand_delay_by = 0;
        d->i_flush = rand()%2;
        d->i_exec_pc = rand()%4;
        if(d->i_flush && first_flush) {
            first_flush = false;
            d->i_exec_pc = 3;
        }
        if(d->i_flush) {
            next_exp_insn = instructions[d->i_exec_pc];
        } else {
            next_exp_insn = exp_by_idx[d->rootp->fetch__DOT__fetch_pc];
        }
        std::cout<<"flush stat"<<(bool)d->i_flush<<"x"<<d->i_exec_pc<<'\n';
    }

    if(!d->i_next_ready)
        std::cout<<"(NR)";
}

void tick(Vfetch *dut, VerilatedVcdC* vcd) {
    do {
        dut->i_clk ^= 1;
        dut->eval();
        vcd->dump(sim_time++);
    } while(dut->i_clk);

    std::cout<<".";
}

void cpu_reset(Vfetch* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
    req_time = 0;
}

void test_run_flush(Vfetch *dut, VerilatedVcdC* vcd) {
    cpu_reset(dut, vcd);
    res_ins.clear();
    while (sim_time < MAX_SIM_TIME*2) {
        // simulate external devices
        sim_mem(dut);
        sim_cpu(dut, true);

        dut->i_next_ready = rand()%2;

        // clock tick
        tick(dut, vcd);
        dut->i_flush = 0; // one cycle signal

        if(next_exp_insn == -1)
            break;
    }

    std::cout<<"\nfetch flush_test passed\n";
}

int main() {
    Vfetch *dut = new Vfetch;

    Verilated::traceEverOn(true);
    VerilatedVcdC *v_vcd = new VerilatedVcdC;
    dut->trace(v_vcd, 99);
    v_vcd->open("waveform.vcd");

    cpu_reset(dut, v_vcd);

    srand(1);
    dut->i_next_ready = 1;
    dut->i_flush = 0;

    for(int r=0; r<3; r++) {
        cpu_reset(dut, v_vcd);
        res_ins.clear();
        while (sim_time < MAX_SIM_TIME) {
            // simulate external devices
            sim_mem(dut);
            sim_cpu(dut);

            if(r >= 1)
                dut->i_next_ready = rand()%2;
            else
                dut->i_next_ready = 1;

            // clock tick
            tick(dut, v_vcd);

            if(res_ins.size() == exp_ins.size())
                break;
        }
        // verify
        for(int i=0; i<exp_ins.size(); i++) {
            assert(res_ins.size() > i);
            assert(res_ins[i] == exp_ins[i]);
        }

        std::cout<<"\nfetch test run"<<r<<"passed\n";
    }

    test_run_flush(dut, v_vcd);

    dut->final();
    v_vcd->close();
    delete dut;

    std::cout<<'\n';
}
