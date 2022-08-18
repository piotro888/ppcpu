#include "Vupper_core.h"
#include "Vupper_core___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_TEST_TIME 200000
uint64_t sim_time = 0;
uint64_t test_time = 0;

#define PASS_MAGIC 0x400

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
            //std::cout<<"memreqf "<<std::hex<<d->wb_adr<<' '<<((int)d->wb_we ? 'w' : 'r')<<' '<<(d->wb_we ? d->wb_o_dat : d->wb_i_dat)<<'\n';
            std::cout<<std::hex<<d->wb_adr<<' ';
        } else {
            d->wb_ack = 0;
        }
    } else {
        d->wb_ack = 0;
    }
}

void load_bootjump() {
    // disables mem paging and jumps to 0 (0x800000)
    mem[0xffe000] = 0x0004;
    mem[0xffe001] = 0x0000;
    mem[0xffe002] = 0x0011;
    mem[0xffe003] = 0x0002;
    mem[0xffe004] = 0x000E;
    mem[0xffe005] = 0x0000;
}

void load_mem(std::vector<int>& instr) {
    load_bootjump();
    for(int i=0; i<instr.size(); i++) {
        mem[0x800000 + 2*i] = instr[i] & 0xffff;
        mem[0x800000 + 2*i + 1] = instr[i] >> 16;
    }
}

void load_hex() {
    load_bootjump();
    std::string line;
    while(std::cin>>line) {
        if(line == ":00000001FF")
            break;

        if(line.size() != 19 || line[0] != ':') {
            std::cerr<<"invalid input";
            exit(1);
        }

        std::string addr = line.substr(3, 4);
        std::string data = line.substr(9, 8);

        mem[0x800000 | (std::stoul(addr, nullptr, 16)<<1)] = std::stoul(data, nullptr, 16) & 0xffff;
        mem[0x800000 | ((std::stoul(addr, nullptr, 16)<<1)+1)] = std::stoul(data, nullptr, 16) >> 16;
        std::cout<<std::hex<<(0x800000 | (std::stoul(addr, nullptr, 16)<<1))<<" to "<<(std::stoul(data, nullptr, 16) & 0xffff)<<'\n';
    }
}

#define RF_REG(x) dut->rootp->upper_core__DOT__core__DOT__execute__DOT__rf__DOT____Vcellout__rf_regs__BRA__ ## x ## __KET____DOT__rf_reg__o_d

int main() {
    Vdut *dut = new Vdut;

    Verilated::traceEverOn(true);
    VerilatedVcdC *vcd = new VerilatedVcdC;
    dut->trace(vcd, 99);
    vcd->open("waveform.vcd");

    srand(8);
    cpu_reset(dut, vcd);

    load_hex();
    int prev_pc = -1;
    bool fail = true;
    while (sim_time <= MAX_TEST_TIME) {
        sim_wishbone_mem(dut);

        tick(dut, vcd);

        if(dut->dbg_pc != prev_pc) {
            prev_pc = dut->dbg_pc;
            std::cout<<'\n';
            std::cout<<" pc:"<<dut->dbg_pc<<' ';
            std::cout<<"r"<<0<<": "<<std::hex<<RF_REG(0)<<' ';
            std::cout<<"r"<<1<<": "<<std::hex<<RF_REG(1)<<' ';
            std::cout<<"r"<<2<<": "<<std::hex<<RF_REG(2)<<' ';
            std::cout<<"r"<<3<<": "<<std::hex<<RF_REG(3)<<' ';
            std::cout<<"r"<<4<<": "<<std::hex<<RF_REG(4)<<' ';
            std::cout<<"r"<<5<<": "<<std::hex<<RF_REG(5)<<' ';
            std::cout<<"r"<<6<<": "<<std::hex<<RF_REG(6)<<' ';
            std::cout<<"r"<<7<<": "<<std::hex<<RF_REG(7)<<' ';
        }
        
        if(dut->dbg_pc == PASS_MAGIC) {
            fail = false;
            break;
        }
    }

    dut->final();
    vcd->close();
    delete dut;

    std::cout<<'\n';
    if(fail)
        std::cout<<"TEST FAILED\n";
    else
        std::cout<<"TEST PASSED\n";
    std::cout<<'\n';
    return fail;
}
