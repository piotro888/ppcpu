#include "Vcore.h"
#include "Vcore___024root.h"

#include <iostream>
#include <verilated_vcd_c.h>
#include <verilated.h>

#define MAX_TEST_TIME 200
uint64_t sim_time = 0;

std::map<unsigned int, unsigned int> instr_mem;
std::map<unsigned int, unsigned short> data_mem;

int req_time = 0;

void tick(Vcore *dut, VerilatedVcdC* vcd) {
    do {
        dut->i_clk ^= 1;
        dut->eval();
        vcd->dump(sim_time++);
    } while(dut->i_clk);

    std::cout<<".";
}

void cpu_reset(Vcore* dut, VerilatedVcdC* v_vcd) {
    dut->i_rst = 1;
    for(int i=0; i<4; i++)
        tick(dut, v_vcd);
    dut->i_rst = 0;
    req_time = 0;
}

void sim_mem(Vcore* d) {
    d->i_req_data_valid = 0;
    if(d->o_req_active && !d->i_req_data_valid) {
        static int req_addr = -1;
        if(!req_time)
            req_addr = d->o_req_addr;
        
        if(++req_time >= 2) {
            req_time = 0;
            d->i_req_data_valid = 1;
            d->i_req_data = instr_mem[req_addr];
            std::cout<<"REQF "<<std::hex<<req_addr<<" r:"<<d->i_req_data<<"\n";
        }
    }
}

int data_mem_req_time = 0;
int data_mem_req_addr = 0, data_mem_req_data = 0;
bool data_mem_req_we = false;
void sim_data_mem(Vcore* d) {
    d->i_mem_ack = 0;
    if(d->o_mem_req && !d->i_mem_ack) {
        if (!req_time) {
            data_mem_req_addr = d->o_mem_addr;
            data_mem_req_data = d->o_mem_data;
            data_mem_req_we = d->o_mem_we;
        }
        
        if(++data_mem_req_time >= 2) {
            data_mem_req_time = 0;
            d->i_mem_ack = 1;
            assert(data_mem_req_addr < 100);
            if(data_mem_req_we)
                data_mem[data_mem_req_addr] = d->o_mem_data;
            else
                d->i_mem_data = data_mem[data_mem_req_addr];
            std::cout<<"memreq a"<<data_mem_req_addr<<'d'<<(data_mem_req_we ? data_mem_req_data : d->i_mem_data)<<
                'w'<<data_mem_req_we<<'\n';
        }
    }
}

void load_hex() {
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

        instr_mem[std::stoul(addr, nullptr, 16)] = std::stoul(data, nullptr, 16);
    }
}

int main() {
    Vcore *dut = new Vcore;

    Verilated::traceEverOn(true);
    VerilatedVcdC *vcd = new VerilatedVcdC;
    dut->trace(vcd, 99);
    vcd->open("waveform.vcd");

    srand(8);
    load_hex();

    cpu_reset(dut, vcd);

    bool fail = true;
    while (sim_time <= MAX_TEST_TIME) {
        sim_mem(dut);
        sim_data_mem(dut);

        tick(dut, vcd);

        std::cout<<"r0:"<<dut->dbg_r0<<" pc:"<<dut->dbg_pc
            <<(dut->rootp->core__DOT__execute__DOT__exec_submit ? '<' : ' ')<<'\n';
        
        if(dut->dbg_pc == 0x1000) {
            fail = false;
            break;
        }
    }

    dut->final();
    vcd->close();

    if(fail)
        std::cerr<<"test failed\n";
    return fail;
}
