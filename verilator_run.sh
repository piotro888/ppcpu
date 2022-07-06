verilator --cc -Irtl/ --exe --trace rtl/$1.v --exe test/tb_$1.cpp \
&& make -C obj_dir/ -f V$1.mk V$1 \
&& obj_dir/V$1 \
&& [ "$2" = "t" ] && gtkwave waveform.vcd