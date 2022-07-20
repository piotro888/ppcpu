bn=$(basename $1)
dn=$(dirname $1)
verilator --cc -Irtl/ --exe --trace rtl/$1.v --exe test/$dn/./tb_$bn.cpp \
&& make -C obj_dir/ -f V$bn.mk V$bn \
&& obj_dir/V$bn