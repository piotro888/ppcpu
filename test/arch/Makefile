ALL_TESTS=\
	fncall,\
	sreg0

ALL_TESTS_TRG=$(patsubst %, %.hex, $(ALL_TESTS))

.PHONY: test clean

test: $(ALL_TESTS_TRG) $(ALL_TESTS)

%: %.hex arch
	./arch < $<
	@echo ">> test $@ passed"

%.hex: %.s
	python3 ~/opt/pcpu-tools/pas.py $< -o $@

arch: arch.cpp
	verilator --cc -I../../rtl/ --exe --trace core.v arch.cpp \
	&& make -C obj_dir/ -f Vcore.mk Vcore \
	&& cp obj_dir/Vcore arch

clean:
	rm -f arch *.hex *.vcd
	rm -rf obj_dir/
