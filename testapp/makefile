#

Objs = vl_cmd.o vl_dial.o i2c_371.o

vl : $(Objs) ../vl53/libvl53.a
	qcc -Vgcc_ntox86 -ovl $(Objs) \
		-L/usr/qnx650/target/qnx6/x86/lib -L../vl53 \
		-lvl53 -lm

%.o: %.c
	qcc -Vgcc_ntox86 -c -Wc,-Wall -Wc,-Wno-parentheses -O -DNDEBUG \
		-I/usr/qnx630/target/qnx6/usr/include -c $< -o $@

