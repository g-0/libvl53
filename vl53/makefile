#

Objs = vl53_api.o vl53_core.o vl53_sigma.o vl53_cal.o vl53_strings.o

vl53lib.o : $(Objs)
	ar cr libvl53.a $(Objs)
#	qcc -Vgcc_ntox86 -ove $(Objs) -L/usr/qnx650/target/qnx6/x86/lib

%.o: %.c
	qcc -Vgcc_ntox86 -c -Wc,-Wall -Wc,-Wno-parentheses -O -DNDEBUG \
		-I/usr/qnx630/target/qnx6/usr/include -c $< -o $@

