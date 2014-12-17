# martin ebner, igi, tu graz, austria
# this script builds and reloads the linux aer driver
# this script must run once for each new kernel
# runs on the (k)ubuntu and debian linux distributions

UNAME := $(shell uname -r)
KERNEL26 := 2.6
KERNELVERSION := $(findstring $(KERNEL26),$(UNAME))

all:: retina_test

ifeq ($(KERNELVERSION),2.6)

obj-m	:= retina.o

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)

all::
	rm -f *.ko *.o 
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
	rm -f *~ *.symvers *.o *.mod.c
	
else

TARGET	:= retina
CFLAGS	:= -O2 -Wall -DMODULE -D__KERNEL__ -DLINUX
CC	:= gcc

#builds retina driver for linux
all:: ${TARGET}.o

${TARGET}.o: ${TARGET}.c
	$(CC) $(CFLAGS) ${INCLUDE} -c ${TARGET}.c
	
endif

retina_test: retina_test.c
	gcc -o $@ $<

install: 
#ubuntu:        add line "retina" to /etc/modules [finished]
#other distros: load module at boot time (modprobe retina)
#               i.e. /etc/rc.3/S10retina.sh
#               linking to /etc/init.d/retina
#	@if lsmod|grep retina; then modprobe -r retina; fi;
	@cp retina.ko /lib/modules/`uname -r`/kernel/drivers/video
	@depmod -a
	@cp 88-retina.rules /etc/udev/rules.d
	@modprobe retina
	@if [ -x /etc/init.d/udev ] ; then /etc/init.d/udev restart; fi;
	@sleep 1
	@if [ -r /etc/modules ] ; then \
		if ! less /etc/modules | grep retina ; then \
			echo retina >> /etc/modules; \
		fi; \
		echo "autoload at system boot [ubuntu/debian]"; \
	fi;
#	@ls -l /dev/retina?
	@echo "retina driver module built for kernel `uname -r` and module installed.";
	