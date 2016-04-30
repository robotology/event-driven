#ccflags-m := -DDEBUG 
obj-m +=  iit-i2c_vsctrl.o

ccflags-m := -DDEBUG 

all:
	make -C ../../../LinuxXcompile/linux-kernel/ M=$(PWD) modules
clean:
	make -C ../../../LinuxXcompile/linux-kernel/ M=$(PWD) clean
