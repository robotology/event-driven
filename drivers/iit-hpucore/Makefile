#ccflags-m := -DDEBUG 
obj-m +=  iit-hpucore.o

ccflags-m := -DDEBUG 

all:
	make -C ~/Software/Linux/linux-kernel/ M=$(PWD) modules
clean:
	make -C ~/Software/Linux/linux-kernel/ M=$(PWD) clean
