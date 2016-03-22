These drivers run on the Zturn to interface to hardware
iit-hpucore: event I/O from sensor, or to SpiNNaker board
iit-i2c_vsctrl: writing biases to sensors using i2c protocol

These drivers need to be cross-compiled for the arm processor which requires the Xilinx SDK and compiler. First add the correct binaries to the $PATH using:

source /opt/Xilinx/SDK/2014.4/settings64.sh

and then make the required driver using:

make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi-

The makefile needs to point to the Linux kernel source which has arm architecture support.
