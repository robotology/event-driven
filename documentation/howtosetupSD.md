
These instructions are the standard way to set-up the YARP/event-driven environment on an sd-card meant for the zcb or z-turn. If you already have a working sd-card you can simply copy the sd-card. You can find instructions for copying an sd-card at the bottom.

# How to set-up an SD for a zynq board

## Set-up icub user

(as root)

> adduser icub

> groups icub

> visudo

* add line: "icub ALL=(ALL:ALL) ALL"

> mkdir /usr/local/src/robot

> chown icub /usr/local/src/robot

## Set up repositories

(as icub)

> mkdir /usr/local/src/robot/icub-contrib-install

> vim ~/.bashrc

add lines:

* export ROBOT_CODE=/usr/local/src/robot
* export YARP_DIR=$ROBOT_CODE/yarp/build
* export ICUBcontrib_DIR=$ROBOT_CODE/icub-contrib-install
* export YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUBcontrib_DIR/share/ICUBcontrib
* export PATH=$YARP_DIR/bin:$ICUBcontrib_DIR/bin:$PATH
* export ED_ROOT=$ROBOT_CODE/event-driven

### Set up YARP

#### Important note:
The newest YARP requires CMake>3.5, which is not installable via apt on the Debian 8.10 (jessie) distribution we have installed on the zynq. To upgrade CMake you need to install it via backports (reference: https://backports.debian.org/Instructions). 
To do so:
 - add to /etc/apt/sources.list the line below:
      deb http://ftp.debian.org/debian jessie-backports main 
- sudo apt update
- sudo apt -t jessie-backports install cmake
 
At this point you should be able to recompile YARP 3.0 and event-driven master branch.
 
**We can consider updating the Debian distribution of the zynq boards since the Debian 8.10 is no longer supported by YARP**

> cd /usr/local/src/robot

> git clone https://github.com/robotology/yarp

> cd yarp

> mkdir build

> cd build

> ccmake ../

* set CREATE_GUIS = OFF

> make (not install)

### Set up install directory as $ICUBcontrib_DIR
> cd /usr/local/src/robot

> git clone https://github.com/robotology/icub-contrib-common

> cd icub-contrib-common

> mkdir build

> cd build

> ccmake ../

* set CMAKE_INSTALL_PREFIX=$ICUBcontrib_DIR

> make install


### Set up event-driven
> cd /usr/local/src/robot

> git clone https://github.com/robotology/event-driven

> cd event-driven

> mkdir build

> cd build

> ccmake ../

* (cmake should have found install directory as $ICUBcontrib_DIR automatically)
* BUILD_HARDWAREIO = ON

configure and then

* ENABLE_zynqgrabber = ON

> make install

## Set up device drivers

(as icub)

> sudo usermod -a -G i2c icub

> sudo vim /lib/udev/rules.d/77-iit-hpu.rules

add lines:

* SUBSYSTEM=="iit-hpu-class", GROUP="i2c"

> sudo vim /etc/rc.local

add lines:

* insmod $PATH_TO_HPU_DRIVER.ko ps=4096
* $PATH_TO_ZYNQGRABBER --biaswrite > /usr/local/src/robot/zynqGrabber.log

## Misc

check the device driver meta data

> udevadm info -q all -a /dev/iit-hpu0

check the device driver parameters

> cat /sys/module/iit_hpucore_dma/parameters/ps

# How copy an entire sd-card for a new board

## PARTITION THE NEW SD

* insert the new SD
* sudo gparted (sudo apt-get install gparted if needed)
* gparted GUI should detect the SD
* unmount the SD in gparted GUI (you cannot partition a mounted drive)
* create new partitions: 1. FAT32 name:BOOT 50MiB 2. EXT4 name:rootfs (max-250) 3. linux-swap name:swap 200MiB
* edit -> apply all operations

## COPY THE FILES

* insert old SD (mount the boot and filesystem partitions)
* copy BOOT (old) -> BOOT (new) (use /tmp as a temporary location to store files if you cannot mount both SD cards simultaneously)
* sudo tar zcvf filesystem.tgz /media/$username/rootfs (from the old SD - again do this in /tmp)
* sudo sync (ensure files are copied by flushing file writing queue)
* cd /media/$username/rootfs (on the new SD)
* sudo tar zxvf /tmp/filesystem.tgz --strip-components=3
* sudo sync



