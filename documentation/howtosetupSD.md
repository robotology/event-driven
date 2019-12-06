
:warning: These instructions are the standard way to set-up the YARP/event-driven environment on an sd-card meant for the zcb or z-turn. If you already have a working sd-card you can simply copy the sd-card. You can find instructions for copying an sd-card at the [bottom](#how-copy-an-entire-sd-card-for-a-new-board).

# How to set-up an SD for a zynq board

:warning: These instructions are only needed if your SD card has a fresh OS install and the `event-driven` library is not yet compiled. These instructions shuold be performed on the ZCB/z-turn and not your own laptop.

## Set-up icub user

As **root**:
```bash
adduser icub
groups icub
visudo
```
Add line: `icub ALL=(ALL:ALL) ALL`

## Set up repositories

As **icub**:
- Follow the [installation instructions](full_installation.md) putting projects into `~/projects` and installing into `~/install`. You will need to make these folders in the home directory of icub.

### Note 1:
As only basic `YARP` support is needed, not all dependencies are required to be installed, instead install only:
```bash
sudo apt install [TODO]
```
### Note 2:
The newest YARP requires CMake>3.5, which is not installable via `apt` on the Debian 8.10 (jessie) distribution we have installed on the zynq. To upgrade CMake you need to install it via backports (reference: https://backports.debian.org/Instructions).
To do so:
- add to `/etc/apt/sources.list` the line below:
    ```bash
          deb http://ftp.debian.org/debian jessie-backports main
    ```
Then:
```bash
sudo apt update
sudo apt -t jessie-backports install cmake
```
At this point you should be able to recompile YARP 3.0 and `event-driven` **master** branch.

**We can consider updating the Debian distribution of the zynq boards since the Debian 8.10 is no longer supported by YARP**

### Note 3:

When installing `event-driven` use the following options for cmake:
```bash
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_HARDWAREIO=ON -DENABLE_zynqgrabber=ON
```
## Set up device drivers

As **icub**:
```bash
sudo usermod -a -G i2c icub
sudo vim /lib/udev/rules.d/77-iit-hpu.rules
```
Add lines:
```bash
SUBSYSTEM=="iit-hpu-class", GROUP="i2c"
```
Then
```bash
sudo vim /etc/rc.local
```
Add lines:
```bash
insmod /home/icub/iit-hpucore-dma.ko rx_pn=1024 rx_ps=8192 rx_to=5000
```
## Misc

Check the device driver meta data:
```bash
udevadm info -q all -a /dev/iit-hpu0
```
Check the device driver parameters:
```bash
cat /sys/module/iit_hpucore_dma/parameters/ps
```

# How copy an entire sd-card for a new board

## PARTITION THE NEW SD

* Insert the new SD
* `sudo gparted` (`sudo apt-get install gparted` if needed)
* `gparted` GUI should detect the SD
* Unmount the SD in gparted GUI (you cannot partition a mounted drive)
* Create new partitions: 1. FAT32 name:BOOT 50MiB 2. EXT4 name:rootfs (max-250) 3. linux-swap name:swap 200MiB
* Edit -> apply all operations

## COPY THE FILES

* Insert old SD (mount the boot and filesystem partitions)
* Copy BOOT (old) -> BOOT (new) (use `/tmp` as a temporary location to store files if you cannot mount both SD cards simultaneously)
* `sudo tar zcvf filesystem.tgz /media/$username/rootfs` (from the old SD - again do this in `/tmp`)
* `sudo sync` (ensure files are copied by flushing file writing queue)
* `cd /media/$username/rootfs` (on the new SD)
* `sudo tar zxvf /tmp/filesystem.tgz --strip-components=3`
* `sudo sync`
