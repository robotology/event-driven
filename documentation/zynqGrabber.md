# How to configure and run `zynqGrabber`

`zynqGrabber` is an application that runs on the ZCB or z-turn which forms a bridge between the FPGA (reading events from the sensors), and the `YARP` network, such that the events can be processed on one or more CPUs or neuromorphic hardware. To run the `zynqGrabber`:
-  the ZCB needs to be running, or connected to, a [`yarpserver`](setup_yarpserver.md),
-  `event-driven` needs to be installed correctly with [`zynqGrabber ENABLED`](howtosetupSD.md), and
-  the configuration file needs to be configured for the sensors connected to the hardware.

## `zynqGrabber` configuration file

When `event-driven` is installed (`make install`), it installs also the configuration files for the `zynqGrabber`. However the default values in the file need to be modified depending on the hardware that you want to use. 

:warning: We need to make a local copy of the file such that re-installing `event-driven` won't overwrite your local changes with the default ones!

First of all make an [SSH connection](connect_to_zcb.md) to the ZCB. You can modify the local configuration file using:
```bash
nano ~/.local/share/yarp/contexts/event-driven/zynqGrabber.ini
```
If the file/folder do not exist you must first import the local copy from the installed copy. Run the following command to automatically import a local copy of the configurations file and re-try modifying the file:
```bash
yarp-config context --import event-driven zynqGrabber.ini
```
The `zynqGrabber` has the following options:
* *name* : renames the ports opened, needed if more than one zynqGrabber is running on the same network
* *verbose* : print out a little more information on start-up
* *aps* : turn on the ATIS aps events
* *dataDevice* : the full path to the device that will be opened to read events
* *hpu_read* : open a thread to read data from the HPU and publish it to `YARP`
* *hpu_write* : open a thread to write data to the HPU device
* *packet_size* : the maximum number of events to send in a single packet (too small can lead to latency, too large can lead to packet-loss in UDP connections)
* *visCtrlLeft* : connect to a device to configure the left camera
* *visCtrlRight* : connect to a device to configure the right camera
* *skinCtrl* : connect to a device to configure the skin
* *use_spinnaker* : configure HPU to write to a SpiNNaker device
* *[ATIS_BIAS_LEFT]* : bias values for the left camera
* *[ATIS_BIAS_RIGHT]* : bias values for the right camera
* *[SKIN_CNFG]* : configuration parameters for the skin

Most default values are fine for use of `event-driven` sensors. However, the hardware you have means that the file will need to be configured.

* Comment out any devices you don't have connected.
* Set *use_spinnaker* and the HPU device to *hpu_read* and *hpu_write* if connected to the SpiNNaker.
* Change the name if this is not the only `zynqGrabber` on the network

## Run the `zynqGrabber`

Once the configuration has been correctly performed, you can run the `zynqGrabber` from the terminal:
```bash
zynqGrabber
```
If `zynqGrabber` doesn't open it should give a warning message stating the problem, typically `yarpserver` is not running/connected or it is trying to open a device that is not physically connected.

## Check the zynqGrabber is streaming data

On your own laptop that has a [connection to the ZCB `yarpserver`](setup_yarpserver.md) you can run:
```bash
yarp read ... /zynqGrabber/AE:o
```
to verify that the data is streaming. The data is not in a human-readable format but it's presence indicates that the `zynqGrabber` is working.
