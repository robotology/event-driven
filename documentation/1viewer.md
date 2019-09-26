# Testing your install with the viewer

Visualise a stream of events either from real hardware/camera or from a pre-recorded sequence.

## Description

This application demonstrates how to visualise a stream of address events either from the cameras or from a pre-recorded sequence.
The event stream is transmitted from the cameras (`/zynqGrabber/AE:o`) to the vPreProcess (`/vPreProcess/AE:i`), that removes salt-and-pepper noise from the event stream. The filtered stream (`/vPreProcess/left:o` and `/vPreProcess/right:o`) is sent to vFramer (`/vFramer/left/AE:i`), that converts it to a yarpview-able image. The "images" from left (`/vFramer/left/image:o`) and right camera (`/vFramer/right/image:o`) are then sent to the yarp viewers (`/viewCh0` and `/viewCh1`).

Here is a visualisation of the instantiated modules and connections.

<img src="http://robotology.github.io/event-driven/doxygen/images/vView.png" width="480">

## How to run the application

These are basic instructions for first time YARP users, assuming the [comprehensive instructions](full_installation.md) have been followed.

* Download the sample dataset from [here](https://doi.org/10.5281/zenodo.2556755) and unpack to a location of your choosing.

* Set the yarp namespace:
```bash
yarp namespace /<yourname>
```
* Run a yarpserver:
```bash
yarpserver --write
```
### Using actual hardware/camera? Skip [ahead](#using-real-hardwarecamera). Using a dataset? Follow these instructions:

* In a separate terminal, run a yarpdataplayer:
```bash
yarpdataplayer
```
* In the yarpdataplayer gui, open the downloaded datsets (File->open) by pressing "Choose" on the upper-level folder (e.g. folder name: fasthandtrim).
* The dataset should be open in the yarpdataplayer window with a "Port Name" of `/zynqGrabber/AE:o`.

### Using real hardware/camera?

* You should have followed the instructions [to run install hardware and run zynqGrabber](README.md). 
* if everything went smoothly, your zynqGrabber should be running and a port `/zynqGrabber/AE:o` should be already open (check with `yarp name list`).

### Okay - yarpdataplayer users, and hardware users back together here:
* In a separate terminal, run a yarpmanager:
```bash
yarpmanager
```
* The yarpmanager gui should be open.
* On the entities tab (left) open the "Applications" folder - the vView application should be visible. Double click the `event-viewer-example` application to load it.
* If you do not have the `event-viewer-example` application, make sure you followed the installation steps correctly, and that your `YARP_DATA_DIRS` environment variable correctly points to the share folders in your install folder (`echo $YARP_DATA_DIRS`)
* Run all the modules in the `event-viewer-example` app by choosing "Run All" in the left-most vertical toolbar. All applications should become green.
* If not all applications are green, it means the executable files could not be found on the `PATH`. Verfify your installation and your `PATH` environment variable (`echo $PATH`).
* Connect all yarp ports by choosing "Connect All" in the same toolbar. All connections should turn green.
* Press "Play" on the yarpdataplayer. The dataset should be visible in the yarpview windows (split into left and right cameras).
* To close all applications first "Disconnect All" and then "Close All" on the left-hand toolbar. GUI's are closed as per a normal window. The yarpserver can be closed using `ctrl+c` in the appropriate terminal.