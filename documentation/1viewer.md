# Getting started with the viewer

Visualise a stream of events from the cameras / from a pre-recorded sequence.

## Description

This application demonstrates how to visualise a stream of address events either from the cameras or from a pre-recorded sequence. 
The event stream is transmitted from the cameras (/zynqGrabber/vBottle:o) to the vPreProcess (/vPreProcess/vBottle:i), that removes salt-and-pepper noise from the event stream. The filtered stream (/vPepper/vBottle:o) is sent to vFramer (/vFramer/AE:i), that converts it to a yarpview-able image. The "images" from left (/vFramer/left) and right camera (/vFramer/right) are then sent to the yarp viewers (/viewCh0 and /viewCh1).

Here is a visualisation of the instantiated modules and connections.

<img src="http://robotology.github.io/event-driven/doxygen/images/vView.png" width="480">

## Dependencies

No special dependencies are required, all the required modules will be executed by the application. 

## How to run the application

These are basic instructions for first time YARP users, assuming the comprehensive instructions have been followed.

* copy the application template into the yarpmanager search path, removing the .template extension

> cp $ICUBcontrib_DIR/share/ICUBcontrib/templates/applications/app_vView.xml.template ~/.local/share/yarp/applications/app_vView.xml

* download a dataset from the datasets section of the [tutorials page](http://robotology.github.io/event-driven/doxygen/doc/html/pages.html) and unpack to a location of your choosing.
* run a yarpserver

> yarpserver

* (in a seperate terminal) run a yarpdataplayer

> yarpdataplayer

* in the yarpdataplayer gui, open the downloaded datsets (File->open) by pressing "Choose" on the upper-level folder (e.g. folder name: fasthandtrim)
* the dataset should be open in the yarpdataplayer window with a "Port Name" of "/zynqGrabber/vBottle:o".
* (in a separate terminal) run a yarpmanager

> yarpmanager

* the yarpmanager gui should be open.
* on the entities tab (left) open the "Applications" folder - the vView application should be visible. Double click the vView application to load it.
* give the height and width of the camera used in the dataset (see the dataset specifications) to the vFramer by inserting in the Parameters section of the vFramer module in the yarpmodulemanager. For example:

> --height 128 --width 128

* run all the modules in the vView app by choosing "Run All" in the left-most vertical toolbar. All applications should become green.
* connect all yarp ports by choosing "Connect All" in the same toolbar. All connections should turn green.
* press "Play" on the yarpdataplayer. The dataset should be visible in the yarpview windows (split into left and right cameras).
* To close all applications first "Disconnect All" and then "Close All" on the left-hand toolbar. GUI's are closed as per a normal window. The yarpserver can be closed using "ctrl+c" in the appropriate terminal.

