# Event-driven camera calibration

The following procedure is used to calibrate the event-camera intrinsic parameters, as well as the stereo extrinsic parameters to a second camera, being either event-based or traditional RGB. The most complex prodecure or stereo calibration to RGB camera is described first, and the simplification to achieve mono, or event-driven stereo pair are described afterwards. The calibration follows the classic procedure detecting a predefined checkerboard pattern placed in front of the cameras.

## Requirements

* YARP (sending receiving events and images)
* icub-main (the main calibration software which wraps opencv)
* event-driven (event reading and forming images)
* RGB and/or event-based camera able to connect with Yarp
* tablet (prefered) or laptop/portable screen

## Method of calibration

### The checkerboard pattern

The calibration method described uses the stardard checkerboard pattern seen from multiple different viewpoints in front of the camera. However, a fiducial that can be simultaneously detected as a checkerboard from both the event- and RGB-camera is required to enable the stereo calibration. To do so we have provided a flashing video file, which flashes fast enough to be ignored by an RGB camera, and is tuned to the event visualisation method to produce also a reliable checkerboard image also from the event-camera. The [checkerboard video](video_checkboard.mp4) should be played back on a portable screen, we suggest a tablet/pad or possibly a lightweight laptop/monitor. Depending on the screen type the video can be used in two ways:

* Use the screen refresh rate: set the screen refresh to 60 Hz and play and pause the video. The screen refresh rate itself may be detectable by the event-camera and RGB-camera without anything further required.
* Use the video flash: some screens may not refresh in a compatible way, in which case the video must be played on repeat/loop to achieve the desired effect. 

:warning: in either of these cases, it may be necessary to play with the screen brightness to balance event-producation with edge bleeding.

### Creating the event image

The events from the event-camera must first be sent to the visualisation method to produce an image from the asynchronous pixel firing pattern. The `vFramerLite` application is used to do so using the `BLACK` draw type, and with an event window of 34ms to capture both on and off frames of the flashing video:

 `--displays "(/right (BLACK))" --frameRate 30 --eventWindow 0.034`
 
:bulb: don't worry a YARP application is provided that sets all parameters correctly.

### Running the calibration procedure

The calibration is performed by sending images from the RGB camera and the output of the `vFramerLite` to the stereo-calibration module found in `icub-main`, in this case, using the `YARP` framework. A `yarpmanager` application is available to automatically set-up the required applications and connections:

* see [documentation](../README.md) to set-up the software and run a `yarpserver`.
* open a `yarpmanager` (e.g. from command line)
* in the GUI click open and search for the location you have saved the (application file)[stereoCalib.xml]
* In the entities find `stereoCalib`, double-click, then click `run all` and `connect all`.
* Check the `--from` arguments on the relevant modules and point them to the files for (calibration parameters)[stereoCalibe.ini] and (if using) (realsense parameters)[RealSense_config.ini]
* Open a terminal and run a remote procedure call: `yarp rpc /stereoCalib/cmd`
* Send the command: `>>start`
* Move the flashing checkerboard infront of both cameras, it should be apparent when a checkerboard is found, and the output of both cameras should be visualised to debug the images.

:bulb: The calibration between frame-based camera and event-driven camera can work also with different resolutions of the cameras.

### Setting calibration parameters

The `stereoCalib` works exploiting the information in the `stereoCalib.ini` file. 
Please, check the parameters carefully. 

* standalone: is required to run without the iCub kinematic chain
* boardWidth: number of corners to detection horizontally
* boardHeight: number of corners to detection vertically
* boardSize: the length in metres of the checkerboard square. :warning: as the physical size will change depending on the screen/tablet resolution and size, this will need to be measured depending on the device used.
* numberOfPairs: how many images to collect

### Running a mono calibration

Follow the above procedure with the following adjustments

* do not run the RGB camera, only the event-camera. Send only the `vFramerLite` output to the camera calibration software
* in the `stereoCalib.ini` file set the parameter: `MonoCalib 1` (i.e. do only a mono camera calibration)

### Running a stereo event-camera calibration

To run a stereo event-camera calibration both inputs to the `stereoCalib` module need to come from the `vFramerLite`

* set the `vFramerLite` parameters to:  `--displays "(/right (BLACK) /left (BLACK))" --frameRate 30 --eventWindow 0.034`
* modify the port connection settings to send events to the input of `vFramerLite` appropriately and send the image outputs to the `stereoCalib` module.




