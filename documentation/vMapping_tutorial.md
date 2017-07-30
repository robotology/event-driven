#vMapping App

Introduction
------------
The vMapping app is used to calibrate and visualize the events onto the frame-based image.
This module is needed within the iCub setup which combines both traditional and event cameras
to map the events in the corresponding location of the image provided by the normal cameras.
The calibration is performed using the [asymmetric circle grid board](https://nerian.com/support/resources/patterns/)
similarly to the classical calibration procedure. Both events and images need to be undistorted since the mapping is planar. 
If no calibration is required the module will simply output the image overlapped with the events.
  
 Dependencies
 ------------
 This app does not need any further module to be running. The intrinsic parameters of both traditional and event camera
 have to be computed beforehand and are loaded from specific files within the *cameraCalibration* context as default setting.
 
 Instantiated Modules
 --------------------
 * **zynqGrabber**
 * **yarpdev**
 * **camCalib**
 * **vPreProcess**  
 * **vFramer** (for calibration only)
 * **vMapping**
 * **yarpview**
 
 Opened ports
 ------------
 * `/zynqGrabber/vBottle:o`
 * `/icub/cam/left`
 * `/icub/cam/right`
 * `/icub/camCalib/left/in`
 * `/icub/camCalib/right/in`
 * `/icub/camCalib/left/out`
 * `/icub/camCalib/right/out`
 * `/vPreProcess/vBottle:i`
 * `/vPreProcess/vBottle:o`
 * `/vFramer/AE:i`*
 * `/vFramer/blob/left`*
 * `/vFramer/blob/right`*
 * `/vMapping/vBottle:i`
 * `/vMapping/left/vImg:i`*
 * `/vMapping/right/vImg:i`*
 * `/vMapping/left/vImg:o`
 * `/vMapping/right/vImg:o`
 * `/mapped/left`
 * `/mapped/right`

    *for calibration only

How to run the application
--------------------------

On a console, run yarpserver (if not already running).

You can now run yarpmanager.

Inside the Application folder in the yarpmanager gui, you should see an entry called *mapping*. Double click and 
open it.

Make sure the cameras are connected properly.

Now you are ready to run the application! Hit the run button and then connect on the yarpmanager gui. You can avoid launching the vFramer if no calibration is necessary.

If you want to perform the calibration then after the application has started and the ports are connected you should see two windows showing the events (in form of blobs. See vFramer documentation) and the camera image. Move the calibration board in front of the cameras until enough images have been collected (number of images required is set with the maxIter parameter of the vMapping module). When calibration is over the results are saved in the vMapping.ini file within the context folder (*cameraCalibration* by default) .

Once the calibration is done (or in case of no calibration) the yarpview windows will show the image overlapped with the events.