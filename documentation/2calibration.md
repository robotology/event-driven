# Camera Calibration

Introduction
------------
This app launches the modules necessary to calibrate event-driven cameras. The event stream is sent in form of BLOB 
events (see [vFramer](http://robotology.github.io/event-driven/doxygen/doc/html/group__vFramer.html) for further 
details on the event types.) to the [stereoCalib](http://wiki.icub.org/iCub/main/dox/html/group__icub__stereoCalib.html) module.
In the app we already provide a set of default parameters to the stereoCalib module defining the calibration board 
type as the asymmetric circle grid (shown in image below) which have proven to be optimal for event cameras 
calibration.  

![calib_pattern](http://robocraft.ru/files/opencv/acircles_pattern.png)

In the following image an overview of the opened ports and how they are connected.
  
![builder_view](http://robotology.github.io/event-driven/doxygen/images/vCalib_builder.png)

Dependencies
------------
No special dependencies are required, all the required modules will be executed by the application. 

How to run the application
--------------------------

* On a console, run yarpserver (if not already running).

* On another console run yarpmanager

* Inside the Application folder in the yarpmanager gui, you should see an entry called vCalib. Double click and 
open it. Check that the parameters of the stereoCalib module are coherent with the board type you are going to use for 
calibration.

* Run the application and connect the ports with the buttons in the yarpmanager GUI.
You will now see the yarpview windows displaying the events.

* On a terminal type

        yarp rpc /stereoCalib/cmd
   
You can now send commands to the stereoCalib module. 
* Type `start` in the command prompt. You should get the 
following output:
    
        >>start
        Response: "Starting Calibration..."

Now the image acquisition has started.
* Move the calibration board in front of the cameras until the module recognises
 the board as many times as specified within the stereoCalib parameters (30 by default).
 
* Once the calibration is over the results are written in the `$ICUB_DIR/contexts/cameraCalibration/outputCalib.ini`.
 
* If you are satisfied with the results, you can copy them into the camera configuration file.
