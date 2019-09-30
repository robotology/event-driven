# iCub Vergence Demo

Control the iCub to verge on a stimulus placed in the center of the field of view.

## Description

This application demonstrates how the iCub verges on a stimulus placed in the center of the field of view, based on the responses of a set of binocular Gabor filters.
The event stream is transmitted from the cameras (/zynqGrabber/vBottle:o) to vPreProcess (/vPreProcess/vBottle:i), that removes salt-and-pepper noise. The filtered stream (/vPreProcess/vBottle:o) is sent to vVergence (/vVergence/vBottle:i), that updates the responses of the filter bank and sends a command to the robot encoders. Events from left and right cameras are superimposed (/vVergence/debug:o) and sent to the yarp viewer (/viewDebug).

The *depthgt* module is useful for evaluating the performances of the algorithm, but not necessary for the demonstration. It automatically connects to the device (/OpenNI2/depthFrame:o) that exposes depth image from a Kinect sensor (/depthgt/depthim:i) and produces a depth value that can be used as ground truth (/depthgt/gt:o). The depth image (/depthgt/depthim:o) is then sent for visualisation to the yarp viewer (/viewGT). 

Here is a visualisation of the instantiated modules and connections.

![vVergence visualization](http://robotology.github.io/event-driven/doxygen/images/vVergence.png)

If you're going to use this controller for your work, please quote it within any resulting publication: V. Vasco, A. Glover, Y. Tirupachuri, F. Solari, M. Chessa, and Bartolozzi C. Vergence control with a neuromorphic iCub. In IEEE-RAS International Conference on Humanoid Robots (Humanoids), November 2016, Mexico.

## Dependencies

This application assumes that [yarprobotinterface](http://www.yarp.it/yarprobotinterface.html) is running.
This application requires [OpenNI2](http://wiki.icub.org/wiki/OpenNI2) installed to obtain the ground truth depth image, but it is not necessary for the demonstration.

## How to run the application

The application assumes you are connected to a *yarpserver* - see http://www.yarp.it/ for basic instructions for using yarp.

Inside the *Application* folder in the yarpmanager gui, you should see an entry called *vVergence*. Double click and open it.

Hit the *run* button and then *connect* on the yarpmanager gui.
You will now see the yarpview windows, displaying events from both camera and the ground truth depth image (if you are using a depth sensor).
An additional yarpscope window will open, to visualise the responses of the filter bank. This loads the xml file *scope_filtersConf.xml*, in the *vergenceController*,
that you can modify according to your needs.

The iCub is not controlled yet. To start the control, open a terminal and type

        yarp rpc /vVergence/rpctrigger:i

You can send commands to the *vergenceController*. 
Type `start` in the command prompt. You should get the following output:
    
        >>start
        Response: "Starting Verging..."

Now the vergence is controlled and the iCub will start verging on the object.
Move the object in depth to see the iCub vergence following the taget.
You should always see events from left and right cameras superimposed in the yarpview window *viewDebug*.

When you are done, type `reset` in the command prompt. This will stop the *vergenceController*.
You should get the following output:
    
        >>reset
        Response: "Resetting..."
