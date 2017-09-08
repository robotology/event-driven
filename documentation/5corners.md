# How to Detect and Visualise a Stream of Corner Events

Detect and visualise a stream of corner events from the cameras / from a pre-recorded sequence.

## Description

This application demonstrates how to detect and visualise a stream of corner events either from the cameras or from a pre-recorded sequence. 
The event stream is transmitted from the cameras (/zynqGrabber/vBottle:o) to the vPreProcess (/vPreProcess/vBottle:i), that removes salt-and-pepper noise from the event stream. The filtered stream (/vPreProcess/vBottle:o) is sent to vCorner (/vCorner/vBottle:i) that detects corners whose Harris score is higher than a threshold. Both address events and corner events are sent to the vFramer, that has specific ports for different kinds of events (/vFramer/AE:i for address events, /vFramer/LAE:i for corner events). Address events and corresponding corner events are visualised on the yarp viewers (/viewLeft and /viewRight).

Here is a visualisation of the instantiated modules and connections.

![vCorner visualization](http://robotology.github.io/event-driven/doxygen/images/vCorner.png)

For more details on the algorithm, please refer to: Vasco V., Glover A., and Bartolozzi C. (2016) Fast event-based harris corner detection exploiting the advantages of event-driven cameras. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea.

## Dependencies

No special dependencies are required, all the required modules will be executed by the application. 

## How to run the application

The application assumes you are connected to a *yarpserver* - see http://www.yarp.it/ for basic instructions for using yarp.

Inside the *Application* folder in the yarpmanager gui, you should see an entry called *vCorner*. Double click and open it.

Now you are ready to run the application. Hit the *run* button and then *connect* on the yarpmanager gui.

You will now see the yarpview windows displaying the address events and the corner events (in red).

To visualise events from a pre-recorded dataset, you can run *yarpdataplayer*.

Since *yarpdataplayer* opens the port with the same name as the real robot, make sure the same port is not running (or that you start an instance of the nameserver with your own namespace).
