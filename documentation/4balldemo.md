# Ball Tracking with Head and Arm following

This tutorial explains how to perform ball tracking with the event-driven cameras. The Robot can be commanded to look at the position of the ball, and also move its arm to the position of the ball.

![icubandball](http://robotology.github.io/event-driven/doxygen/images/icubandball.png)

## How it works

Ball tracking is performed using a particle filter. There are limits on the maximum and minimum size of the ball, and thresholds for minimum "roundness" to be successfully detected. The output of the particle filter is a ev::GaussianAE event which describes the position and size of the ball and whether the ball is over the "true detection threshold". The visualisation will display the ball position as blue if the detection score is above the threshold, and red otherwise.

The application is run with ball tracking performed on both the left and right event-streams. The detected position in the left and right cameras are sent to the vGazeDemo module, which checks the consistency between left and right streams (are they tracking the same ball?) and if both are above the "true detection threshold". If all checks pass, the vGazeDemo module uses the iCub Gaze Controller to look at the position of the ball, using the 3D position calculated from stereo triangulation. The left arm can also be commanded to move to the same position by setting the appropriate flag on start-up.

The ball should then be moved in front of the robot to have the gaze of the robot follow its position. If the position of the ball is lost for some seconds, the particles are resampled to the centre of the image, and the ball will have to be moved to the centre of the image to regain tracking.

## Dependencies

To move the robot a iKinGazeCtrl module needs to be running and connected to the yarprobotinterface (a robot or simulator is required). The iKinGazeCtrl also needs a correctly calibrated camera parameter file.

## How to run the application

The application assumes you are connected to a *yarpserver* - see http://www.yarp.it/ for basic instructions for using yarp.

1. run a yarpmanager
2. in the yarpmanager, find and open the vGazeDemo application. If the applciation is not available the event-driven library has not been correctly installed, or the path to the icub-contrib-common install folder is not correctly set.
3. the zynqGrabber in the modules panel can only be used if you have the robot environment (with yarprun) correctly installed and set-up. If not, you can manually run your event grabbing module on the computer connected to the camera, or use a yarpdataplayer to play a dataset instead.
4. run all modules (ignoring zynqGrabber if you have a custom grabber, and ignoring vGazeDemo if no robot is present).
5. the visualation will show the current estimate of the ball position on the left and the right cameras.

![connections](http://robotology.github.io/event-driven/doxygen/images/vGazeDemoConnections.png)

note: shmem connections can only be used if the modules are running on the same physical machine.
