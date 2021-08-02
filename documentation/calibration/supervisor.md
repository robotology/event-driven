# camera-calibration-supervisor

This README is a guide to use the [camera-calibration-supervisor
](https://github.com/robotology/camera-calibration-supervisor) for the ATIS cameras mounted on iCub. If you need more information just have a look to the original repo! 


The stereo calibration or the mono calibration allows obtaining intrinsic and extrinsic parameters removing distortions from the lenses. 
The camera-calibration-supervisor is a user-friendly framework that identifies the best chessboard poses guiding you into the next chessboard position. This set of positions are the best set to obtain a good calibration.  

 <p align="center">
    <img src="https://github.com/event-driven-robotics/camera-calibration-supervisor/blob/main/calibration.png" width="500">
 </p>


## How to set up your calibration on the robot 🤖

### Requirements
* Laptop with the installed camera-calibration-supervisor app: icub23
* Checkerboard-fast: SAMSUNG tablet
* iCub's eyes vergence ~ 5

Let's start with the calibration on the robot! 

* First of all you need to turn on the iCub's motors and CPU by clicking the buttons on the side of the iCub's backpack.
Then, run `yarpmanager` and run `icub-head`,`icub-zynq` & `icub-23/24`from the Cluster. 

* From Entities search for`iCubStartup-eventdriven` application and run:

  * `yarplogger` to check any error occurring using the robot. 
  * `yarprobotinterface` to run the robot. 
  * Do not forget to shift iCub's eyes vergence to ~ 5 (`yarpmotorgui` from the terminal, /head, JOINT5)

* From Entities search for `vViewCamCalibSupervisor` application, run and connect everything.


* From Entities search for `Event_Cam_Calib_Supervisor_App` application and run all the `yarpviews`. 
  * Then, from a terminal run `stereoCalib --from icubEyes_ATIS-board-gazebo.ini`. 
  * Finally, run `calibSupervisor` from the  `Event_Cam_Calib_Supervisor_App` application and connect everything. 
  
Please, make sure to check the .ini file. It should look like the one below:

```ruby
[CAMERA_CALIBRATION_RIGHT]

projection         pinhole
drawCenterCross    0

w 304
h 240
fx 195.625
fy 195.719
cx 141.748
cy 131.03
k1 -0.341102
k2 0.0972876
p1 -0.00761546
p2 0.00759488

[CAMERA_CALIBRATION_LEFT]

projection         pinhole
drawCenterCross    0

w 304
h 240
fx 179.823
fy 178.975
cx 157.431
cy 114.611
k1 -0.304171
k2 0.193928
p1 -0.000112923
p2 -0.0111818

[STEREO_DISPARITY]
HN (0.999425 0.00522887 -0.0335127 0.096062 -0.00676219 0.998928 -0.0458046 -0.0046386 0.0332373 0.0460048 0.998388 0.0465875 0 0 0 1)
QL (-0.000767	-0.000192	-0.026078	 0.003068	-0.003068	-0.001534	-0.000479	 0.043191)
QR (-0.000767	-0.000192	-0.026078	 0.003068	-0.003068	-0.001534	-0.000479	-0.043191)

[STEREO_CALIBRATION_CONFIGURATION]
boardWidth 8
boardHeight 6
boardSize 0.021675
numberOfPairs 30
```

* Open a yarp rpc port: `yarp rpc /stereoCalib/cmd`  and type `start`. You should get something like this: 
```
>>start
Response: "Starting Calibration..."
```
This step should be handled by `stereoCalib`, but if you want to monitor the ongoing process I advise you to run `stereoCalib` from a terminal looking at what the application returns. 
* Now, your calibration is ready to start. Therefore, take your chessboard and put the `yarpviews` (/display, /viewleft and /viewright) visible whilst you are moving your board. The calibration process needs to collect several good positions for the board that is equal to the `numberOfPairs` of the .ini file. Please, be sure `stereoCalib` is collecting the images writing:  `[INFO] Saving images number #`
 
* Once you have collected all the positions is the time to obtain the intrinsic and extrinsic parameters running the script: 
```ruby
modify-params.sh icubEyes_ATIS-board-gazebo.ini outputCalib.ini cameraCalibration
```
This script will overwrite the results on the initial .ini file. You should obtain something like this: 
```ruby
Using file /home/icub/.local/share/yarp/robots/iCubGenova02/icubEyes_ATIS-board-gazebo.ini
stereoCalib writes the following file: /home/icub/.local/share/yarp/contexts/cameraCalibration/outputCalib.ini
 
Running script...with params /home/icub/.local/share/yarp/robots/iCubGenova02/icubEyes_ATIS-board-gazebo.ini /home/icub/.local/share/yarp/contexts/cameraCalibration/outputCalib.ini cameraCalibration
 
 
Script completed successfully...
```

* Let's now check our calibration! Run `Event_Calib_Cameras` and conenct everything. You now should be able to see the chessboard with no distortion!

<p align="center">
    <img src="https://github.com/event-driven-robotics/camera-calibration-supervisor/blob/main/checkingcalibration.png" width="500">
 </p>



