# event-driven

_YARP integration for event-cameras and other neuromorphic sensors_

https://user-images.githubusercontent.com/9265237/222401464-73a9beaa-a1b6-4518-ae53-5bac5dfaeb9d.mp4

Libraries that handle neuromorphic sensors, such as the dynamic vision sensor, installed on the iCub can be found here, along with algorithms to process the event-based data. Examples include, optical flow, corner detection and ball detection. Demo applications for the iCub robot, and tutorials for running them, include saccading and attention, gaze following a ball, and vergence control.
```
@article{Glover2017b,
author = {Glover, Arren and Vasco, Valentina and Iacono, Massimiliano and Bartolozzi, Chiara},
doi = {10.3389/frobt.2017.00073},
journal = {Frontiers in Robotics and AI},
pages = {73},
title = {{The event-driven Software Library for YARP — With Algorithms and iCub Applications}},
volume = {4},
year = {2018}
}
```
## Libraries

Event-driven libraries provide basic functionality for handling events in a YARP environment. The library has definitions for:
 * core
   * codecs to encode/decode events to be compatable with address event representation (AER) formats.
   * Sending packets of events in `ev::packet` that is compatible with yarpdatadumper and yarpdataplayer.
   * asynchronous reading and writing ports that ensure data is never lost and giving access to latency information.
   * helper functions to handle event timestamp wrapping and to convert between timestamps and seconds.
 * vision
   * filters for removing salt and pepper noise.
   * sparse event warping using camera intrinsic parameters and extrinsic parameters for a stereo-pair
   * methods to draw events onto the screen in a variety of methods
 * algorithms
   * event surfaces such as the Surface of Active Events (SAE), Polarity Integrated Images (PIM), and Exponentially Reduced Ordinal Surface (EROS)
   * corner detection
   * optical flow

## TOOLS

 * [**vFramer**](https://github.com/robotology/event-driven/tree/main/cpp_tools/vFramer) - visualisation of events streamed over a YARP port. Various methods for visualisation are available.
 * [**calibration**](https://github.com/robotology/event-driven/tree/main/cpp_tools/calibration) - estimating the camera intrinsic parameters
 * [**vPreProcess**](https://github.com/robotology/event-driven/tree/main/cpp_tools/vPreProcess) - splitting different event-types into separate event-streams, performing filtering, and simple augmentations (flipping etc.)
 * [**atis-bridge**](https://github.com/robotology/event-driven/tree/main/cpp_tools/atis3-bridge) - bridge between the Prophesee ATIS cameras and YARP
 * [**zynqGrabber**](https://github.com/robotology/event-driven/tree/main/cpp_tools/zynqGrabber) - bridge between zynq-based FPGA sensor interface and YARP
 * [**log2vid**](https://github.com/robotology/event-driven/tree/main/cpp_tools/log2vid) - produce videos from datasets
 
## Applications

Applications that implement the algorithms available in `event-driven` are found in our organisation:

 * [event-driven-demos](https://github.com/event-driven-robotics/event-driven-demos)
 * [human-pose-estimation](https://github.com/event-driven-robotics/hpe-core)
 * [Fruit Ninja](https://github.com/event-driven-robotics/fruit-ninja-hpe)
 * [affine-tracking-and-control](https://github.com/event-driven-robotics/four-dof-affine-tracking)
 * [batch input, manipulation, visualisation, and export](https://github.com/event-driven-robotics/bimvee)
 * [6-DoF Tracking - EDOPT](https://github.com/event-driven-robotics/EDOPT)
 * [Visual Fault Button](https://github.com/event-driven-robotics/visual-fault-button)


## Installation and Documentation:

[Documentation Home](https://github.com/robotology/event-driven/blob/main/documentation/home.md)

## References

Glover, A., Gava, L., Li, Z. and Bartolozzi, C., 2024, May. *EDOPT: Event-camera 6-DoF Dynamic Object Pose Tracking*. In 2024 IEEE International Conference on Robotics and Automation (ICRA) (pp. 18200-18206). IEEE.

Goyal, G., Di Pietro, F., Carissimi, N., Glover, A. and Bartolozzi, C., 2023. *MoveEnet: Online high-frequency human pose estimation with an event camera*. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (pp. 4024-4033).

Glover, A., and Bartolozzi C. (2016) *Event-driven ball detection and gaze fixation in clutter*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea. **Finalist for RoboCup Best Paper Award**

Vasco V., Glover A., and Bartolozzi C. (2016) *Fast event-based harris corner detection exploiting the advantages of event-driven cameras*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea.

V. Vasco, A. Glover, Y. Tirupachuri, F. Solari, M. Chessa, and Bartolozzi C. *Vergence control with a neuromorphic iCub. In IEEE-RAS International Conference on Humanoid Robots (Humanoids)*, November 2016, Mexico.

