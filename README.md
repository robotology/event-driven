| Read the [Documentation](http://robotology.github.io/event-driven/doxygen/doc/html/index.html) | Download the [Code](https://github.com/robotology/event-driven) |

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

 * [**vFramer**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/atis3-bridge) - visualisation of events streamed over a YARP port. Various methods for visualisation are available.
 * [**calibration**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/calibration) - estimating the camera intrinsic parameters
 * [**vPreProcess**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/vPreProcess) - splitting different event-types into separate event-streams, performing filtering, and simple augmentations (flipping etc.)
 * [**atis-bridge**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/atis3-bridge) - bridge between the Prophesee ATIS cameras and YARP
 * [**dv-bridge**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/dv-bridge) - bridge between the Inivation DVXplorer Micro camera and YARP
 * [**zynqGrabber**](https://github.com/robotology/event-driven/tree/ev2-dev/cpp_tools/zynqGrabber) - bridge between zynq-based FPGA sensor interface and YARP
 
## Applications

Applications that implement the algorithms available in `event-driven` are found in our companion repository

[event-driven-demos](https://github.com/event-driven-robotics/event-driven-demos)

## How to Install:

[Comprehensive instructions available for installation](http://robotology.github.io/event-driven/doxygen/doc/html/pages.html).

## References

Glover, A., and Bartolozzi C. (2016) *Event-driven ball detection and gaze fixation in clutter*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea. **Finalist for RoboCup Best Paper Award**

Vasco V., Glover A., and Bartolozzi C. (2016) *Fast event-based harris corner detection exploiting the advantages of event-driven cameras*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea.

V. Vasco, A. Glover, Y. Tirupachuri, F. Solari, M. Chessa, and Bartolozzi C. *Vergence control with a neuromorphic iCub. In IEEE-RAS International Conference on Humanoid Robots (Humanoids)*, November 2016, Mexico.

