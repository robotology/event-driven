| Read the [Documentation](https://github.com/robotology/event-driven/blob/main/documentation/README.md) | Download the [Code](https://github.com/robotology/event-driven) |

# event-driven

_YARP integration for event-cameras and other neuromorphic sensors_

https://user-images.githubusercontent.com/9265237/222401464-73a9beaa-a1b6-4518-ae53-5bac5dfaeb9d.mp4

Libraries that handle neuromorphic sensors, such as the dynamic vision sensor, installed on the iCub can be found here, along with algorithms to process the event-based data. 

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
   * codecs to encode/decode events to be compatible with address event representation (AER) formats.
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
 * [**x320-bridge**](https://github.com/robotology/event-driven/tree/main/cpp_tools/x320-bridge) - bridge from Prophesee x320 chip using KTH's STM32 USB forwarding
 * [**zynqGrabber**](https://github.com/robotology/event-driven/tree/main/cpp_tools/zynqGrabber) - bridge between zynq-based FPGA sensor interface and YARP
 * [**event video creation**](https://github.com/robotology/event-driven/tree/main/cpp_tools/log2vid) - create nice 3D (x,y,t) videos from data files
 * [**visualisation and annotation**](https://github.com/event-driven-robotics/mustard-cpp)
 * [**data format conversion**](https://github.com/event-driven-robotics/bimvee)

## Algorithms

* [**EDOPT: 6-DoF Object Tracking**](https://github.com/event-driven-robotics/EDOPT)
* [**MoveEnet: Human Pose Estimation**](https://github.com/event-driven-robotics/hpe-core)
* [**Optical Flow**](https://github.com/event-driven-robotics/event-flow-str)
* [**Line Tracking**](https://github.com/event-driven-robotics/LEDGE)

 
## How to Install:

[Comprehensive instructions available for installation](https://github.com/robotology/event-driven/blob/main/documentation/full_installation.md).

## References

Glover, A., and Bartolozzi C. (2016) *Event-driven ball detection and gaze fixation in clutter*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea. **Finalist for RoboCup Best Paper Award**

Glover, A., Gava, L., Li, Z., & Bartolozzi, C. (2024, May). *Edopt: Event-camera 6-dof dynamic object pose tracking*. In 2024 IEEE International Conference on Robotics and Automation (ICRA) (pp. 18200-18206). IEEE.

Vasco V., Glover A., and Bartolozzi C. (2016) *Fast event-based harris corner detection exploiting the advantages of event-driven cameras*. In IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2016, Daejeon, Korea.

V. Vasco, A. Glover, Y. Tirupachuri, F. Solari, M. Chessa, and Bartolozzi C. *Vergence control with a neuromorphic iCub. In IEEE-RAS International Conference on Humanoid Robots (Humanoids)*, November 2016, Mexico.

Glover, A., & Bartolozzi, C. (2017, September). *Robust visual tracking with a freely-moving event camera*. In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 3769-3776). IEEE.

Iacono, M., Weber, S., Glover, A., & Bartolozzi, C. (2018, October). *Towards event-driven object detection with off-the-shelf deep learning*. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 1-9). IEEE.

Vasco, V., Glover, A., Mueggler, E., Scaramuzza, D., Natale, L., & Bartolozzi, C. (2017, July). *Independent motion detection with event-driven cameras*. In 2017 18th International Conference on Advanced Robotics (ICAR) (pp. 530-536). IEEE.

Goyal, G., Di Pietro, F., Carissimi, N., Glover, A., & Bartolozzi, C. (2023, June). *Moveenet: Online high-frequency human pose estimation with an event camera*. In 2023 IEEE/CVF Conference on Computer Vision and Pattern Recognition Workshops (CVPRW) (pp. 4024-4033). IEEE.

Glover, A., Dinale, A., Rosa, L. D. S., Bamford, S., & Bartolozzi, C. (2021). *luvharris: A practical corner detector for event-cameras*. IEEE Transactions on Pattern Analysis and Machine Intelligence, 44(12), 10087-10098.

Kreiser, R., Renner, A., Leite, V. R., Serhan, B., Bartolozzi, C., Glover, A., & Sandamirskaya, Y. (2020). *An on-chip spiking neural network for estimation of the head pose of the icub robot*. Frontiers in Neuroscience, 14, 551.

Glover, A., Vasco, V., & Bartolozzi, C. (2018, May). *A controlled-delay event camera framework for on-line robotics*. In 2018 IEEE International Conference on Robotics and Automation (ICRA) (pp. 2178-2183). IEEE.

Iacono, M., D’Angelo, G., Glover, A., Tikhanoff, V., Niebur, E., & Bartolozzi, C. (2019, November). *Proto-object based saliency for event-driven cameras*. In 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 805-812). IEEE.

Gava, L., Monforte, M., Bartolozzi, C., & Glover, A. (2022, June). *How late is too late? a preliminary event-based latency evaluation*. In 2022 8th International Conference on Event-Based Control, Communication, and Signal Processing (EBCCSP) (pp. 1-4). IEEE.\

Li, Z., Glover, A., Bartolozzi, C., & Natale, L. (2025, October). *6-DoF Object Tracking with Event-based Optical Flow and Frames*. In 2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 18880-18887). IEEE.
