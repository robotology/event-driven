bimvee - Batch Import, Manipulation, Visualisation, and Export Events etc.

Quickstart: Look at 'examples.py' for examples of how to use the functionality in this library.

Working with timestamped address-event data from event cameras (dvs), and 
possibly other neuromorphic sensors (e.g. icub skin, maybe INI silicon cochlea etc), 
alongside other timestamped data that we need for our experiments, 
including but not limited to:
- frame-based camera images (frame)
- IMU (imu)
- tracked poses (e.g. 6 DOF - pose6q)
- etc
- derived datatypes, such as optical (flow) events, or labelled dvs events 
    (dvsL) etc might also be supported. 
- Camera calibration info is also imported from e.g. ros (cam) 

Aim is to include, in reverse order of importance:
IIT YARP .log - ATIS Gen1 - 24 bit (incl. IMU, SKIN?)
rpg_dvs_ros - DVS/DAVIS .bag
Penn MvSEC (by using the above rosbag import)
Intel realsense 265 (by using the above rosbag import)
Vicon - as dumped by yarpDumper; maybe also c3d?
INI jAER / cAER .aedat (v1/2/3) DVS / DAVIS / Cochlea 
(there is legacy code for this from "aedattools" repo)
Samsung (SEC) Gen3 VGA .bin
Celex v5 .bin
Maybe Prophesee raw?

Import functions:

The aim is to bring the different formats into as common a format as possible.
Parameters: at least the param "filePathOrName" (otherwise working from current directory)
Returns a dict containing:

{'info': {<filePathOrName, any other info derivable from file headers>},

 'data': {
 
         channel0: {}
         channel1: {}
         ...
         }}

Channel is loosely a sensor, so for example a file might contain 'left' and 'right' 
camera channels, and each of these channels might contain dvs events alongside 
other data types like frames. 
Each channel is a dict containing one dict for each type of data.
Data types may include:
 - dvs (Timestamped (ts) 2D address-events (x, y) with polarity (pol), from an event camera)
 - frame
 - imu
 - flow 
 - pose
 - etc

dvs data type, for example, then contains:

- "pol": numpy array of uint8 in [0, 1]
- "x": numpy array of np.uint16
- "y": numpy array of np.uint16
- "ts": numpy array of np.float64 

timestamps are always converted to seconds; 
(raw formats are, however, e.g. int with unit increments of 80 ns for ATIS, 
int with unit increments of 1 us for DAVIS, etc) 

Currently (2019_11_13) there's no alignment on how to interpret polarity (i.e. 1 = ON or OFF?)

Visualisation functions:
A set of general functions for common visualisations of imported datasets.

- plotDvsContrast
- plotDvsLastTs
- plot Spikeogram
- plotEventRate
- plotFrame
- plotImu
- plotPose

info.py includes various functions to give quick text info about the contents 
of the containers that result from imports. 

There are some functions for standard manipulations of data:

timestamps.py contains timestamp manipulations 
including jointly zeroing timestamps across multiple files, channels and datatypes. 
split.py includes various common ways by which datasets need to be split, e.g. splitByPolarity

Export functions:

exportIitYarp - exports to IIT's EDPR YARP format. Alongside data.log and 
info.log files, it exports an xml which specifies to yarpmanager how to 
visualise the resulting data. 

Dependencies:

This library is intended to require minimal dependencies.
Notably, rosbag import is achieved without needing a ros installation.
Beyond the python standard library, the main dependencies are:

- numpy

For the 'plot' family of visualisation functions:

- matplotlib
- mpl_toolkits (only for certain 3d visualisations)

The "visualiser", however, generates graphics as numpy arrays 
without reference to matplotlib, for rendering by an external application.

plotDvsLastTs uses rankdata from scipy; however if it's not installed, 
it defaults to a local definition; scipy is therefore an optional dependency.

import and export functions give progress bars using:

- tqdm

If this is not installed, a local tqdm module allows the library to function regardless. 
