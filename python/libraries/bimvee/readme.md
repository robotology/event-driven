# bimvee - Batch Import, Manipulation, Visualisation, and Export Events etc.

# Quickstart: 

Look at 'examples.py' for examples of how to use the functionality in this library.

# Introduction

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

# Contents of library

## Import functions:

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

- "pol": numpy array of bool
- "x": numpy array of np.uint16
- "y": numpy array of np.uint16
- "ts": numpy array of np.float64 

timestamps are always converted to seconds; 
(raw formats are, however, e.g. int with unit increments of 80 ns for ATIS, 
int with unit increments of 1 us for DAVIS, etc) 

To the extent possible, dvs polarity is imported so that 1/True = ON/increase-in-light and
0/False = OFF/decrease-in-light. Be aware that individual datasets may contain the opposite convention. 

Multiple files imported simultaneously appear in a list of dicts;
lists and dicts are referred to jointly as containers, 
and the manipulation, visualistation and export functions which follow 
tend toward accepting containers with an arbitrarily deep hierarchy.

## Visualisation functions

A set of general functions for common visualisations of imported datasets.

- plotDvsContrast
- plotDvsLastTs
- plotSpikeogram
- plotEventRate
- plotFrame
- plotImu
- plotPose
- plotCorrelogram

info.py includes various functions to give quick text info about the contents of the containers that result from imports.

visualiser.py introduces classes which accept a datatype dict and serves frames,
perhaps to an external application, based on the data at various time points
and in various time windows.

## Manipulation functions

There are some functions for standard manipulations of data:

timestamps.py contains timestamp manipulations 
including jointly zeroing timestamps across multiple files, channels and datatypes. 
split.py includes various common ways by which datasets need to be split, e.g. splitByPolarity

## Export functions

exportIitYarp - exports to IIT's EDPR YARP format. Alongside data.log and 
info.log files, it exports an xml which specifies to yarpmanager how to 
visualise the resulting data. 

# Dependencies:

This library is intended to require minimal dependencies.
Notably, rosbag import is achieved without needing a ros installation.
Beyond the python standard library, the main dependencies are:

- numpy
- tqdm (for progress bars during import and export functions)

For the 'plot' family of visualisation functions:

- matplotlib
- mpl_toolkits (only for certain 3d visualisations)

The "visualiser", however, generates graphics as numpy arrays 
without reference to matplotlib, for rendering by an external application.

plotDvsLastTs uses rankdata from scipy; however if it's not installed, 
it defaults to a local definition; scipy is therefore an optional dependency.

# Type definitions

bimvee doesn't use classes for datatypes. Consequently, the code doesn't have a central place to refer to for the definition of datatypes. The types are intended to be used loosely, with minimal features which can be extended by adding optional fields. 

There are some datatypes which are simply dicts which act as containers to group information, for example the 'cam' type. However most of the functionality of the library is based around the idea of a datatype dict containing a set of keys where each is a numpy array (or other iterable) where there is a 'ts' key, containing a numpy array of np.float64 timestamps, and then each iterable key should have the same number of elements (in the zeroth dimension) as the ts field. Thus a set of timestamped 'events' or other data type is defined. Other keys may be included which either aren't iterables or don't have the same number of elements in the zeroth dimension. These are therefore not interpreted as contributing dimensions to the set of data points. Concretely the datatypes which have some kind of support are:

- dvs
- frame
- sample
- imu
- pose6q
- point3
- flow

- cam

Definitions of minimal and optional(*) fields follow.

- fieldName   dimensions  datatype(numpy array unless otherwise stated) notes

## dvs:

- ts  n np.float64
- x   n np.uint16
- y   n np.uint16 As the sensor outputs it; plot functions assume that y increases in downward direction, following https://arxiv.org/pdf/1610.08336.pdf
- pol n np.bool To the extent possible, True means increase in light, False means decrease. 
- dimX* 1 int
- dimY* 1 int

## frame:

- ts    n np.float64
- frame n list (of np.array of 2 or 3 dimensions np.uint8)

## sample:

- ts     n np.float64
- sensor n np.uint8
- value  n np.int16

## imu:

- ts  n    np.float64
- acc  nx3 np.float64 accelerometer readings [x,y,z] in m/s
- angV nx3 np.float64 angV readings [yaw, pitch roll?] in rad/s
- mag  nx3 np.float64 magnetometer readings [x, y, z] in tesla
- temp n   np.float64

## point3:

- ts    n   np.float64
- point nx3 np.float64 row format is [x, y, z]


## pose6q (effectively extends point3):

- ts       n   np.float64
- point    nx3 np.float64 row format is [x, y, z]
- rotation nx4 np.float64 row format is [rw, rx, ry, rz] where r(wxyz) define a quaternion

Note: quaternion order follows the convention of e.g. blender (wxyz) but not e.g. ros. (xyzw)

## flow:

...


## cam:

Following ros msg camera info, the fields this might contain include:

- height           1   int
- width            1   int
- distortion_model     string 
- D                5   np.float64 distortion params
- K                3x3 np.float64 Intrinsic camera matrix
- R                3x4 np.float64 Rectification matrix
- P                4x4 np.float64 projection matrix






