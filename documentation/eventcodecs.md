# Event Coding

Events are serialised and coded in a standardised format for sending and receiving between modules. In addition, a packet containing multiple different types of events is segmented by event-type such that a search can quickly retrieve events of only a specific type. The packet is formed as such:
```
EVENTTYPE-1-TAG ( serialised and concatenated events of type 1) EVENTTYPE-2-TAG ( serialised and concatenated events of type 2) ...
```
Each event class defines the TAG used to identify itself and also the method with which the event data is serialised. Managing the serialisation and de-serialisation of the event data is then simply a case of using the event class to write/read its TAG and then call its encode/decode functions on the serialised data. The `eventdriven::vBottle` class handles the coding of packets in the event-driven project.

Events are defined in a class hierarchy, with each child class calling its parent encode/decode function before its own. Adding a new event therefore only requires defining the serialisation method for any new data that the event-class contains (_e.g._ the **Flow** event only defines how the velocities are encoded and calls its parent class, the **AdressEvent**, to encode other information, such as position and timestamp).

![The Event-Type Class Hierarchy](@ref classev_1_1vEvent.png)

# Event Coding Definitions

The **vEvent** uses 4 bytes to encode a timestamp (_T_)
```
[10000000 TTTTTTT TTTTTTTT TTTTTTTT]
```
An **AddressEvent** uses 4 bytes to encode position (_X_, _Y_), polarity (_P_) and channel (_C_). Importantly, as AddressEvent  is of type `vEvent` the timestamp information of this event is always encoded as well.
```
[00000000 00000000 CYYYYYYY XXXXXXXP]
```
or, if the **VLIB_10BITCODEC** CMake flag is set **ON** (used for the ATIS camera) the AddressEvent is encoded as:
```
[00000000 000C00YY YYYYYYXX XXXXXXXP]
```

A **FlowEvent** uses 8 bytes to encode velocity (ẋ, ẏ), each 4 bytes represent a _float_. Similarly as FlowEvent is of time AddressEvent the FlowEvent also encodes all the position and timestamp information above.
```
[ẋẋẋẋẋẋẋẋ ẋẋẋẋẋẋẋẋ ẏẏẏẏẏẏẏẏ ẏẏẏẏẏẏẏẏ]
```
 A **LabelledAE** is labelled as belonging to a group ID (_I_) using a 4 byte _int_.
```
[IIIIIIIII IIIIIIIII IIIIIIIII IIIIIIIII]
```
A **GaussianAE** extends a cluster event with a 2 dimensional Gaussian distribution parameterised by (_sx_, _sy_, _sxy_) using a total of 12 bytes.
```
[sxsxsxsxsxsxsxsx sxsxsxsxsxsxsxsx sxsxsxsxsxsxsxsx sxsxsxsxsxsxsxsx sysysysysysysysy sysysysysysysysy sysysysysysysysy sysysysysysysysy sxysxysxysxysxysxysxysxy sxysxysxysxysxysxysxysxy sxysxysxysxysxysxysxysxy sxysxysxysxysxysxysxysxy]
```
A **CochleaEvent** uses 4 bytes to encode frequency channel (_F_), polarity (_P_), neuron id of the Jeffress model (_N_), the olive model (_O_), the auditory model (_M_), and channel (_C_). Importantly, as CochleaEvent  is of type `vEvent` the timestamp information of this event is always encoded as well.
```
[00000000 0C000MO0 0NNNNNN0 0FFFFFFP]
```

# Coding in YARP

The `eventdriven::vBottle` class wraps the encoding and decoding operations into a `yarp::os::Bottle` such that an example `vBottle` will appear as:
```
AE (-2140812352 15133 -2140811609 13118) FLOW (-2140812301 13865 -1056003417 -1055801578)
```
**NOTE:** The actual data sent by YARP for a bottle includes signifiers for data type and data length, adding extra data to the bottle as above.

```
256 4 4 2 'A' 'E' 257 4 -2140812352 15133 -2140811609 13118 4 4 'F' 'L' 'O' 'W' 257 4 -2140812301 13865 -1056003417 -1055801578
```
