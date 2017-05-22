/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/// \defgroup Library Library
/// \defgroup vCodec vCodec
/// \ingroup Library
/// \brief class hierarchy for different types of eventdriven::vEvent
///
/// Event Coding
/// ============
///
/// Events are serialised and coded in a standardised format for sending and
/// receiving between modules. In addition a packet containing multiple
/// different types of events is segmented by event-type such that a search can
/// quickly retrieve events of only a specific type. The packet is formed as
/// such:
///
///     EVENTTYPE-1-TAG ( serialised and concatinated events of type 1) EVENTTYPE-2-TAG ( serialised and concatinated events of type 2) ...
///
/// Each event class defines the TAG used to identify itself and also the
/// method with which the event data is serialised. Managing the serialisation
/// and de-serialisation of the event data is then simply a case of using the
/// event class to write/read its TAG and then call its encode/decode functions
/// on the serialised data. The eventdriven::vBottle class handles the coding of
/// packets in the event-driven project.
///
/// Events are defined in a class hierarchy, with each child class calling its
/// parent encode/decode function before its own. Adding a new event therefore
/// only requires defining the serialisation method for any new data that the
/// event-class contains (e.g. the Flow event only defines how the velocities
/// are encoded and calls its parent class, the AdressEvent, to encode other
/// information, such as position and timestamp).
///
/// ![The Event-Type Class Hierarchy](@ref classeventdriven_1_1vEvent.png)
///
/// Event Coding Definitions
/// ------------------------
///
/// The **vEvent** uses 4 bytes to encode a timestamp (_T_)
///
///     [10000000 TTTTTTT TTTTTTTT TTTTTTTT]
///
/// An **AddressEvent** uses 4 bytes to encode position (_X_, _Y_), polarity
/// (_P_) and channel (_C_). Importantly as AddressEvent is of type vEvent the
/// timestamp information of this event is always encoded as well.
///
///     [00000000 00000000 CYYYYYYY XXXXXXXP]
///
/// A **FlowEvent** uses 8 bytes to encode velocity (ẋ, ẏ), each 4 bytes
/// represent a _float_. Similarly as FlowEvent is of time AddressEvent the
/// FlowEvent also encodes all the position and timestamp information above.
///
///     [ẋẋẋẋẋẋẋẋ ẋẋẋẋẋẋẋẋ ẏẏẏẏẏẏẏẏ ẏẏẏẏẏẏẏẏ]
///
/// An **AddressEventClustered** is labelled as belonging to a group ID (_I_)
/// using a 4 byte _int_.
///
///     [IIIIIIIII IIIIIIIII IIIIIIIII IIIIIIIII]
///
/// A **ClusterEvent** is encodes the central position (_X_, _Y_) of a labelled
/// cluster (_I_) and also has a polarity (_P_) and channel (_C_)
///
///     [IIIIIIII IIIIIIII CYYYYYYY XXXXXXXP]
///
/// _NOTE: Cluster Event and AddressEventClustered could be consolidated in
/// some way_
///
/// A **ClusterEventGauss** extends a cluster event with a 2 dimensional
/// Gaussian distribution parameterised by (_sx_, _sy_, _sxy_) a count of events
/// falling in this distribution (_n_) and its velocity (ẋ, ẏ) using a
/// total of 12 bytes.
///
///     [sxysxysxysxysxysxysxysxy sxysxysxysxysxysxysxysxy nnnnnnnn nnnnnnnn
///      sxsxsxsxsxsxsxsx sxsxsxsxsxsxsxsx sysysysysysysysy sysysysysysysysy
///      ẋẋẋẋẋẋẋẋ ẋẋẋẋẋẋẋẋ ẏẏẏẏẏẏẏẏ ẏẏẏẏẏẏẏẏ]
///
/// A **CollisionEvent** uses 1 byte to encode collision position (_X_, _Y_),
/// the channel (_C_) and the two cluster IDs that collided (_I1_, _I2_)
///
///     [I1I1I1I1I1I1I1I1 I2I2I2I2I2I2I2I2 CXXXXXXX 0YYYYYYY]
///
/// Coding in YARP
/// --------------
///
/// The eventdriven::vBottle class wraps the encoding and decoding operations into a
/// yarp::os::Bottle such that an example vBottle will appear as:
///
///     AE (-2140812352 15133 -2140811609 13118) FLOW (-2140812301 13865 -1056003417 -1055801578)
///
/// _NOTE: The actual data sent by YARP for a bottle includes signifiers for
/// data type and data length, adding extra data to the bottle as above._
///
///     256 4 4 2 'A' 'E' 257 4 -2140812352 15133 -2140811609 13118 4 4 'F' 'L' 'O' 'W' 257 4 -2140812301 13865 -1056003417 -1055801578
///
/// Coding in ROS
/// -------------
///
/// Here explain how the coding would/will be performed for ROS. How does the
/// YARP/ROS interface consolidate the above two formats.
///

#ifndef __VCODEC__
#define __VCODEC__

#include <yarp/os/Bottle.h>
#include <memory>
#include <deque>
#include <math.h>

namespace ev {

//macros
class vEvent;
template<typename V = vEvent> using event = std::shared_ptr<V>;
template<typename V1, typename V2> event<V1> as_event(event<V2> orig_event) {
    return std::dynamic_pointer_cast<V1>(orig_event);
}
template<typename V1, typename V2> event<V1> is_event(event<V2> orig_event) {
    return std::static_pointer_cast<V1>(orig_event);
}
template<typename V> event<V> make_event(void) {
    return std::make_shared<V>();
}
template<typename V1, typename V2> event<V1> make_event(event<V2> orig_event) {
    return std::make_shared<V1>(*(orig_event.get()));
}
using vQueue = std::deque< event<vEvent> >;

void qsort(vQueue &q, bool respectWraps = false);
event<> createEvent(const std::string &type);

//event declarations
class vEvent
{
public:
    static const std::string tag;
    unsigned int stamp:31;

    vEvent();
    virtual ~vEvent();

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel();
};

class AddressEvent : public vEvent
{
public:
    static const std::string tag;
    unsigned int x:10;
    unsigned int y:10;
    unsigned int channel:1;
    unsigned int polarity:1;

    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
using AE = AddressEvent;

class FlowEvent : public AddressEvent
{
public:
    static const std::string tag;
    float vx;
    float vy;

    FlowEvent();
    FlowEvent(const vEvent &v);
    FlowEvent(const FlowEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

class LabelledAE : public AddressEvent
{
public:
    static const std::string tag;
    int ID;

    LabelledAE();
    LabelledAE(const vEvent &v);
    LabelledAE(const LabelledAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

class GaussianAE : public LabelledAE
{
public:
    static const std::string tag;
    float sigx;
    float sigy;
    float sigxy;

    GaussianAE();
    GaussianAE(const vEvent &v);
    GaussianAE(const GaussianAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

}

#endif


