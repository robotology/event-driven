/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VCODEC__
#define __VCODEC__


#include <yarp/os/Bottle.h>
#include <memory>
#include <deque>
#include <math.h>
#include <vector>
#include <iostream>

namespace ev {

//macros
class vEvent;

/// \brief an "event" is wrapper for the shared_ptr class
template<typename V = vEvent> using event = std::shared_ptr<V>;
/// \brief typecast the "event" safely checking the event type
template<typename V1, typename V2> inline event<V1> as_event(event<V2> orig_event) {
    return std::dynamic_pointer_cast<V1>(orig_event);
}
/// \brief typecast the "event" forcing the event type
template<typename V1, typename V2> inline event<V1> is_event(event<V2> orig_event) {
    return std::static_pointer_cast<V1>(orig_event);
}
/// \brief allocate memory for, and instantiate, a new event
template<typename V> event<V> inline make_event(void) {
    return std::make_shared<V>();
}
/// \brief a fast event-type conversion to access event data. Does no checking
/// that the event actually exists.
template<typename V> inline V* read_as(const event<> &orig_event) {
    return (V *)orig_event.get();
}
/// \brief make a new event, copying from an existent event. Can be used to
/// upgrade the event-type.
template<typename V1, typename V2> event<V1> make_event(event<V2> orig_event) {
    return std::make_shared<V1>(*(orig_event.get()));
}
/// \brief vQueue is a wrapper for a deque of "event"
using vQueue = std::deque< event<vEvent> >;

/// \brief sort a vQueue ensuring temporal order
void qsort(vQueue &q, bool respectWraps = false);

/// \brief create an "event" based on the string tag it uses.
event<> createEvent(const std::string &type);

/// \brief get the coded packet size of an event
unsigned int packetSize(const std::string &type);

/// \brief camera values for stereo set-up
enum { VLEFT = 0, VRIGHT = 1 } ;

//event declarations
/// \brief base event class which defines the time the event occurs
class vEvent
{
public:
    static const std::string tag;
    unsigned int stamp:31;

    vEvent();
    virtual ~vEvent();

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<std::int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(int *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel();
};

/// \brief an event with a pixel location, camera number and polarity
class AddressEvent : public vEvent
{
public:
    static const std::string tag;
    unsigned int x:10;
    unsigned int y:10;
    unsigned int channel:1;
    unsigned int polarity:1;
    unsigned int type:1;

    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<std::int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(int *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
using AE = AddressEvent;

/// \brief an AddressEvent with a velocity in visual space
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
    virtual void encode(std::vector<std::int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(int *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

/// \brief an AddressEvent with an ID or class label
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
    virtual void encode(std::vector<std::int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(int *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

/// \brief a LabelledAE with parameters that define a 2D gaussian
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
    virtual void encode(std::vector<std::int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(int *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

}

#endif


