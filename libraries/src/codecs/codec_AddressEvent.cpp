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

#include <yarp/os/Bottle.h>
#include "iCub/eventdriven/vCodec.h"

namespace ev {

const std::string AddressEvent::tag = "AE";

AddressEvent::AddressEvent() : vEvent(), polarity(0), x(0), y(0), channel(0), type(0), skin(0) {}

AddressEvent::AddressEvent(const vEvent &v) : vEvent(v)
{
    const AddressEvent *v2 = dynamic_cast<const AddressEvent *>(&v);
    if(v2) {
        _coded_data = v2->_coded_data;
    }
}

AddressEvent::AddressEvent(const AddressEvent &v) : vEvent(v)
{
    _coded_data = v._coded_data;
}

event<> AddressEvent::clone()
{
    return std::make_shared<AddressEvent>(*this);
}

void AddressEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt32(_coded_data);
}

void AddressEvent::encode(std::vector<std::int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
    b[pos++] = _coded_data;
}

void AddressEvent::decode(int *&data)
{
    vEvent::decode(data);
    _coded_data = *(data++);
}

bool AddressEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        _coded_data = packet.get(pos++).asInt();
        return true;
    }
    return false;
}

yarp::os::Property AddressEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("channel", (int)channel);
    prop.put("polarity", (int)polarity);
    prop.put("type", (int)type);
    prop.put("skin", (int)skin);
    prop.put("x", (int)x);
    prop.put("y", (int)y);

    return prop;
}

std::string AddressEvent::getType() const
{
    return AddressEvent::tag;
}

int AddressEvent::getChannel() const
{
    return (int)channel;
}

void AddressEvent::setChannel(const int channel)
{
    this->channel=channel;
}

}

