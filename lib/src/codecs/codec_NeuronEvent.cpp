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
#include "event-driven/vCodec.h"

namespace ev {

const std::string neuronEvent::tag = "NEU";

neuronEvent::neuronEvent() : vEvent(), id(0) {}

neuronEvent::neuronEvent(const vEvent &v) : vEvent(v), id(0)
{
    const neuronEvent *v2 = dynamic_cast<const neuronEvent *>(&v);
    if(v2) {
        id = v2->id;
    }
}

neuronEvent::neuronEvent(const neuronEvent &v) : vEvent(v)
{
    id = v.id;
}

event<> neuronEvent::clone()
{
    return std::make_shared<neuronEvent>(*this);
}

void neuronEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt32(id);
}

void neuronEvent::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
    b[pos++] = id;
}

void neuronEvent::decode(const int32_t *&data)
{
    vEvent::decode(data);
    id = *(data++);
}

bool neuronEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        id = packet.get(pos++).asInt32();
        return true;
    }
    return false;
}

yarp::os::Property neuronEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("id", (int)id);
    return prop;
}

std::string neuronEvent::getType() const
{
    return neuronEvent::tag;
}


}

