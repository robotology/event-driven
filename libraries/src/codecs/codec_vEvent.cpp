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

#ifndef __VCODEC_VEVENT__
#define __VCODEC_VEVENT__


#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

const std::string vEvent::tag = "TS";

vEvent::vEvent() : stamp(0) {}

vEvent::~vEvent() {}

event<> vEvent::clone()
{
    return std::make_shared<vEvent>(*this);
}

void vEvent::encode(yarp::os::Bottle &b) const
{
    b.addInt(stamp&vtsHelper::max_stamp);
}

void vEvent::encode(std::vector<int> &b, unsigned int &pos) const
{
    b[pos++] = stamp&vtsHelper::max_stamp;
}

bool vEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    if(pos + 1 <= packet.size()) {
        stamp = packet.get(pos).asInt()&vtsHelper::max_stamp;
        pos += 1;
        return true;
    }
    return false;

}

void vEvent::decode(int *&data)
{
    stamp = (*data) & vtsHelper::max_stamp;
    data++;
}

yarp::os::Property vEvent::getContent() const
{
    yarp::os::Property prop;
    prop.put("type", getType().c_str());
    prop.put("stamp", (int)stamp);

    return prop;
}


std::string vEvent::getType() const
{
    return vEvent::tag;
}

int vEvent::getChannel() const
{
    return -1;
}

void vEvent::setChannel()
{

}

}

#endif
