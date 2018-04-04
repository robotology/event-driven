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

AddressEvent::AddressEvent() : vEvent(), x(0), y(0), channel(0), polarity(0) {}

AddressEvent::AddressEvent(const vEvent &v) : vEvent(v)
{
    const AddressEvent *v2 = dynamic_cast<const AddressEvent *>(&v);
    if(v2) {
        x = v2->x;
        y = v2->y;
        channel = v2->channel;
        polarity = v2->polarity;
    }
}

AddressEvent::AddressEvent(const AddressEvent &v) : vEvent(v)
{
    x = v.x;
    y = v.y;
    channel = v.channel;
    polarity = v.polarity;
}

event<> AddressEvent::clone()
{
    return std::make_shared<AddressEvent>(*this);
}

void AddressEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
#ifdef CODEC_128x128
    b.addInt(((channel&0x01)<<15)|((x&0x7f)<<8)|(((127-y)&0x7f)<<1)|(polarity&0x01));
#else
    b.addInt(((channel&0x01)<<20)|((y&0x0FF)<<10)|((x&0x1FF)<<1)|(polarity&0x01));
#endif
}

void AddressEvent::encode(std::vector<YARP_INT32> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
#ifdef CODEC_128x128
    b[pos++] = (((channel&0x01)<<15)|((x&0x7f)<<8)|(((127-y)&0x7f)<<1)|(polarity&0x01));
#else
    //b.addInt(((channel&0x01)<<15)|((y&0x7f)<<8)|((x&0x7f)<<1)|(polarity&0x01));
    b[pos++] = (((channel&0x01)<<20)|((y&0x0FF)<<10)|((x&0x1FF)<<1)|(polarity&0x01));
#endif
}

void AddressEvent::decode(int *&data)
{
    vEvent::decode(data);
    polarity = (*data >> 0) & 0x0001;
    x = (*data >> 1) & 0x001FF;
    y = (*data >> 10) & 0x00FF;
    channel = (*data >> 20) & 0x0001;
    data++;
}

bool AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        int word0=packet.get(pos).asInt();

#ifdef CODEC_128x128
        polarity=word0&0x01;

        word0>>=1;
        y=127-(word0&0x7f);

        word0>>=7;
        x=word0&0x7f;

        word0>>=7;
        channel=word0&0x01;
#else
        polarity=word0&0x01;

        word0>>=1;
        x=word0&0x1FF;

        word0>>=9;
        y=word0&0xFF;

        word0>>=10;
        channel=word0&0x01;
#endif

        pos += 1;
        return true;
    }
    return false;
}

yarp::os::Property AddressEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("channel", (int)channel);
    prop.put("polarity", (int)polarity);
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

