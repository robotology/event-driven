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

AddressEvent::AddressEvent() : vEvent(), x(0), y(0), channel(0), polarity(0), type(0) {}

AddressEvent::AddressEvent(const vEvent &v) : vEvent(v)
{
    const AddressEvent *v2 = dynamic_cast<const AddressEvent *>(&v);
    if(v2) {
        x = v2->x;
        y = v2->y;
        channel = v2->channel;
        polarity = v2->polarity;
        type = v2->type;
    }
}

AddressEvent::AddressEvent(const AddressEvent &v) : vEvent(v)
{
    x = v.x;
    y = v.y;
    channel = v.channel;
    polarity = v.polarity;
    type = v.type;
}

event<> AddressEvent::clone()
{
    return std::make_shared<AddressEvent>(*this);
}

void AddressEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
#if defined CODEC_128x128
    b.addInt(((channel&0x01)<<15)|((x&0x7f)<<8)|(((127-y)&0x7f)<<1)|(polarity&0x01));
#elif defined CODEC_304x240_20 //ATIS 20 bits encoding
    b.addInt(((channel&0x01)<<20)|((type&0x1)<<18)|((y&0x0FF)<<10)|((x&0x1FF)<<1)|(polarity&0x01));
#else //CODEC_304x240_24
    b.addInt(((channel&0x01)<<22)|((type&0x1)<<23)|((y&0x0FF)<<12)|((x&0x1FF)<<1)|(polarity&0x01));
#endif
}

void AddressEvent::encode(std::vector<std::int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
#if defined CODEC_128x128
    b[pos++] = (((channel&0x01)<<15)|((x&0x7f)<<8)|(((127-y)&0x7f)<<1)|(polarity&0x01));
#elif defined CODEC_304x240_20 //ATIS 20 bits encoding
    b[pos++] = (((channel&0x01)<<20)|((type&0x1)<<18)|((y&0x0FF)<<10)|((x&0x1FF)<<1)|(polarity&0x01));
#else
    b[pos++] = (((channel&0x01)<<22)|((type&0x1)<<23)|((y&0x0FF)<<12)|((x&0x1FF)<<1)|(polarity&0x01));
#endif
}

void AddressEvent::decode(int *&data)
{
    vEvent::decode(data);

#if defined CODEC_128x128
    polarity = (*data >> 0) & 0x0001;
    y = 127 - (*data >> 1) & 0x007F;
    x = (*data >> 8) & 0x007F;
    channel = (*data >> 15) & 0x0001;
#elif defined CODEC_304x240_20 //ATIS 20 bits encoding
    polarity = (*data >> 0) & 0x0001;
    x = (*data >> 1) & 0x01FF;
    y = (*data >> 10) & 0x00FF;
    type = (*data >> 18) & 0x0001;
    channel = (*data >> 20) & 0x0001;
#else
    polarity = (*data >> 0) & 0x0001;
    x = (*data >> 1) & 0x01FF;
    y = (*data >> 12) & 0x00FF;
    type = (*data >> 23) & 0x0001;
    channel = (*data >> 22) & 0x0001;
#endif
    data++;
}

bool AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        int data=packet.get(pos).asInt();

#if defined CODEC_128x128
        polarity = (data >> 0) & 0x0001;
        y = 127 - (data >> 1) & 0x007F;
        x = (data >> 8) & 0x007F;
        channel = (data >> 15) & 0x0001;
#elif defined CODEC_304x240_20 //ATIS 20 bits encoding
        polarity = (data >> 0) & 0x0001;
        x = (data >> 1) & 0x001FF;
        y = (data >> 10) & 0x00FF;
        type = (data >> 18) & 0x0001;
        channel = (data >> 20) & 0x0001;
#else
        polarity = (data >> 0) & 0x0001;
        x = (data >> 1) & 0x001FF;
        y = (data >> 12) & 0x00FF;
        type = (data >> 23) & 0x0001;
        channel = (data >> 22) & 0x0001;
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
    prop.put("type", (int)type);
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

