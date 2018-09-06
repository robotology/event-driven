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

#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

const std::string LabelledAE::tag = "LAE";

LabelledAE::LabelledAE() : AddressEvent(), ID(0) {}

LabelledAE::LabelledAE(const vEvent &v) : AddressEvent(v)
{
    const LabelledAE * v2 = dynamic_cast<const LabelledAE *>(&v);
    if(v2) {
        ID = v2->ID;
    }
}

LabelledAE::LabelledAE(const LabelledAE &v) : AddressEvent(v)
{
    ID = v.ID;
}

event<> LabelledAE::clone()
{
    return std::make_shared<LabelledAE>(*this);
}

void LabelledAE::encode(yarp::os::Bottle &b) const
{
    AddressEvent::encode(b);
    b.addInt(ID);
}

void LabelledAE::encode(std::vector<std::int32_t> &b, unsigned int &pos) const
{
    AddressEvent::encode(b, pos);
    b[pos++] = ID;
}

void LabelledAE::decode(int *&data)
{
    AddressEvent::decode(data);
    ID = *data;
    data++;
}

bool LabelledAE::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    if (AddressEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        ID = packet.get(pos).asInt();
        pos+=1;
        return true;
    }
    return false;
}

yarp::os::Property LabelledAE::getContent() const
{
    yarp::os::Property prop = AddressEvent::getContent();
    prop.put("ID", ID);
    return prop;
}

std::string LabelledAE::getType() const
{
    return LabelledAE::tag;
}

}
