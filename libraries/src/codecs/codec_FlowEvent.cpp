#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

const std::string FlowEvent::tag = "FLOW";

FlowEvent::FlowEvent() : AddressEvent(), vx(0), vy(0) {}

FlowEvent::FlowEvent(const vEvent &v) : AddressEvent(v)
{
    const FlowEvent * v2 = dynamic_cast<const FlowEvent *>(&v);
    if(v2) {
        vx=v2->vx;
        vy=v2->vy;
    }
}

FlowEvent::FlowEvent(const FlowEvent &v) : AddressEvent(v)
{
    vx = v.vx;
    vy = v.vy;
}

event<> FlowEvent::clone()
{
    return std::make_shared<FlowEvent>(*this);
}

void FlowEvent::encode(yarp::os::Bottle &b) const
{
    AddressEvent::encode(b);
    b.addInt(*(int*)(&vx));
    b.addInt(*(int*)(&vy));
}

bool FlowEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (AddressEvent::decode(packet, pos) &&
            pos + 2 <= packet.size())
    {
        int word1=packet.get(pos).asInt();
        vx=*(float*)(&word1);
        int word2=packet.get(pos+1).asInt();
        vy=*(float*)(&word2);

        pos+=2;
        return true;
    }
    return false;
}

yarp::os::Property FlowEvent::getContent() const
{
    yarp::os::Property prop = AddressEvent::getContent();
    prop.put("vx",vx);
    prop.put("vy",vy);

    return prop;
}

std::string FlowEvent::getType() const
{
    return FlowEvent::tag;
}

int FlowEvent::getDeath() const
{
    return stamp + 1.0 / (sqrt(pow(vx, 2.0f) + pow(vy, 2.0f))
                          * vtsHelper::tstosecs());
}

}
