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

bool LabelledAE::decode(const yarp::os::Bottle &packet, int &pos)
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
