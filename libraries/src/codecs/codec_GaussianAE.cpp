#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

const std::string GaussianAE::tag = "GAE";

GaussianAE::GaussianAE() : LabelledAE(), sigx(0), sigy(0), sigxy(0) {}

GaussianAE::GaussianAE(const vEvent &v) : LabelledAE(v)
{
    const GaussianAE * v2 = dynamic_cast<const GaussianAE *>(&v);
    if(v2) {
        sigx = v2->sigx;
        sigy = v2->sigy;
        sigxy = v2->sigxy;
    }
}

GaussianAE::GaussianAE(const GaussianAE &v) : LabelledAE(v)
{
    sigx = v.sigx;
    sigy = v.sigy;
    sigxy = v.sigxy;
}

event<> GaussianAE::clone()
{
    return std::make_shared<GaussianAE>(*this);
}

void GaussianAE::encode(yarp::os::Bottle &b) const
{
    LabelledAE::encode(b);
    b.addInt(*(int*)(&sigx));
    b.addInt(*(int*)(&sigy));
    b.addInt(*(int*)(&sigxy));
}

bool GaussianAE::decode(const yarp::os::Bottle &packet, int &pos)
{
    if (LabelledAE::decode(packet, pos) && pos + 3 <= packet.size())
    {
        int word;
        word = packet.get(pos).asInt();
        sigx=*(float*)(&word);
        word = packet.get(pos+1).asInt();
        sigy=*(float*)(&word);
        word = packet.get(pos+2).asInt();
        sigxy=*(float*)(&word);
        pos+=3;
        return true;
    }
    return false;
}

yarp::os::Property GaussianAE::getContent() const
{
    yarp::os::Property prop = LabelledAE::getContent();
    prop.put("Sigma X^2", sigx);
    prop.put("Sigma Y^2", sigy);
    prop.put("Sigma XY", sigxy);
    return prop;
}

std::string GaussianAE::getType() const
{
    return GaussianAE::tag;
}


}
