#ifndef __VCODEC_VEVENT__
#define __VCODEC_VEVENT__

#include <algorithm>
#include <yarp/os/Bottle.h>
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
#ifdef TIME32BIT
    b.addInt(stamp&0x7FFFFFFF);
#else
    b.addInt((32<<26)|(stamp&0x00ffffff));
#endif
}

bool vEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    if(pos + 1 <= packet.size()) {

        //TODO: this needs to take into account the code aswell
#ifdef TIME32BIT
        stamp = packet.get(pos).asInt()&0x7FFFFFFF;
#else
        stamp = packet.get(pos).asInt()&0x00ffffff;
#endif
        pos += 1;
        return true;
    }
    return false;

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


bool temporalSortStraight(const event<> &e1, const event<> &e2) {
    return e2->stamp > e1->stamp;
}

bool temporalSortWrap(const event<> &e1, const event<> &e2)
{

    if(std::abs(e1->stamp - e2->stamp) > vtsHelper::max_stamp/2)
        return e1->stamp > e2->stamp;
    else
        return e2->stamp > e1->stamp;

}

void qsort(vQueue &q, bool respectWraps)
{

    if(respectWraps)
        std::sort(q.begin(), q.end(), temporalSortWrap);
    else
        std::sort(q.begin(), q.end(), temporalSortStraight);

}

event<> createEvent(const std::string &type)
{

    if(type == AddressEvent::tag)
        return make_event<AE>();
    if(type == LabelledAE::tag)
        return make_event<LabelledAE>();
    if(type == FlowEvent::tag)
        return make_event<FlowEvent>();
    if(type == GaussianAE::tag)
        return make_event<GaussianAE>();
    return event<>(nullptr);

}


}



#endif
