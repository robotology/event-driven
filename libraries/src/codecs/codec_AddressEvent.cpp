#include <yarp/os/Bottle.h>
#include "iCub/eventdriven/vCodec.h"

namespace ev {

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
#ifdef TENBITCODEC
    b.addInt(((channel&0x01)<<20)|((y&0x0FF)<<10)|((x&0x1FF)<<1)|(polarity&0x01));
#else
    //b.addInt(((channel&0x01)<<15)|((y&0x7f)<<8)|((x&0x7f)<<1)|(polarity&0x01));
    b.addInt(((channel&0x01)<<15)|((x&0x7f)<<8)|(((127-y)&0x7f)<<1)|(polarity&0x01));
#endif
}

bool AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        int word0=packet.get(pos).asInt();

#ifdef TENBITCODEC
        polarity=word0&0x01;

        word0>>=1;
        x=word0&0x1FF;

        word0>>=9;
        y=word0&0xFF;

        word0>>=10;
        channel=word0&0x01;
#else
        //        polarity=word0&0x01;

        //        word0>>=1;
        //        x=word0&0x7f;

        //        word0>>=7;
        //        y=word0&0x7f;

        //        word0>>=7;
        //        channel=word0&0x01;
        polarity=word0&0x01;

        word0>>=1;
        y=127-(word0&0x7f);

        word0>>=7;
        x=word0&0x7f;

        word0>>=7;
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
    return "AE";
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

