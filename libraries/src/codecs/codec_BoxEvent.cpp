#include <yarp/os/Bottle.h>
#include "iCub/eventdriven/vCodec.h"

namespace ev {
    
    const std::string BoxEvent::tag = "BOX";
    
    BoxEvent::BoxEvent() : AddressEvent(){}//,, width(0), height(0)
    
    BoxEvent::BoxEvent(const vEvent &v) : AddressEvent(v) {
        const BoxEvent *v2 = dynamic_cast<const BoxEvent *>(&v);
        if ( v2 ) {
            width = v2->width;
            height = v2->height;
        }
    }
    
    BoxEvent::BoxEvent(const BoxEvent &v) : AddressEvent(v)
    {
        width = v.width;
        height = v.height;
    }
    
    event<> BoxEvent::clone()
    {
        return std::make_shared<BoxEvent>(*this);
    }
    
    void BoxEvent::encode(yarp::os::Bottle &b) const
    {
        AddressEvent::encode(b);
        b.addInt(*(&width));
        b.addInt(*(&height));
    }
    
    void BoxEvent::encode(std::vector<std::int32_t> &b, unsigned int &pos) const
    {
        AddressEvent::encode(b, pos);
        b[pos++] = (*(&width));
        b[pos++] = (*(&height));
    }
    
    bool BoxEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
    {
        // check length
        if (AddressEvent::decode(packet, pos) &&
            pos + 2 <= packet.size())
        {
            int word1=packet.get(pos).asInt();
            width=*(&word1);
            int word2=packet.get(pos+1).asInt();
            height=*(&word2);
            
            pos+=2;
            return true;
        }
        return false;
    }
    
    yarp::os::Property BoxEvent::getContent() const
    {
        yarp::os::Property prop = AddressEvent::getContent();
        prop.put("width",width);
        prop.put("height",height);
        
        return prop;
    }
    
    std::string BoxEvent::getType() const
    {
        return BoxEvent::tag;
    }

    void BoxEvent::decode(int *&data) {
        AddressEvent::decode(data);
    }

}

