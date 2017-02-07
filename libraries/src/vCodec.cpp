/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini, edited by Arren Glover(10/14)
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <algorithm>
#include <iCub/eventdriven/vCodec.h>
#include <cmath>

namespace ev
{

event<> createEvent(const std::string type)
{
    vEvent * ret = nullptr;

    ret = new AddressEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new AddressEventClustered();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new ClusterEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new ClusterEventGauss();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new CollisionEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new FlowEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new InterestEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);

    ret = new NeuronIDEvent();
    if(type == ret->getType()) return event<>(ret);
    else delete(ret);
    ret = nullptr;

    return event<>(nullptr);

}

/******************************************************************************/
//vEvent
/******************************************************************************/
void vEvent::encode(yarp::os::Bottle &b) const
{
#ifdef TIME32BIT
    b.addInt(stamp&0x7FFFFFFF);
#else
    b.addInt((32<<26)|(stamp&0x00ffffff));
#endif
}

/******************************************************************************/
bool vEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    if(pos + localWordsCoded <= packet.size()) {

        //TODO: this needs to take into account the code aswell
#ifdef TIME32BIT
        stamp = packet.get(pos).asInt()&0x00ffffff;
#else
        stamp = packet.get(pos).asInt()&0x7FFFFFFF;
#endif
        pos += localWordsCoded;
        return true;
    }
    return false;

}

vEvent::vEvent(const vEvent &event)
{
    stamp = event.stamp;
}

/******************************************************************************/
vEvent &vEvent::operator=(const vEvent &event)
{
    stamp = event.stamp;
    return *this;
}

vEvent *vEvent::clone()
{
    return new vEvent(*this);
}

/******************************************************************************/
bool vEvent::operator==(const vEvent &event)
{
    return
    (
    this->getType() == event.getType() &&
    this->stamp == event.stamp
    );
}

/******************************************************************************/
yarp::os::Property vEvent::getContent() const
{
    yarp::os::Property prop;
    prop.put("type", getType().c_str());
    prop.put("stamp", (int)stamp);

    return prop;
}

/******************************************************************************/
//AddressEvent
/******************************************************************************/
AddressEvent::AddressEvent(const vEvent &event)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &AddressEvent::operator=(const vEvent &event)
{

    //copy timestamp and type (base class =operator)
    vEvent::operator =(event);

    //copy other fields if it's compatible
    const AddressEvent * aep = dynamic_cast<const AddressEvent *>(&event);
    if(aep) {
        channel=aep->channel;
        polarity=aep->polarity;
        x=aep->x;
        y=aep->y;
    } else {
        channel = 0;
        polarity = 0;
        x = 0;
        y = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* AddressEvent::clone() {
    return new AddressEvent(*this);
}

/******************************************************************************/
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

/******************************************************************************/
bool AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + localWordsCoded <= packet.size())
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

        pos += localWordsCoded;
        return true;
    }
    return false;
}



/******************************************************************************/
bool AddressEvent::operator==(const AddressEvent &event)
{
    return ((vEvent::operator==(event)) &&
            (channel==event.channel)&&
            (polarity==event.polarity)&&
            (x==event.x)&&
            (y==event.y));
}

/******************************************************************************/
yarp::os::Property AddressEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("channel", (int)channel);
    prop.put("polarity", (int)polarity);
    prop.put("x", (int)x);
    prop.put("y", (int)y);

    return prop;
}

/******************************************************************************/
//AddressEventClustered
/******************************************************************************/
AddressEventClustered::AddressEventClustered(const vEvent &event/*always vEvent*/)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &AddressEventClustered::operator=(const vEvent &event/*always vEvent*/)
{

    //copy timestamp and type (base class =operator)
    AddressEvent::operator =(event);

    //copy other fields if it's compatible
    const AddressEventClustered * aep =
            dynamic_cast<const AddressEventClustered *>(&event);
    if(aep) {
        clID = aep->clID;
    } else {
        clID = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* AddressEventClustered::clone() {
    return new AddressEventClustered(*this);
}

/******************************************************************************/
void AddressEventClustered::encode(yarp::os::Bottle &b) const
{
    AddressEvent::encode(b);
    b.addInt(clID);
}

/******************************************************************************/
bool AddressEventClustered::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (AddressEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();
        clID = word0;
        pos += localWordsCoded;
        return true;
    }

    return false;
}



/******************************************************************************/
bool AddressEventClustered::operator==(const AddressEventClustered &event)
{
    return ((AddressEvent::operator==(event)) &&
            (clID==event.clID));
}

/******************************************************************************/
yarp::os::Property AddressEventClustered::getContent() const
{
    yarp::os::Property prop = AddressEvent::getContent();
    prop.put("clID",clID);

    return prop;
}

/******************************************************************************/
//CollisionEvent
/******************************************************************************/
CollisionEvent::CollisionEvent(const vEvent &event/*always vEvent*/)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &CollisionEvent::operator=(const vEvent &event/*always vEvent*/)
{

    //copy timestamp and type (base class =operator)
    vEvent::operator =(event);

    //copy other fields if it's compatible
    const CollisionEvent * v =
            dynamic_cast<const CollisionEvent *>(&event);
    if(v) {
        x = v->x;
        y = v->y;
        channel = v->channel;
        clid1 = v->clid1;
        clid2 = v->clid2;
    } else {
        x = 0;
        y = 0;
        channel = 0;
        clid1 = 0;
        clid2 = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* CollisionEvent::clone() {
    return new CollisionEvent(*this);
}

/******************************************************************************/
void CollisionEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt((clid1&0xff)<<24 | (clid2&0xff)<<16 | (channel&0x01)<<15 |
             (x&0x7f)<<8 | (y&0x7f));
}


/******************************************************************************/
bool CollisionEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {
        int word0 = packet.get(pos).asInt();

        y = word0&0x7f; word0 >>= 8;
        x = word0&0x7f; word0 >>= 7;
        channel = word0&0x01; word0 >>= 1;
        clid2 = word0&0xff; word0 >>= 8;
        clid1 = word0&0xff; word0 >>= 8;

        pos += localWordsCoded;
        return true;
    }

    return false;
}



/******************************************************************************/
bool CollisionEvent::operator==(const CollisionEvent &event)
{
    return (vEvent::operator==(event) &&
            y == event.y &&
            x == event.x &&
            channel == event.channel &&
            clid1 == event.clid1 &&
            clid2 == event.clid2);
}

/******************************************************************************/
yarp::os::Property CollisionEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("x", (int)x);
    prop.put("y", (int)y);
    prop.put("channel", (int)channel);
    prop.put("clid1", (int)clid1);
    prop.put("clid2", (int)clid2);

    return prop;
}

/******************************************************************************/
//ClusterEvent
/******************************************************************************/
ClusterEvent::ClusterEvent(const vEvent &event)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &ClusterEvent::operator=(const vEvent &event)
{

    //copy timestamp and type
    vEvent::operator =(event);

    //copy other fields if it's compatible
    const ClusterEvent * aep = dynamic_cast<const ClusterEvent *>(&event);
    if(aep) {
        channel=aep->channel;
        xCog=aep->xCog;
        yCog=aep->yCog;
        id=aep->id;
        polarity = aep->polarity;
    } else {
        channel = 0;
        xCog = 0;
        yCog = 0;
        id = 0;
        polarity = 1;
    }

    return *this;
}

/******************************************************************************/
vEvent* ClusterEvent::clone() {
    return new ClusterEvent(*this);
}

/******************************************************************************/
void ClusterEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt(((id&0x3FF)<<22)|((yCog&0x3FF)<<12)|((xCog&0x3FF)<<2)|((polarity&0x01)<<1)|(channel&0x01));
}

/******************************************************************************/

bool ClusterEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();

        channel=word0&0x01;
        word0>>=1;

        polarity = word0&0x01;
        word0>>=1;

        xCog=word0&0x3FF;
        word0>>=10;

        yCog=word0&0x3FF;
        word0>>=10;

        id=word0&0x3FF;
        //word0>>10;

        //code=word0&0x3f;
        //word0>>6;

        pos += localWordsCoded;

        return true;
    }
    return false;
}


/******************************************************************************/
bool ClusterEvent::operator==(const ClusterEvent &event)
{
    return ((vEvent::operator ==(event)) &&
            (channel==event.channel)&&
            (polarity==event.polarity)&&
            (id==event.id)&&
            (xCog==event.xCog)&&
            (yCog==event.yCog));
}


/******************************************************************************/
yarp::os::Property ClusterEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("channel", (int)channel);
    prop.put("polarity", (int)polarity);
    prop.put("id",(int)id);
    prop.put("xCog",(int)xCog);
    prop.put("yCog",(int)yCog);

    return prop;
}


/******************************************************************************/
//ClusterEventGauss
/******************************************************************************/
ClusterEventGauss::ClusterEventGauss(const vEvent &event)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &ClusterEventGauss::operator=(const vEvent &event)
{

    //copy timestamp and type
    ClusterEvent::operator =(event);

    //copy other fields if it's compatible
    const ClusterEventGauss * aep = dynamic_cast<const ClusterEventGauss *>(&event);
    if(aep) {
        //this needs to be filled with copy
        numAE = aep->numAE;
        xSigma2 = aep->xSigma2;
        ySigma2 = aep->ySigma2;
        xySigma = aep->xySigma;
        xVel = aep->xVel;
        yVel = aep->yVel;
    } else {
        //this needs the default
        numAE = 0;
        xSigma2 = 0;
        ySigma2 = 0;
        xySigma = 0;
        xVel = 0;
        yVel = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* ClusterEventGauss::clone() {
    return new ClusterEventGauss(*this);
}

/******************************************************************************/
void ClusterEventGauss::encode(yarp::os::Bottle &b) const
{
    ClusterEvent::encode(b);
    b.addInt(((xySigma&0xffff)<<16)|(numAE&0xffff));
    b.addInt(((xSigma2&0xffff)<<16)|(ySigma2&0xffff));
    b.addInt(((xVel&0xffff)<<16)|(yVel&0xffff));
}

/******************************************************************************/

bool ClusterEventGauss::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (ClusterEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {

        int word0=packet.get(pos).asInt();
        int word1=packet.get(pos+1).asInt();
        int word2=packet.get(pos+2).asInt();

//        //assign the decode values here
//        numAE=word0&0x00ffffff;
//        //word0>>=24

//        xSigma2=word1&0xff;
//        word1>>=8;

//        ySigma2=word1&0xff;
//        word1>>=8;

//        xySigma=word1&0xff;

//        xVel=word2&0xff;
//        word2>>=8;

//        yVel=word2&0xff;

        numAE=word0&0xffff;
        word0>>=16;
        xySigma=word0&0xffff;

        ySigma2=word1&0xffff;
        word1>>=16;
        xSigma2=word1&0xffff;

        yVel=word2&0xffff;
        word2>>=16;
        xVel=word2&0xffff;

        pos += localWordsCoded;
        return true;

    }

    return false;
}


/******************************************************************************/
bool ClusterEventGauss::operator==(const ClusterEventGauss &event)
{
    return ((ClusterEvent::operator ==(event))&&
            //add the extra equals operators here new members
            (numAE == event.numAE) &&
            (xSigma2 == event.xSigma2) &&
            (ySigma2 == event.ySigma2) &&
            (xySigma== event.xySigma) &&
            (xVel == event.xVel) &&
            (yVel == event.yVel)
            );
}


/******************************************************************************/
yarp::os::Property ClusterEventGauss::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    //add extra member properties for human readable here
    prop.put("numAE", (int)numAE);
    prop.put("xSigma2", (int)xSigma2);
    prop.put("ySigma2", (int)ySigma2);
    prop.put("xySigma", (int)xySigma);
    prop.put("xVel", (int)xVel);
    prop.put("yVel", (int)yVel);


    return prop;
}

/******************************************************************************/
//FlowEvent
/******************************************************************************/
FlowEvent::FlowEvent(const vEvent &event)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &FlowEvent::operator=(const vEvent &event)
{

    //copy timestamp and type (base class =operator)
    AddressEvent::operator =(event);

    //copy other fields if it's compatible
    const FlowEvent * ofp = dynamic_cast<const FlowEvent *>(&event);
    if(ofp) {
        vx=ofp->vx;
        vy=ofp->vy;
        death = ofp->death;
    } else {
        vx = 0;
        vy = 0;
        death = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* FlowEvent::clone() {
    return new FlowEvent(*this);
}

/******************************************************************************/
void FlowEvent::encode(yarp::os::Bottle &b) const
{
    AddressEvent::encode(b);
    b.addInt(*(int*)(&vx));
    b.addInt(*(int*)(&vy));
    b.addInt(death);
}

/******************************************************************************/
bool FlowEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (AddressEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {
        int word1=packet.get(pos).asInt();
        vx=*(float*)(&word1);
        int word2=packet.get(pos+1).asInt();
        vy=*(float*)(&word2);
        death = packet.get(pos+2).asInt();

        pos+=localWordsCoded;
        return true;
    }
    return false;
}

/******************************************************************************/
bool FlowEvent::operator==(const FlowEvent &event)
{
    return ((AddressEvent::operator==(event)) &&
            (vx==event.vx)&&
            (vy==event.vy)&&
            (death==event.death));
}

/******************************************************************************/
yarp::os::Property FlowEvent::getContent() const
{
    yarp::os::Property prop = AddressEvent::getContent();
    prop.put("vx",vx);
    prop.put("vy",vy);
    prop.put("death", death);

    return prop;
}

void FlowEvent::setDeath()
{
    death = 1.0 / (sqrt(pow(vx, 2.0f) + pow(vy, 2.0f)) * vtsHelper::tstosecs());
    //if(death > 2000000) {
    //    death = 2000000;
    //}
    //if(death < 100000) death = 100000;
    death += stamp;
    return;

    double theta = fabs(atan2(vy, vx));
    if(theta > 1.5707963) theta -= 1.5707963;
    if(theta > 0.785398) theta = 1.5707963 - theta;
    //death = 7.8125*(int)(sqrt(pow(vx, 2.0) + pow(vy, 2.0)) / (cos(theta)*vtsHelper::tstosecs()));
    death = 1.0 / (sqrt(pow(vx, 2.0f) + pow(vy, 2.0f)) * cos(theta) * vtsHelper::tstosecs());
    if(death > 2000000) {
        death = 2000000;
    }
    if(death < 500000) death = 500000;
    death += stamp;

    //double f = std::min(fabs(vx), fabs(vy));
    //death = 7.8125 * f / vtsHelper::tstosecs();
//    death = 7.8125* (int)(sqrt(pow(vx, 2.0) + pow(vy, 2.0)) / vtsHelper::tstosecs());
//    if(death > 1000000) death = 1000000;
//    death += stamp;
}

/******************************************************************************/
//InterestEvent
/******************************************************************************/
InterestEvent::InterestEvent(const vEvent &event/*always vEvent*/)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &InterestEvent::operator=(const vEvent &event/*always vEvent*/)
{

    //copy timestamp and type (base class =operator)
    AddressEvent::operator =(event);

    //copy other fields if it's compatible
    const InterestEvent * aep =
            dynamic_cast<const InterestEvent *>(&event);
    if(aep) {
        intID = aep->intID;
    } else {
        intID = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* InterestEvent::clone() {
    return new InterestEvent(*this);
}

/******************************************************************************/
void InterestEvent::encode(yarp::os::Bottle &b) const
{
    AddressEvent::encode(b);
    b.addInt(intID);
}

/******************************************************************************/
bool InterestEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (AddressEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();
        intID = word0;
        pos += localWordsCoded;
        return true;
    }

    return false;
}



/******************************************************************************/
bool InterestEvent::operator==(const InterestEvent &event)
{
    return ((AddressEvent::operator==(event)) &&
            (intID==event.intID));
}

/******************************************************************************/
yarp::os::Property InterestEvent::getContent() const
{
    yarp::os::Property prop = AddressEvent::getContent();
    prop.put("intID",intID);

    return prop;
}

/******************************************************************************/
//NeuronIDEvent
/******************************************************************************/
NeuronIDEvent::NeuronIDEvent(const vEvent &event/*always vEvent*/)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &NeuronIDEvent::operator=(const vEvent &event/*always vEvent*/)
{

    //copy timestamp and type (base class =operator)
    vEvent::operator =(event);

    //copy other fields if it's compatible
    const NeuronIDEvent * np =
            dynamic_cast<const NeuronIDEvent *>(&event);
    if(np) {
        neurID = np->neurID;
    } else {
        neurID = 0;
    }

    return *this;
}

/******************************************************************************/
vEvent* NeuronIDEvent::clone() {
    return new NeuronIDEvent(*this);
}

/******************************************************************************/
void NeuronIDEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt(neurID);
}

/******************************************************************************/
bool NeuronIDEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) &&
            pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();
        neurID = word0;
        pos += localWordsCoded;
        return true;
    }

    return false;
}



/******************************************************************************/
bool NeuronIDEvent::operator==(const NeuronIDEvent &event)
{
    return ((NeuronIDEvent::operator==(event)) &&
            (neurID==event.neurID));
}

/******************************************************************************/
yarp::os::Property NeuronIDEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("N_ID", neurID);

    return prop;
}

}




