// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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
#include "iCub/emorph/vCodec.h"
using namespace emorph;
using namespace yarp::os;

namespace emorph
{

vEvent * createEvent(const std::string type)
{
    vEvent * ret = 0;

    ret = new AddressEvent();
    if(type == ret->getType()) return ret;
    else delete(ret);

    ret = new AddressEventClustered();
    if(type == ret->getType()) return ret;
    else delete(ret);

    ret = new ClusterEvent();
    if(type == ret->getType()) return ret;
    else delete(ret);

    ret = new ClusterEventGauss();
    if(type == ret->getType()) return ret;
    else delete(ret);

    ret = new OpticalFlowEvent();
    if(type == ret->getType()) return ret;
    else delete(ret);

    return 0;

}

/******************************************************************************/
//vQueue
/******************************************************************************/
vQueue::~vQueue()
{
    if (owner)
        for (size_t i=0; i<size(); i++)
            if ((*this)[i]!=NULL)
                delete (*this)[i];

    clear();
}

void vQueue::sort() {
    std::sort(begin(), end(), temporalSort);
}


bool vQueue::temporalSort(const vEvent *e1, const vEvent *e2){
    return e1->getStamp() < e2->getStamp();
}

/******************************************************************************/
//vEvent
/******************************************************************************/
yarp::os::Bottle vEvent::encode() const
{
    int word0=(32<<26)|(stamp&0x00ffffff);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}

/******************************************************************************/
bool vEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    if(pos + localWordsCoded <= packet.size()) {

        //TODO: this needs to take into account the code aswell
        stamp = packet.get(pos).asInt()&0x00ffffff;
        pos += localWordsCoded;
        return true;
    }
    return false;

}

vEvent::vEvent(const vEvent &event)
{
    *this = event;

}

/******************************************************************************/
vEvent &vEvent::operator=(const vEvent &event)
{
    type = event.type;
    stamp = event.stamp;

    return *this;
}

vEvent *vEvent::clone() {
    return new vEvent(*this);
}

/******************************************************************************/
bool vEvent::operator==(const vEvent &event)
{
    return
    (
    this->type == event.type &&
    this->stamp == event.stamp
    );
}

/******************************************************************************/
yarp::os::Property vEvent::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("stamp",stamp);

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

    //force the type to addressevent
    type = "AE";

    return *this;
}

/******************************************************************************/
vEvent* AddressEvent::clone() {
    return new AddressEvent(*this);
}

/******************************************************************************/
yarp::os::Bottle AddressEvent::encode() const
{
    int word0=(0<<26)|((channel&0x01)<<15)|((y&0xff)<<8)|((x&0x7f)<<1)|
            (polarity&0x01);

    Bottle ret = vEvent::encode();
    ret.addInt(word0);
    return ret;
}

/******************************************************************************/
bool AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();

        polarity=word0&0x01;

        word0>>=1;
        x=word0&0x7f;

        word0>>=7;
        y=word0&0x7f;

        word0>>=7;
        channel=word0&0x01;

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
Property AddressEvent::getContent() const
{
    Property prop = vEvent::getContent();
    prop.put("channel",channel);
    prop.put("polarity",polarity);
    prop.put("x",x);
    prop.put("y",y);

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

    //force the type to addressevent
    type = "AE-C";

    return *this;
}

/******************************************************************************/
vEvent* AddressEventClustered::clone() {
    return new AddressEventClustered(*this);
}

/******************************************************************************/
yarp::os::Bottle AddressEventClustered::encode() const
{
    int word0=clID;

    Bottle ret = AddressEvent::encode();
    ret.addInt(word0);
    return ret;
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
Property AddressEventClustered::getContent() const
{
    Property prop = AddressEvent::getContent();
    prop.put("clID",clID);

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

    //force the type to addressevent
    type = "CLE";

    return *this;
}

/******************************************************************************/
vEvent* ClusterEvent::clone() {
    return new ClusterEvent(*this);
}

/******************************************************************************/
yarp::os::Bottle ClusterEvent::encode() const
{
    //6bits for code
    //10bits for id
    //7bits for yCog
    //7bits for XCog
    //1bit for polarity
    //1bit for channel
    int word0=(8<<26)|((id&0x02ff)<<16)|((yCog&0x7f)<<9)|((xCog&0x7f)<<2)|
            ((polarity&0x01)<<1)|(channel&0x01);

    yarp::os::Bottle ret = vEvent::encode();
    ret.addInt(word0);
    return ret;
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

        xCog=word0&0x7f;
        word0>>=7;

        yCog=word0&0x7f;
        word0>>=7;

        id=word0&0x02ff;
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
Property ClusterEvent::getContent() const
{
    Property prop = vEvent::getContent();
    prop.put("channel",channel);
    prop.put("polarity", polarity);
    prop.put("id",id);
    prop.put("xCog",xCog);
    prop.put("yCog",yCog);

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

    //force the type to addressevent
    type = "CLEG";

    return *this;
}

/******************************************************************************/
vEvent* ClusterEventGauss::clone() {
    return new ClusterEventGauss(*this);
}

/******************************************************************************/
yarp::os::Bottle ClusterEventGauss::encode() const
{
    //the encoding needs to happen here from the old codec
    //code 6
    //activity 24
    //int word0=(20<<26)|(numAE&0x00ffffff);
    //code 6
    //xySig 8
    //ySig 8
    //xSig 8
    //int word1=(21<<26)|((xySigma&0xff)<<16)|((ySigma2&0xff)<<8)|(xSigma2&0xff);
    //code 6
    //yVel 8
    //xVel 8
    //int word2=(22<<26)|((yVel&0xff)<<8)|(xVel&0xff);


    int word0 = ((xySigma&0xffff)<<16)|(numAE&0xffff);
    int word1 = ((xSigma2&0xffff)<<16)|(ySigma2&0xffff);
    int word2 = ((xVel&0xffff)<<16)|(yVel&0xffff);

    yarp::os::Bottle ret = ClusterEvent::encode();
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    return ret;
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
Property ClusterEventGauss::getContent() const
{
    Property prop = vEvent::getContent();
    //add extra member properties for human readable here
    prop.put("numAE", numAE);
    prop.put("xSigma2", xSigma2);
    prop.put("ySigma2", ySigma2);
    prop.put("xySigma", xySigma);
    prop.put("xVel", xVel);
    prop.put("yVel", yVel);


    return prop;
}

/******************************************************************************/
//OpticalFlowEvent
/******************************************************************************/
OpticalFlowEvent::OpticalFlowEvent(const vEvent &event)
{
    //most of the constructor is replicated in the assignment operator
    //so we just use that to construct
    *this = event;

}

/******************************************************************************/
vEvent &OpticalFlowEvent::operator=(const vEvent &event)
{

    //copy timestamp and type (base class =operator)
    vEvent::operator =(event);

    //copy other fields if it's compatible
    const OpticalFlowEvent * ofp = dynamic_cast<const OpticalFlowEvent *>(&event);
    if(ofp) {
        channel=ofp->channel;
        x=ofp->x;
        y=ofp->y;
        vx=ofp->vx;
        vy=ofp->vy;
    } else {
        channel = 0;
        x = 0;
        y = 0;
        vx = 0;
        vy = 0;
    }

    //force the type to OpticalFlowEvent
    type = "OFE";

    return *this;
}

/******************************************************************************/
vEvent* OpticalFlowEvent::clone() {
    return new OpticalFlowEvent(*this);
}

/******************************************************************************/
yarp::os::Bottle OpticalFlowEvent::encode() const
{
    //32bits for vy
    //32bits for vx
    //8bits for y
    //8bits for x
    //1bit for polarity
    //1bit for channel

    int word0=((y&0xff)<<10)|((x&0xff)<<2)|((polarity&0x01)<<1)|(channel&0x01);
    int word1=*(int*)(&vx);
    int word2=*(int*)(&vy);

    Bottle ret = vEvent::encode();
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    return ret;
}

/******************************************************************************/
bool OpticalFlowEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + localWordsCoded <= packet.size())
    {
        int word0=packet.get(pos).asInt();
        int word1=packet.get(pos+1).asInt();
        int word2=packet.get(pos+2).asInt();

        channel=word0&0x01;
        word0>>=1;

        polarity=word0&0x01;
        word0>>2;

        x=word0&0xff;
        word0>>=10;

        y=word0&0xff;

        vx=*(float*)(&word1);

        vy=*(float*)(&word2);

        pos+=localWordsCoded;
        return true;
    }
    return false;
}

/******************************************************************************/
bool OpticalFlowEvent::operator==(const OpticalFlowEvent &event)
{
    return ((vEvent::operator==(event)) &&
            (channel==event.channel)&&
            (x==event.x)&&
            (y==event.y))&&
            (vx==event.vx)&&
            (vy==event.vy);
}

/******************************************************************************/
Property OpticalFlowEvent::getContent() const
{
    Property prop = vEvent::getContent();
    prop.put("channel",channel);
    prop.put("x",x);
    prop.put("y",y);
    prop.put("vx",vx);
    prop.put("vy",vy);

    return prop;
}

}




