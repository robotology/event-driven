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

    ret = new ClusterEvent();
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
emorph::vEvent *vEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * e = 0;
    if(pos + nBytesCoded() <= packet.size()) {
        e = new vEvent();

        //TODO: this needs to take into account the code aswell
        e->stamp = packet.get(pos).asInt()&0x00ffffff;;
        pos++;
    }
    return e;

}

/******************************************************************************/
vEvent &vEvent::operator=(const vEvent &event)
{
    type = event.type;
    stamp = event.stamp;

    return *this;
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
yarp::os::Bottle AddressEvent::encode() const
{
    int word0=(0<<26)|((channel&0x01)<<15)|((y&0xff)<<8)|((x&0x7f)<<1)|
            (polarity&0x01);

    Bottle ret = vEvent::encode();
    ret.addInt(word0);
    return ret;
}

/******************************************************************************/
vEvent *AddressEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * ee = vEvent::decode(packet, pos);
    AddressEvent * ae = 0;

    // check length
    if (ee && pos+nBytesCoded() <= packet.size())
    {
        ae = new AddressEvent(*ee);
        int word0=packet.get(pos).asInt();

        ae->polarity=word0&0x01;

        word0>>=1;
        ae->x=word0&0x7f;

        word0>>=7;
        ae->y=word0&0x7f;

        word0>>=7;
        ae->channel=word0&0x01;

        pos++;
    }
    if(ee) delete ee;

    return ae;
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
yarp::os::Bottle AddressEventClustered::encode() const
{
    int word0=clID;

    Bottle ret = AddressEvent::encode();
    ret.addInt(word0);
    return ret;
}

/******************************************************************************/
vEvent *AddressEventClustered::decode(const yarp::os::Bottle &packet, int &pos)
{
    /*always vEvent*/vEvent * ee = AddressEvent::decode(packet, pos);
    AddressEventClustered * aec = 0;

    // check length
    if (ee && pos+nBytesCoded() <= packet.size())
    {
        aec = new AddressEventClustered(*ee);
        int word0=packet.get(pos).asInt();
        aec->clID = word0;

        pos++;
    }
    if(ee) delete ee;

    return aec;
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
    } else {
        channel = 0;
        xCog = 0;
        yCog = 0;
        id = 0;
    }

    //force the type to addressevent
    type = "CLE";

    return *this;
}

/******************************************************************************/
yarp::os::Bottle ClusterEvent::encode() const
{
    int word0=(8<<26)|((id&0x03ff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|
            (channel&0x01);

    yarp::os::Bottle ret = vEvent::encode();
    ret.addInt(word0);
    return ret;
}

/******************************************************************************/

vEvent * ClusterEvent::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * ee = vEvent::decode(packet, pos);
    ClusterEvent * cle = 0;

    // check length
    if ( ee && pos+nBytesCoded() <= packet.size())
    {
        cle = new ClusterEvent(*ee);
        int word0=packet.get(pos).asInt();

        channel=word0&0x01;

        word0>>=1;
        xCog=word0&0x7f;

        word0>>=7;
        yCog=word0&0xff;
        word0>>=8;
        id=word0&0x03ff;

        pos++;


    }
    if(ee) delete ee;

    return cle;
}


/******************************************************************************/
bool ClusterEvent::operator==(const ClusterEvent &event)
{
    return ((vEvent::operator ==(event)) &&
            (channel==event.channel)&&
            (id==event.id)&&
            (xCog==event.xCog)&&
            (yCog==event.yCog));
}


/******************************************************************************/
Property ClusterEvent::getContent() const
{
    Property prop = vEvent::getContent();
    prop.put("channel",channel);
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
    } else {
        //this needs the default
        numAE = 0;
        xSigma2 = 0;
        ySigma2 = 0;
        xySigma = 0;
    }

    //force the type to addressevent
    type = "CLEG";

    return *this;
}

/******************************************************************************/
yarp::os::Bottle ClusterEventGauss::encode() const
{
    //the encoding needs to happen here from the old codec
    int word0=(20<<26)|(numAE&0x00ffffff);
    int word1=(21<<26)|((xySigma&0xff)<<16)|((ySigma2&0xff)<<8)|(xSigma2&0xff);

    yarp::os::Bottle ret = ClusterEvent::encode();
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}

/******************************************************************************/

vEvent * ClusterEventGauss::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * ee = ClusterEvent::decode(packet, pos);
    ClusterEventGauss * cleg = 0;

    // check length
    if ( ee && pos+nBytesCoded() <= packet.size())
    {
        //need the decoding here
        cleg = new ClusterEventGauss(*ee);
        int word0=packet.get(pos).asInt();
        int word1=packet.get(pos+1).asInt();

        //assign the decode values here
        numAE=word0&0x00ffffff;

        xSigma2=word1&0xff;

        word1>>=8;
        ySigma2=word1&0xff;

        word1>>=8;
        xySigma=word1&0xff;




        pos += nBytesCoded();


    }
    if(ee) delete ee;

    return cleg;
}


/******************************************************************************/
bool ClusterEventGauss::operator==(const ClusterEventGauss &event)
{
    return ((ClusterEvent::operator ==(event))&&
            //add the extra equals operators here new members
            (numAE == event.numAE) &&
            (xSigma2 == event.xSigma2) &&
            (ySigma2 == event.ySigma2) &&
            (xySigma== event.xySigma)
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


    return prop;
}

}




