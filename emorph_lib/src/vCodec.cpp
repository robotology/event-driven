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
AddressEvent::AddressEvent()
{
    type="AE";
    stamp = 0;

    channel=0;
    polarity=0;
    x=0;
    y=0;
}

/******************************************************************************/
AddressEvent::AddressEvent(const AddressEvent &event)
{
    type=event.type;
    stamp = event.stamp;

    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
}

/******************************************************************************/
AddressEvent::AddressEvent(const vEvent &event)// : AddressEvent()
{
    type="AE";
    //stamp = 0;
    
    channel=0;
    polarity=0;
    x=0;
    y=0;

    stamp = event.getStamp();
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
vEvent &AddressEvent::operator=(const vEvent &event)
{
    vEvent::operator =(event);

    const AddressEvent * aep = dynamic_cast<const AddressEvent *>(&event);
    if(aep) {
        channel=aep->channel;
        polarity=aep->polarity;
        x=aep->x;
        y=aep->y;
     }

    return *this;
}

/******************************************************************************/
bool AddressEvent::operator==(const AddressEvent &event)
{
    return ((vEvent::operator==(event)) && //checks type and stamp
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
//AddressEventFeatures
/******************************************************************************/
AddressEventFeatures::AddressEventFeatures() : AddressEvent()
{
    type="AE-F";
    orientation=0;
    xFlow=0;
    yFlow=0;
}


/******************************************************************************/
AddressEventFeatures::AddressEventFeatures(const AddressEventFeatures &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    orientation=event.orientation;
    xFlow=event.xFlow;
    yFlow=event.yFlow;
    stamp = event.stamp;
}


/******************************************************************************/
AddressEventFeatures::AddressEventFeatures(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();

        // check type and fill fields
        if (((word0>>26)==2)&&((word1>>26)==3))
        {
            // word0
            polarity=word0&0x01;

            word0>>=1;
            x=word0&0x7f;

            word0>>=7;
            y=word0&0xff;

            word0>>=8;
            channel=word0&0x01;

            // word1
            orientation=word1&0xff;

            word1>>=8;
            xFlow=word1&0xff;

            word1>>=8;
            yFlow=word1&0xff;

            type="AE-F";
            valid=true;
        }
    }
}


/******************************************************************************/
AddressEventFeatures &AddressEventFeatures::operator=(const AddressEventFeatures &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    orientation=event.orientation;
    xFlow=event.xFlow;
    yFlow=event.yFlow;
    stamp = event.stamp;
    
    return *this;
}


/******************************************************************************/
bool AddressEventFeatures::operator==(const AddressEventFeatures &event)
{
    return (
            (valid==event.valid)&&
            (type==event.type)&&
            (channel==event.channel)&&
            (polarity==event.polarity)&&
            (x==event.x)&&(y==event.y)&&
            (orientation==event.orientation)&&
            (xFlow==event.xFlow)&&
            (yFlow==event.yFlow)&&
            (stamp==event.stamp)
            );
}


/******************************************************************************/
Bottle AddressEventFeatures::encode() const
{
    int word0=(2<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(3<<26)|((yFlow&0xff)<<16)|((xFlow&0xff)<<8)|(orientation&0xff);

    Bottle ret = vEvent::encode();
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/******************************************************************************/
Property AddressEventFeatures::getContent() const
{
    Property prop=AddressEvent::getContent();
    prop.put("orientation",orientation);
    prop.put("xFlow",xFlow);
    prop.put("yFlow",yFlow);

    return prop;
}

    
/******************************************************************************/
//AddressEventCluster
/******************************************************************************/
AddressEventCluster::AddressEventCluster() //: AddressEvent()
{
    type="AE-C";

    channel=0;
    polarity=0;
    x=0;
    y=0;
    
    clId=0;
}

/******************************************************************************/
AddressEventCluster::~AddressEventCluster()
{

}


/******************************************************************************/
AddressEventCluster::AddressEventCluster(const AddressEventCluster &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    clId=event.clId;
    stamp = event.stamp;
}

/******************************************************************************/
AddressEventCluster::AddressEventCluster(const AddressEvent &event)
{
    valid=true;
    type="AE-C";
    channel=event.getChannel();
    polarity=event.getPolarity();
    x=event.getX();
    y=event.getY();
    clId=0;
    stamp = event.getStamp();
}

/******************************************************************************/
vEvent *AddressEventCluster::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * ee = vEvent::decode(packet, pos);
    AddressEventCluster * ae = 0;
    
    // check length
    if (ee && pos+nBytesCoded() <= packet.size())
    {
        ae = new AddressEventCluster(*ee);
        int word0=packet.get(pos).asInt();
        int word1=packet.get(pos+1).asInt();
      
        ae->polarity=word0&0x01;
        
        word0>>=1;
        ae->x=word0&0x7f;
        
        word0>>=7;
        ae->y=word0&0x7f;
        
        word0>>=7;
        ae->channel=word0&0x01;
        
        //word1
        ae->clId=word1&0xff;
        
        pos++;
    }
    if(ee) delete ee;
    
    return ae;
}

/******************************************************************************/
AddressEventCluster &AddressEventCluster::operator=(const AddressEventCluster &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    clId=event.clId;
    stamp = event.stamp;
    
    return *this;
}

/******************************************************************************/
AddressEventCluster &AddressEventCluster::operator=(const AddressEvent &event)
{
    valid=true;
    type=event.getType();
    channel=event.getChannel();
    polarity=event.getPolarity();
    x=event.getX();
    y=event.getY();
    clId=0;
    stamp = event.getStamp();

    return *this;
}

/******************************************************************************/
bool AddressEventCluster::operator==(const AddressEventCluster &event)
{
    return ((vEvent::operator==(event))&&
            (valid==event.valid)&&
            (channel==event.channel)&&
            (polarity==event.polarity)&&
            (x==event.x)&&(y==event.y)&&
            (clId==event.clId)
            );
}


/******************************************************************************/
Bottle AddressEventCluster::encode() const
{
    int word0=(4<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(5<<26)|(clId&0xff);
    
    Bottle ret = vEvent::encode(); // time stamp
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/******************************************************************************/
Property AddressEventCluster::getContent() const
{
    Property prop=AddressEvent::getContent();
    prop.put("clId",clId);
    
    return prop;
}

/******************************************************************************/
//ClusterEvent
/******************************************************************************/
ClusterEvent::ClusterEvent()
{
    type    = "CLE";
    stamp   = 0;
    channel = 0;
    xCog    = 0;
    yCog    = 0;
    id      = 0;
}

/******************************************************************************/
ClusterEvent::ClusterEvent(const ClusterEvent &event)
{
    type    = event.type; 
    stamp   = event.stamp;
    channel = event.channel;
    xCog    = event.xCog;
    yCog    = event.yCog;
    id      = event.id;
}

/******************************************************************************/
ClusterEvent::ClusterEvent(const vEvent &event) //: ClusterEvent()
{
    type    = "CLE";
    //stamp   = 0;
    channel = 0;
    xCog    = 0;
    yCog    = 0;
    id      = 0;
    stamp = event.getStamp();
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
ClusterEvent &ClusterEvent::operator=(const ClusterEvent &event)
{

    vEvent::operator =(event);
    channel = event.channel;
    id      = event.id;
    xCog    = event.xCog;
    yCog    = event.yCog;

    return *this;
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
/*   ClusterEventGauss                                                        */
/******************************************************************************/
ClusterEventGauss::ClusterEventGauss() //: ClusterEvent()
{
    type="CLE-G";

    stamp   = 0;
    channel = 0;
    xCog    = 0;
    yCog    = 0;
    id      = 0;

    numAE=0;
    xSigma2=0;
    ySigma2=0;
    xySigma=0;
    xVel=0;
    yVel=0;
}


/******************************************************************************/
ClusterEventGauss::ClusterEventGauss(const ClusterEventGauss &event)
{
    valid=event.valid;
    type=event.type;
    stamp   = event.stamp;
    channel=event.channel;
    id=event.id;
    xCog=event.xCog;
    yCog=event.yCog;
    xSigma2=event.xSigma2;
    ySigma2=event.ySigma2;
    xySigma=event.xySigma;
    xVel=event.xVel;
    yVel=event.yVel;
    numAE=event.numAE;
}

/******************************************************************************/

vEvent * ClusterEventGauss::decode(const yarp::os::Bottle &packet, int &pos)
{
    vEvent * ee = vEvent::decode(packet, pos);
    ClusterEvent * cle = 0;
    
    // check length
    if ( ee && pos+nBytesCoded() <= packet.size())
    {
        cle = new ClusterEvent(*ee);
        int word0=packet.get(pos).asInt();
        int word1=packet.get(pos+1).asInt();
        int word2=packet.get(pos+2).asInt();
        int word3=packet.get(pos+3).asInt();

        channel=word0&0x01;
        
        //word0
        word0>>=1;
        xCog=word0&0x7f;
        
        word0>>=7;
        yCog=word0&0xff;
        word0>>=8;
        id=word0&0x03ff;
        // word1
        numAE=word1&0x00ffffff;
        
        // word2
        xSigma2=word2&0xff;
        
        word2>>=8;
        ySigma2=word2&0xff;
        
        word2>>=8;
        xySigma=word2&0xff;
        
        // word3
        xVel=word3&0xff;
        
        word3>>=8;
        yVel=word3&0xff;
        
        type="CLE-G";

        pos++;
        
        
    }
    if(ee) delete ee;
    
    return cle;
}



/******************************************************************************/
ClusterEventGauss &ClusterEventGauss::operator=(const ClusterEventGauss &event)
{
    vEvent::operator =(event);
    channel=event.channel;
    id=event.id;
    xCog=event.xCog;
    yCog=event.yCog; 
    xSigma2=event.xSigma2;
    ySigma2=event.ySigma2;
    xySigma=event.xySigma;
    xVel=event.xVel;
    yVel=event.yVel;
    numAE=event.numAE;

    return *this;
}


/******************************************************************************/
bool ClusterEventGauss::operator==(const ClusterEventGauss &event)
{

    return ((vEvent::operator ==(event)) &&
            (type==event.type)&&
            (channel==event.channel)&&
            (id==event.id)&&(xCog==event.xCog)&&
            (yCog==event.yCog)&&
            (xSigma2==event.xSigma2)&&
            (ySigma2==event.ySigma2)&&
            (xySigma==event.xySigma)&&
            (xVel==event.xVel)&&
            (yVel==event.yVel)&&
            (numAE==event.numAE)
            );
}


/******************************************************************************/
Bottle ClusterEventGauss::encode() const
{   
    int word0=(19<<26)|((id&0xff)<<16)|((yCog&0x07f)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(20<<26)|(numAE&0x00ffffff);
    int word2=(21<<26)|((xySigma&0xff)<<16)|((ySigma2&0xff)<<8)|(xSigma2&0xff);
    int word3=(22<<26)|((yVel&0xff)<<8)|(xVel&0xff);
    
    yarp::os::Bottle ret = vEvent::encode();
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    ret.addInt(word3);
    return ret;
}


/******************************************************************************/
Property ClusterEventGauss::getContent() const
{
    Property prop=ClusterEvent::getContent(); 
    prop.put("numAE",numAE);
    prop.put("xSigma2",xSigma2);
    prop.put("ySigma2",ySigma2);
    prop.put("xySigma",xySigma);
    prop.put("xVel",xVel);
    prop.put("yVel",yVel);
    
    return prop;
}

}




