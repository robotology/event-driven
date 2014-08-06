// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
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

#include "eventCodec.h"

using namespace yarp::os;
using namespace emorph::ecodec;


namespace emorph
{

namespace ecodec
{


/**************************************************************************/
eEvent *factoryTimeStamp(const Bottle &packets, const int pos=0)
{
    TimeStamp *pEvent=new TimeStamp(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryAddressEvent(const Bottle &packets, const int pos=0)
{
    AddressEvent *pEvent=new AddressEvent(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryAddressEventFeatures(const Bottle &packets, const int pos=0)
{
    AddressEventFeatures *pEvent=new AddressEventFeatures(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryAddressEvent3D(const Bottle &packets, const int pos=0)
{
    AddressEvent3D *pEvent=new AddressEvent3D(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryAddressEvent3DFeatures(const Bottle &packets, const int pos=0)
{
    AddressEvent3DFeatures *pEvent=new AddressEvent3DFeatures(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEvent(const Bottle &packets, const int pos=0)
{
    ClusterEvent *pEvent=new ClusterEvent(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEventFeatures1(const Bottle &packets, const int pos=0)
{
    ClusterEventFeatures1 *pEvent=new ClusterEventFeatures1(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEventFeatures2(const Bottle &packets, const int pos=0)
{
    ClusterEventFeatures2 *pEvent=new ClusterEventFeatures2(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEventFeatures3(const Bottle &packets, const int pos=0)
{
    ClusterEventFeatures3 *pEvent=new ClusterEventFeatures3(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEvent3D(const Bottle &packets, const int pos=0)
{
    ClusterEvent3D *pEvent=new ClusterEvent3D(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEvent3DFeatures1(const Bottle &packets, const int pos=0)
{
    ClusterEvent3DFeatures1 *pEvent=new ClusterEvent3DFeatures1(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEvent3DFeatures2(const Bottle &packets, const int pos=0)
{
    ClusterEvent3DFeatures2 *pEvent=new ClusterEvent3DFeatures2(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


/**************************************************************************/
eEvent *factoryClusterEvent3DFeatures3(const Bottle &packets, const int pos=0)
{
    ClusterEvent3DFeatures3 *pEvent=new ClusterEvent3DFeatures3(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}

/**************************************************************************/
eEvent *factoryHoughEvent(const Bottle &packets, const int pos=0)
{
    HoughEvent *pEvent=new HoughEvent(packets,pos);
    if (!pEvent->isValid())
    {
        delete pEvent;
        return NULL;
    }
    else
        return static_cast<eEvent*>(pEvent);
}


}

}


/**************************************************************************/
bool eEvent::decode(const Bottle &packets, eEventQueue &events)
{
    int pos=0;
    while (pos<packets.size())
    {
        eEvent *pEvent=NULL;
        if      (pEvent = factoryTimeStamp(packets,pos))               { }
        else if (pEvent = factoryAddressEvent(packets,pos))            { }
        else if (pEvent = factoryAddressEventFeatures(packets,pos))    { }
        else if (pEvent = factoryAddressEvent3D(packets,pos))          { }
        else if (pEvent = factoryAddressEvent3DFeatures(packets,pos))  { }
        else if (pEvent = factoryClusterEvent(packets,pos))            { }
        else if (pEvent = factoryClusterEventFeatures1(packets,pos))   { }
        else if (pEvent = factoryClusterEventFeatures2(packets,pos))   { }
        else if (pEvent = factoryClusterEventFeatures3(packets,pos))   { }
        else if (pEvent = factoryClusterEvent3D(packets,pos))          { }
        else if (pEvent = factoryClusterEvent3DFeatures1(packets,pos)) { }
        else if (pEvent = factoryClusterEvent3DFeatures2(packets,pos)) { }
        else if (pEvent = factoryClusterEvent3DFeatures3(packets,pos)) { }
        else if (pEvent = factoryHoughEvent(packets,pos))              { }


        if (pEvent==NULL) {
            return false;
        }

        events.push_back(pEvent);
        pos+=pEvent->getLength();
    }

    return true;
}

/**************************************************************************/
std::string eEvent::getType() {

    if(type.c_str() == NULL) {
        printf("type null \n");
        return "null!";
    }

    return type;

}


/**************************************************************************/
eEventQueue::~eEventQueue()
{
    if (owner)
        for (size_t i=0; i<size(); i++)
            if ((*this)[i]!=NULL)
                delete (*this)[i];

    clear();
}


/**************************************************************************/
TimeStamp::TimeStamp()
{
    valid=true;
    type="TS";
    stamp=0;
}


/**************************************************************************/
TimeStamp::TimeStamp(const TimeStamp &event)
{
    valid=event.valid;
    type=event.type;
    stamp=event.stamp;
}


/**************************************************************************/
TimeStamp::TimeStamp(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        // whenever the 2^5 bit is on
        // change sign
        if ((word0>>26)==-32)
        {
            stamp=word0&0x00ffffff;

            type="TS";
            valid=true;
        }
    }
}


/**************************************************************************/
TimeStamp &TimeStamp::operator=(const TimeStamp &event)
{
    valid=event.valid;
    type=event.type;
    stamp=event.stamp;

    return *this;
}


/**************************************************************************/
bool TimeStamp::operator==(const TimeStamp &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(stamp==event.stamp));
}


/**************************************************************************/
Bottle TimeStamp::encode() const
{
    int word0=(32<<26)|(stamp&0x00ffffff);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property TimeStamp::getContent() const
{
    yarp::os::Property prop;
    prop.put("type",type.c_str());
    prop.put("stamp",stamp);

    return prop;
}


/**************************************************************************/
AddressEvent::AddressEvent()
{
    valid=true;
    type="AE";
    channel=0;
    polarity=0;
    x=0;
    y=0;
}


/**************************************************************************/
AddressEvent::AddressEvent(const AddressEvent &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
}


/**************************************************************************/
AddressEvent::AddressEvent(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        if ((word0>>26)==0)
        {
            polarity=word0&0x01;

            word0>>=1;
            x=word0&0x7f;

            word0>>=7;
            y=word0&0x7f;

            word0>>=7;
            channel=word0&0x01;

            type="AE";
            valid=true;
        }
    }
}


/**************************************************************************/
AddressEvent &AddressEvent::operator=(const AddressEvent &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    polarity=event.polarity;
    x=event.x;
    y=event.y;

    return *this;
}


/**************************************************************************/
bool AddressEvent::operator==(const AddressEvent &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (polarity==event.polarity)&&(x==event.x)&&(y==event.y));
}


/**************************************************************************/
Bottle AddressEvent::encode() const
{
    int word0=(0<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property AddressEvent::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("channel",channel);
    prop.put("polarity",polarity);
    prop.put("x",x);
    prop.put("y",y);

    return prop;
}


/**************************************************************************/
AddressEventFeatures::AddressEventFeatures() : AddressEvent()
{
    type="AE-F";
    orientation=0;
    xFlow=0;
    yFlow=0;
}


/**************************************************************************/
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
}


/**************************************************************************/
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


/**************************************************************************/
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

    return *this;
}


/**************************************************************************/
bool AddressEventFeatures::operator==(const AddressEventFeatures &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (polarity==event.polarity)&&(x==event.x)&&(y==event.y)&&
            (orientation==event.orientation)&&(xFlow==event.xFlow)&&(yFlow==event.yFlow));
}


/**************************************************************************/
Bottle AddressEventFeatures::encode() const
{
    int word0=(2<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(3<<26)|((yFlow&0xff)<<16)|((xFlow&0xff)<<8)|(orientation&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
Property AddressEventFeatures::getContent() const
{
    Property prop=AddressEvent::getContent();
    prop.put("orientation",orientation);
    prop.put("xFlow",xFlow);
    prop.put("yFlow",yFlow);

    return prop;
}


/**************************************************************************/
AddressEvent3D::AddressEvent3D()
{
    valid=true;
    type="3D-AE";
    disparity=0;
    polarity=0;
    x=0;
    y=0;
}


/**************************************************************************/
AddressEvent3D::AddressEvent3D(const AddressEvent3D &event)
{
    valid=event.valid;
    type=event.type;
    disparity=event.disparity;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
}


/**************************************************************************/
AddressEvent3D::AddressEvent3D(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        if ((word0>>26)==1)
        {
            polarity=word0&0x01;

            word0>>=1;
            x=word0&0x7f;

            word0>>=7;
            y=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            type="3D-AE";
            valid=true;
        }
    }
}


/**************************************************************************/
AddressEvent3D &AddressEvent3D::operator=(const AddressEvent3D &event)
{
    valid=event.valid;
    type=event.type;
    disparity=event.disparity;
    polarity=event.polarity;
    x=event.x;
    y=event.y;

    return *this;
}


/**************************************************************************/
bool AddressEvent3D::operator==(const AddressEvent3D &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(disparity==event.disparity)&&
            (polarity==event.polarity)&&(x==event.x)&&(y==event.y));
}


/**************************************************************************/
Bottle AddressEvent3D::encode() const
{
    int word0=(1<<26)|((disparity&0xff)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property AddressEvent3D::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("disparity",disparity);
    prop.put("polarity",polarity);
    prop.put("x",x);
    prop.put("y",y);

    return prop;
}


/**************************************************************************/
AddressEvent3DFeatures::AddressEvent3DFeatures() : AddressEvent3D()
{
    type="3D-AE-F";
    orientation=0;
    xFlow=0;
    yFlow=0;
}


/**************************************************************************/
AddressEvent3DFeatures::AddressEvent3DFeatures(const AddressEvent3DFeatures &event)
{
    valid=event.valid;
    type=event.type;
    disparity=event.disparity;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    orientation=event.orientation;
    xFlow=event.xFlow;
    yFlow=event.yFlow;
}


/**************************************************************************/
AddressEvent3DFeatures::AddressEvent3DFeatures(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();

        // check type and fill fields
        if (((word0>>26)==4)&&((word1>>26)==3))
        {
            // word0
            polarity=word0&0x01;

            word0>>=1;
            x=word0&0x7f;

            word0>>=7;
            y=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            // word1
            orientation=word1&0xff;

            word1>>=8;
            xFlow=word1&0xff;

            word1>>=8;
            yFlow=word1&0xff;

            type="3D-AE-F";
            valid=true;
        }
    }
}


/**************************************************************************/
AddressEvent3DFeatures &AddressEvent3DFeatures::operator=(const AddressEvent3DFeatures &event)
{
    valid=event.valid;
    type=event.type;
    disparity=event.disparity;
    polarity=event.polarity;
    x=event.x;
    y=event.y;
    orientation=event.orientation;
    xFlow=event.xFlow;
    yFlow=event.yFlow;

    return *this;
}


/**************************************************************************/
bool AddressEvent3DFeatures::operator==(const AddressEvent3DFeatures &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(disparity==event.disparity)&&
            (polarity==event.polarity)&&(x==event.x)&&(y==event.y)&&
            (orientation==event.orientation)&&(xFlow==event.xFlow)&&
            (yFlow==event.yFlow));
}


/**************************************************************************/
Bottle AddressEvent3DFeatures::encode() const
{
    int word0=(4<<26)|((disparity&0xff)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(3<<26)|((yFlow&0xff)<<16)|((xFlow&0xff)<<8)|(orientation&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
Property AddressEvent3DFeatures::getContent() const
{
    Property prop=AddressEvent3D::getContent();
    prop.put("orientation",orientation);
    prop.put("xFlow",xFlow);
    prop.put("yFlow",yFlow);

    return prop;
}


/**************************************************************************/
ClusterEvent::ClusterEvent()
{
    valid   = true;
    type    = "CLE";
    channel = 0;
    xCog    = 0;
    yCog    = 0;
    id      = 0;
}


/**************************************************************************/
ClusterEvent::ClusterEvent(const ClusterEvent &event)
{
    valid   = event.valid;
    type    = event.type;
    channel = event.channel;
    xCog    = event.xCog;
    yCog    = event.yCog;
    id      = event.id;
}


/**************************************************************************/
ClusterEvent::ClusterEvent(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        if ((word0>>26)==8)
        {
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            id=word0&0x03ff;

            type="CLE";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent &ClusterEvent::operator=(const ClusterEvent &event)
{
    valid   = event.valid;
    type    = event.type;
    channel = event.channel;
    id      = event.id;
    xCog    = event.xCog;
    yCog    = event.yCog;

    return *this;
}


/**************************************************************************/
bool ClusterEvent::operator==(const ClusterEvent &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (id==event.id)&&(xCog==event.xCog)&&(yCog==event.yCog));
}


/**************************************************************************/
Bottle ClusterEvent::encode() const
{
    int word0=(8<<26)|((id&0x03ff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property ClusterEvent::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("channel",channel);
    prop.put("id",id);
    prop.put("xCog",xCog);
    prop.put("yCog",yCog);

    return prop;
}


/**************************************************************************/
ClusterEventFeatures1::ClusterEventFeatures1() : ClusterEvent()
{
    type="CLE-F1";
}


/**************************************************************************/
ClusterEventFeatures1::ClusterEventFeatures1(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();

        // check type and fill fields
        if (((word0>>26)==9)&&((word1>>26)==10))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            // word1
            id=word1&0x00ffffff;

            type="CLE-F1";
            valid=true;
        }
    }
}


/**************************************************************************/
Bottle ClusterEventFeatures1::encode() const
{
    int word0=(9<<26)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(10<<26)|(id&0x00ffffff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
ClusterEventFeatures2::ClusterEventFeatures2() : ClusterEventFeatures1()
{
    type="CLE-F2";
    shapeType=0;
    xSize=0;
    ySize=0;
    numAE=0;
}


/**************************************************************************/
ClusterEventFeatures2::ClusterEventFeatures2(const ClusterEventFeatures2 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    id=event.id;
    shapeType=event.shapeType;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;
    numAE=event.numAE;
}


/**************************************************************************/
ClusterEventFeatures2::ClusterEventFeatures2(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();
        int word2=packets.get(pos+2).asInt();

        // check type and fill fields
        if (((word0>>26)==9)&&((word1>>26)==11)&&((word2>>26)==12))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            id = word0&0xff;

            // word1
            numAE=word1&0x00ffffff;

            // word2
            xSize=word2&0xff;

            word2>>=8;
            ySize=word2&0xff;

            word2>>=8;
            shapeType=word2&0xff;

            type="CLE-F2";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEventFeatures2 &ClusterEventFeatures2::operator=(const ClusterEventFeatures2 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    id=event.id;
    shapeType=event.shapeType;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;
    numAE=event.numAE;

    return *this;
}


/**************************************************************************/
bool ClusterEventFeatures2::operator==(const ClusterEventFeatures2 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (id==event.id)&&(numAE==event.numAE)&&(shapeType==event.shapeType)&&(xCog==event.xCog)&&
            (yCog==event.yCog)&&(xSize==event.xSize)&&(ySize==event.ySize));
}


/**************************************************************************/
Bottle ClusterEventFeatures2::encode() const
{
    int word0=(9<<26) |((id & 0xff)<<16) | ((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(11<<26)|(numAE&0x00ffffff);
    int word2=(12<<26)|((shapeType&0xff)<<16)|((ySize&0xff)<<8)|(xSize&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    return ret;
}


/**************************************************************************/
Property ClusterEventFeatures2::getContent() const
{
    Property prop=ClusterEventFeatures1::getContent();
    prop.put("shapeType",shapeType);
    prop.put("xSize",xSize);
    prop.put("ySize",ySize);
    prop.put("numAE",numAE);

    return prop;
}


/**************************************************************************/
ClusterEventFeatures3::ClusterEventFeatures3() : ClusterEventFeatures2()
{
    type="CLE-F3";
    shapeProb=0;
    xVel=0;
    yVel=0;
    numAE=0;
}


/**************************************************************************/
ClusterEventFeatures3::ClusterEventFeatures3(const ClusterEventFeatures3 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    id=event.id;
    shapeType=event.shapeType;
    shapeProb=event.shapeProb;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;
    xVel=event.xVel;
    yVel=event.yVel;
    numAE=event.numAE;
}


/**************************************************************************/
ClusterEventFeatures3::ClusterEventFeatures3(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();
        int word2=packets.get(pos+2).asInt();
        int word3=packets.get(pos+3).asInt();

        // check type and fill fields
        if (((word0>>26)==9)&&((word1>>26)==11)&&
            ((word2>>26)==13)&&((word3>>26)==14))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            id=word0&0xff;

            // word1
            numAE=word1&0x00ffffff;

            // word2
            xSize=word2&0xff;

            word2>>=8;
            ySize=word2&0xff;

            word2>>=8;
            shapeType=word2&0xff;

            // word3
            xVel=word3&0xff;

            word3>>=8;
            yVel=word3&0xff;

            word3>>=8;
            shapeProb=word3&0xff;

            type="CLE-F3";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEventFeatures3 &ClusterEventFeatures3::operator=(const ClusterEventFeatures3 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    numAE=event.numAE;
    shapeType=event.shapeType;
    shapeProb=event.shapeProb;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;
    xVel=event.xVel;
    yVel=event.yVel;
    numAE=event.numAE;

    return *this;
}


/**************************************************************************/
bool ClusterEventFeatures3::operator==(const ClusterEventFeatures3 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (id==event.id)&&(numAE==event.numAE)&&(shapeType==event.shapeType)&&(shapeProb==event.shapeProb)&&
            (xCog==event.xCog)&&(yCog==event.yCog)&&(xSize==event.xSize)&&(ySize==event.ySize)&&
            (xVel==event.xVel)&&(yVel==event.yVel));
}


/**************************************************************************/
Bottle ClusterEventFeatures3::encode() const
{
    int word0=(9<<26) |((id&0xff)<<8)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(11<<26)|(numAE&0x00ffffff);
    int word2=(13<<26)|((shapeType&0xff)<<16)|((ySize&0xff)<<8)|(xSize&0xff);
    int word3=(14<<26)|((shapeProb&0xff)<<16)|((yVel&0xff)<<8)|(xVel&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    ret.addInt(word3);
    return ret;
}


/**************************************************************************/
Property ClusterEventFeatures3::getContent() const
{
    Property prop=ClusterEventFeatures2::getContent();
    prop.put("shapeProb",shapeProb);
    prop.put("xVel",xVel);
    prop.put("yVel",yVel);
    prop.put("numAE",numAE);

    return prop;
}


/**************************************************************************/
ClusterEvent3D::ClusterEvent3D()
{
    valid=true;
    type="3D-CLE";
    channel=0;
    disparity=0;
    xCog=0;
    yCog=0;
}


/**************************************************************************/
ClusterEvent3D::ClusterEvent3D(const ClusterEvent3D &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    xCog=event.xCog;
    yCog=event.yCog;
}


/**************************************************************************/
ClusterEvent3D::ClusterEvent3D(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        if ((word0>>26)==16)
        {
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            type="3D-CLE";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent3D &ClusterEvent3D::operator=(const ClusterEvent3D &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    xCog=event.xCog;
    yCog=event.yCog;

    return *this;
}


/**************************************************************************/
bool ClusterEvent3D::operator==(const ClusterEvent3D &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (disparity==event.disparity)&&(xCog==event.xCog)&&(yCog==event.yCog));
}


/**************************************************************************/
Bottle ClusterEvent3D::encode() const
{
    int word0=(16<<26)|((disparity&0xff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property ClusterEvent3D::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("channel",channel);
    prop.put("disparity",disparity);
    prop.put("xCog",xCog);
    prop.put("yCog",yCog);

    return prop;
}


/**************************************************************************/
ClusterEvent3DFeatures1::ClusterEvent3DFeatures1() : ClusterEvent3D()
{
    type="3D-CLE-F1";
    numAE=0;
}


/**************************************************************************/
ClusterEvent3DFeatures1::ClusterEvent3DFeatures1(const ClusterEvent3DFeatures1 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
}


/**************************************************************************/
ClusterEvent3DFeatures1::ClusterEvent3DFeatures1(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();

        // check type and fill fields
        if (((word0>>26)==17)&&((word1>>26)==10))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            // word1
            numAE=word1&0x00ffffff;

            type="3D-CLE-F1";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent3DFeatures1 &ClusterEvent3DFeatures1::operator=(const ClusterEvent3DFeatures1 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;

    return *this;
}


/**************************************************************************/
bool ClusterEvent3DFeatures1::operator==(const ClusterEvent3DFeatures1 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (disparity==event.disparity)&&(numAE==event.numAE)&&
            (xCog==event.xCog)&&(yCog==event.yCog));
}


/**************************************************************************/
Bottle ClusterEvent3DFeatures1::encode() const
{
    int word0=(17<<26)|((disparity&0xff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(10<<26)|(numAE&0x00ffffff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
Property ClusterEvent3DFeatures1::getContent() const
{
    Property prop=ClusterEvent3D::getContent();
    prop.put("numAE",numAE);

    return prop;
}


/**************************************************************************/
ClusterEvent3DFeatures2::ClusterEvent3DFeatures2() : ClusterEvent3DFeatures1()
{
    type="3D-CLE-F2";
    shapeType=0;
    xSize=0;
    ySize=0;
}


/**************************************************************************/
ClusterEvent3DFeatures2::ClusterEvent3DFeatures2(const ClusterEvent3DFeatures2 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
    shapeType=event.shapeType;
    xSize=event.xSize;
    ySize=event.ySize;
}


/**************************************************************************/
ClusterEvent3DFeatures2::ClusterEvent3DFeatures2(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();
        int word2=packets.get(pos+2).asInt();

        // check type and fill fields
        if (((word0>>26)==17)&&((word1>>26)==11)&&((word2>>26)==12))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            // word1
            numAE=word1&0x00ffffff;

            // word2
            xSize=word2&0xff;

            word2>>=8;
            ySize=word2&0xff;

            word2>>=8;
            shapeType=word2&0xff;

            type="3D-CLE-F2";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent3DFeatures2 &ClusterEvent3DFeatures2::operator=(const ClusterEvent3DFeatures2 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
    shapeType=event.shapeType;
    xSize=event.xSize;
    ySize=event.ySize;

    return *this;
}


/**************************************************************************/
bool ClusterEvent3DFeatures2::operator==(const ClusterEvent3DFeatures2 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (disparity==event.disparity)&&(numAE==event.numAE)&&(xCog==event.xCog)&&
            (yCog==event.yCog)&&(shapeType==event.shapeType)&&(xSize==event.xSize)&&
            (ySize==event.ySize));
}


/**************************************************************************/
Bottle ClusterEvent3DFeatures2::encode() const
{
    int word0=(17<<26)|((disparity&0xff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(11<<26)|(numAE&0x00ffffff);
    int word2=(12<<26)|((shapeType&0xff)<<16)|((ySize&0xff)<<8)|(xSize&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    return ret;
}


/**************************************************************************/
Property ClusterEvent3DFeatures2::getContent() const
{
    Property prop=ClusterEvent3DFeatures1::getContent();
    prop.put("shapeType",shapeType);
    prop.put("xSize",xSize);
    prop.put("ySize",ySize);

    return prop;
}


/**************************************************************************/
ClusterEvent3DFeatures3::ClusterEvent3DFeatures3() : ClusterEvent3DFeatures2()
{
    type="3D-CLE-F3";
    shapeProb=0;
    xVel=0;
    yVel=0;
}


/**************************************************************************/
ClusterEvent3DFeatures3::ClusterEvent3DFeatures3(const ClusterEvent3DFeatures3 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
    shapeType=event.shapeType;
    xSize=event.xSize;
    ySize=event.ySize;
    shapeProb=event.shapeProb;
    xVel=event.xVel;
    yVel=event.yVel;
}


/**************************************************************************/
ClusterEvent3DFeatures3::ClusterEvent3DFeatures3(const Bottle &packets, const int pos)
{
    valid=false;

    // check length
    if ((pos+getLength()-1)<packets.size())
    {
        int word0=packets.get(pos).asInt();
        int word1=packets.get(pos+1).asInt();
        int word2=packets.get(pos+2).asInt();
        int word3=packets.get(pos+3).asInt();

        // check type and fill fields
        if (((word0>>26)==17)&&((word1>>26)==11)&&
            ((word2>>26)==13)&&((word3>>26)==14))
        {
            // word0
            channel=word0&0x01;

            word0>>=1;
            xCog=word0&0x7f;

            word0>>=7;
            yCog=word0&0xff;

            word0>>=8;
            disparity=word0&0xff;

            // word1
            numAE=word1&0x00fffff;

            // word2
            xSize=word2&0xff;

            word2>>=8;
            ySize=word2&0xff;

            word2>>=8;
            shapeType=word2&0xff;

            // word3
            xVel=word3&0xff;

            word3>>=8;
            yVel=word3&0xff;

            word3>>=8;
            shapeProb=word3&0xff;

            type="3D-CLE-F3";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent3DFeatures3 &ClusterEvent3DFeatures3::operator=(const ClusterEvent3DFeatures3 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    disparity=event.disparity;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
    shapeType=event.shapeType;
    xSize=event.xSize;
    ySize=event.ySize;
    shapeProb=event.shapeProb;
    xVel=event.xVel;
    yVel=event.yVel;

    return *this;
}


/**************************************************************************/
bool ClusterEvent3DFeatures3::operator==(const ClusterEvent3DFeatures3 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (disparity==event.disparity)&&(numAE==event.numAE)&&(xCog==event.xCog)&&
            (yCog==event.yCog)&&(shapeType==event.shapeType)&&(xSize==event.xSize)&&
            (ySize==event.ySize)&&(shapeProb=event.shapeProb)&&(xVel==event.xVel)&&
            (yVel==event.yVel));
}


/**************************************************************************/
Bottle ClusterEvent3DFeatures3::encode() const
{
    int word0=(17<<26)|((disparity&0xff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(11<<26)|(numAE&0x00ffffff);
    int word2=(13<<26)|((shapeType&0xff)<<16)|((ySize&0xff)<<8)|(xSize&0xff);
    int word3=(14<<26)|((shapeProb&0xff)<<16)|((yVel&0xff)<<8)|(xVel&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    ret.addInt(word3);
    return ret;
}


/**************************************************************************/
Property ClusterEvent3DFeatures3::getContent() const
{
    Property prop=ClusterEvent3DFeatures2::getContent();
    prop.put("shapeProb",shapeProb);
    prop.put("xVel",xVel);
    prop.put("yVel",yVel);

    return prop;
}

/**************************************************************************/
HoughEvent::HoughEvent()
{
    valid   = true;
    type    = "HGE";
    channel = 0;
    radius  = 0;
    xCoc    = 0;
    yCoc    = 0;
}


/**************************************************************************/
HoughEvent::HoughEvent(const HoughEvent &event)
{
    valid   = event.valid;
    type    = event.type;
    channel = event.channel;
    radius  = event.radius;
    xCoc    = event.xCoc;
    yCoc    = event.yCoc;
}


/**************************************************************************/
HoughEvent::HoughEvent(const Bottle &packets, const int pos)
{
    valid = false;

    // check length
    if ((pos + getLength()-1) < packets.size())
    {
        int word0=packets.get(pos).asInt();

        // check type and fill fields
        if ((word0>>26)==24)
        {
            channel = word0&0x01;

            word0 >>= 1;
            xCoc = word0&0x7f;

            word0 >>= 7;
            yCoc = word0&0xff;

            word0 >>= 8;
            radius = word0&0x03ff;

            type  = "HGE";
            valid = true;
        }
    }
}


/**************************************************************************/
HoughEvent &HoughEvent::operator=(const HoughEvent &event)
{
    valid   = event.valid;
    type    = event.type;
    channel = event.channel;
    radius  = event.radius;
    xCoc    = event.xCoc;
    yCoc    = event.yCoc;

    return *this;
}


/**************************************************************************/
bool HoughEvent::operator==(const HoughEvent &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (radius==event.radius)&&(xCoc==event.xCoc)&&(yCoc==event.yCoc));
}


/**************************************************************************/
Bottle HoughEvent::encode() const
{
    int word0=(24<<26)|((radius&0x03ff)<<16)|((yCoc&0xff)<<8)|((xCoc&0x7f)<<1)|(channel&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property HoughEvent::getContent() const
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("channel",channel);
    prop.put("radius",radius);
    prop.put("xCoc",xCoc);
    prop.put("yCoc",yCoc);

    return prop;
}


