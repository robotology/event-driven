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


}

}


/**************************************************************************/
bool eEvent::decode(const Bottle &packets, eEventQueue &events)
{
    int pos=0;
    while (pos<packets.size())
    {
        eEvent *pEvent=NULL;
        if (pEvent=factoryTimeStamp(packets,pos))                   { }
        else if (pEvent=factoryAddressEvent(packets,pos))           { }
        else if (pEvent=factoryAddressEventFeatures(packets,pos))   { }
        else if (pEvent=factoryAddressEvent3D(packets,pos))         { }
        else if (pEvent=factoryAddressEvent3DFeatures(packets,pos)) { }
        else if (pEvent=factoryClusterEvent(packets,pos))           { }
        else if (pEvent=factoryClusterEventFeatures1(packets,pos))  { }
        else if (pEvent=factoryClusterEventFeatures2(packets,pos))  { }
        else if (pEvent=factoryClusterEventFeatures3(packets,pos))  { }

        if (pEvent==NULL)
            return false;

        events.push_back(pEvent);
        pos+=pEvent->getLength();
    }

    return true;
}


/**************************************************************************/
eEventQueue::~eEventQueue()
{
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
Bottle TimeStamp::encode()
{
    int word0=(32<<26)|(stamp&0x00ffffff);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property TimeStamp::getContent()
{
    Property prop;
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
            int shrw=word0;
            polarity=shrw&0x01;

            shrw>>=1;
            x=shrw&0x7f;

            shrw>>=7;
            y=shrw&0xff;

            shrw>>=8;
            channel=shrw&0x01;

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
Bottle AddressEvent::encode()
{
    int word0=(0<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property AddressEvent::getContent()
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
            int shrw=word0;
            polarity=shrw&0x01;

            shrw>>=1;
            x=shrw&0x7f;

            shrw>>=7;
            y=shrw&0xff;

            shrw>>=8;
            channel=shrw&0x01;

            // word1
            shrw=word1;
            orientation=shrw&0xff;

            shrw>>=8;
            xFlow=shrw&0xff;

            shrw>>=8;
            yFlow=shrw&0xff;

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
Bottle AddressEventFeatures::encode()
{
    int word0=(2<<26)|((channel&0x01)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(3<<26)|((yFlow&0xff)<<16)|((xFlow&0xff)<<8)|(orientation&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
Property AddressEventFeatures::getContent()
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
            int shrw=word0;
            polarity=shrw&0x01;

            shrw>>=1;
            x=shrw&0x7f;

            shrw>>=7;
            y=shrw&0xff;

            shrw>>=8;
            disparity=shrw&0xff;

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
Bottle AddressEvent3D::encode()
{
    int word0=(1<<26)|((disparity&0xff)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property AddressEvent3D::getContent()
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
            int shrw=word0;
            polarity=shrw&0x01;

            shrw>>=1;
            x=shrw&0x7f;

            shrw>>=7;
            y=shrw&0xff;

            shrw>>=8;
            disparity=shrw&0xff;

            // word1
            shrw=word1;
            orientation=shrw&0xff;

            shrw>>=8;
            xFlow=shrw&0xff;

            shrw>>=8;
            yFlow=shrw&0xff;

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
Bottle AddressEvent3DFeatures::encode()
{
    int word0=(4<<26)|((disparity&0xff)<<16)|((y&0xff)<<8)|((x&0x7f)<<1)|(polarity&0x01);
    int word1=(3<<26)|((yFlow&0xff)<<16)|((xFlow&0xff)<<8)|(orientation&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    return ret;
}


/**************************************************************************/
Property AddressEvent3DFeatures::getContent()
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
    valid=true;
    type="CLE";
    channel=0;
    numAE=0;
    xCog=0;
    yCog=0;
}


/**************************************************************************/
ClusterEvent::ClusterEvent(const ClusterEvent &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;
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
            int shrw=word0;
            channel=shrw&0x01;

            shrw>>=1;
            xCog=shrw&0x7f;

            shrw>>=7;
            yCog=shrw&0xff;

            shrw>>=8;
            numAE=shrw&0xff;

            type="CLE";
            valid=true;
        }
    }
}


/**************************************************************************/
ClusterEvent &ClusterEvent::operator=(const ClusterEvent &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    numAE=event.numAE;
    xCog=event.xCog;
    yCog=event.yCog;

    return *this;
}


/**************************************************************************/
bool ClusterEvent::operator==(const ClusterEvent &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (numAE==event.numAE)&&(xCog==event.xCog)&&(yCog==event.yCog));
}


/**************************************************************************/
Bottle ClusterEvent::encode()
{
    int word0=(8<<26)|((numAE&0xff)<<16)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);

    Bottle ret;
    ret.addInt(word0);
    return ret;
}


/**************************************************************************/
Property ClusterEvent::getContent()
{
    Property prop;
    prop.put("type",type.c_str());
    prop.put("channel",channel);
    prop.put("numAE",numAE);
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
            int shrw=word0;
            channel=shrw&0x01;

            shrw>>=1;
            xCog=shrw&0x7f;

            shrw>>=7;
            yCog=shrw&0xff;

            // word1
            shrw=word1;
            numAE=shrw&0x00ffffff;

            type="CLE-F1";
            valid=true;
        }
    }
}


/**************************************************************************/
Bottle ClusterEventFeatures1::encode()
{
    int word0=(9<<26)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(10<<26)|(numAE&0x00ffffff);

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
}


/**************************************************************************/
ClusterEventFeatures2::ClusterEventFeatures2(const ClusterEventFeatures2 &event)
{
    valid=event.valid;
    type=event.type;
    channel=event.channel;
    numAE=event.numAE;
    shapeType=event.shapeType;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;
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
            int shrw=word0;
            channel=shrw&0x01;

            shrw>>=1;
            xCog=shrw&0x7f;

            shrw>>=7;
            yCog=shrw&0xff;

            // word1
            shrw=word1;
            numAE=shrw&0x00ffffff;

            // word2
            shrw=word2;
            xSize=shrw&0xff;

            shrw>>=8;
            ySize=shrw&0xff;

            shrw>>=8;
            shapeType=shrw&0xff;

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
    numAE=event.numAE;
    shapeType=event.shapeType;
    xCog=event.xCog;
    yCog=event.yCog;
    xSize=event.xSize;
    ySize=event.ySize;

    return *this;
}


/**************************************************************************/
bool ClusterEventFeatures2::operator==(const ClusterEventFeatures2 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (numAE==event.numAE)&&(shapeType==event.shapeType)&&(xCog==event.xCog)&&
            (yCog==event.yCog)&&(xSize==event.xSize)&&(ySize==event.ySize));
}


/**************************************************************************/
Bottle ClusterEventFeatures2::encode()
{
    int word0=(9<<26)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
    int word1=(11<<26)|(numAE&0x00ffffff);
    int word2=(12<<26)|((shapeType&0xff)<<16)|((ySize&0xff)<<8)|(xSize&0xff);

    Bottle ret;
    ret.addInt(word0);
    ret.addInt(word1);
    ret.addInt(word2);
    return ret;
}


/**************************************************************************/
Property ClusterEventFeatures2::getContent()
{
    Property prop=ClusterEventFeatures1::getContent();
    prop.put("shapeType",shapeType);
    prop.put("xSize",xSize);
    prop.put("ySize",ySize);

    return prop;
}














/**************************************************************************/
ClusterEventFeatures3::ClusterEventFeatures3() : ClusterEventFeatures2()
{
    type="CLE-F3";
    shapeProb=0;
    xVel=0;
    yVel=0;
}


/**************************************************************************/
ClusterEventFeatures3::ClusterEventFeatures3(const ClusterEventFeatures3 &event)
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
            int shrw=word0;
            channel=shrw&0x01;

            shrw>>=1;
            xCog=shrw&0x7f;

            shrw>>=7;
            yCog=shrw&0xff;

            // word1
            shrw=word1;
            numAE=shrw&0x00ffffff;

            // word2
            shrw=word2;
            xSize=shrw&0xff;

            shrw>>=8;
            ySize=shrw&0xff;

            shrw>>=8;
            shapeType=shrw&0xff;

            // word3
            shrw=word3;
            xVel=shrw&0xff;

            shrw>>=8;
            yVel=shrw&0xff;

            shrw>>=8;
            shapeProb=shrw&0xff;

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

    return *this;
}


/**************************************************************************/
bool ClusterEventFeatures3::operator==(const ClusterEventFeatures3 &event)
{
    return ((valid==event.valid)&&(type==event.type)&&(channel==event.channel)&&
            (numAE==event.numAE)&&(shapeType==event.shapeType)&&(shapeProb==event.shapeProb)&&
            (xCog==event.xCog)&&(yCog==event.yCog)&&(xSize==event.xSize)&&(ySize==event.ySize)&&
            (xVel==event.xVel)&&(yVel==event.yVel));
}


/**************************************************************************/
Bottle ClusterEventFeatures3::encode()
{
    int word0=(9<<26)|((yCog&0xff)<<8)|((xCog&0x7f)<<1)|(channel&0x01);
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
Property ClusterEventFeatures3::getContent()
{
    Property prop=ClusterEventFeatures2::getContent();
    prop.put("shapeProb",shapeProb);
    prop.put("xVel",xVel);
    prop.put("yVel",yVel);

    return prop;
}



