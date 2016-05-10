/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

/// \defgroup emorphLib emorphLib
/// \defgroup vCodec vCodec
/// \ingroup emorphLib
/// \brief class hierarchy for different types of emorph::vEvent
///
/// Event Coding
/// ============
///
/// Events are serialised and coded in a standardised format for sending and
/// receiving between modules. In addition a packet containing multiple
/// different types of events is segmented by event-type such that a search can
/// quickly retrieve events of only a specific type. The packet is formed as
/// such:
///
///     EVENTTYPE-1-TAG ( serialised and concatinated events of type 1) EVENTTYPE-2-TAG ( serialised and concatinated events of type 2) ...
///
/// Each event class defines the TAG used to identify itself and also the
/// method with which the event data is serialised. Managing the serialisation
/// and de-serialisation of the event data is then simply a case of using the
/// event class to write/read its TAG and then call its encode/decode functions
/// on the serialised data. The emorph::vBottle class handles the coding of
/// packets in the event-driven project.
///
/// Events are defined in a class hierarchy, with each child class calling its
/// parent encode/decode function before its own. Adding a new event therefore
/// only requires defining the serialisation method for any new data that the
/// event-class contains (e.g. the Flow event only defines how the velocities
/// are encoded and calls its parent class, the AdressEvent, to encode other
/// information, such as position and timestamp).
///
/// ![The Event-Type Class Hierarchy](@ref classemorph_1_1vEvent.png)
///
/// Event Coding Definitions
/// ------------------------
///
/// The **vEvent** uses 4 bytes to encode a timestamp (_T_)
///
///     [10000000 TTTTTTT TTTTTTTT TTTTTTTT]
///
/// An **AddressEvent** uses 4 bytes to encode position (_X_, _Y_), polarity
/// (_P_) and channel (_C_). Importantly as AddressEvent is of type vEvent the
/// timestamp information of this event is always encoded as well.
///
///     [00000000 00000000 CYYYYYYY XXXXXXXP]
///
/// A **FlowEvent** uses 8 bytes to encode velocity (ẋ, ẏ), each 4 bytes
/// represent a _float_. Similarly as FlowEvent is of time AddressEvent the
/// FlowEvent also encodes all the position and timestamp information above.
///
///     [ẋẋẋẋẋẋẋẋ ẋẋẋẋẋẋẋẋ ẏẏẏẏẏẏẏẏ ẏẏẏẏẏẏẏẏ]
///
/// An **AddressEventClustered** is labelled as belonging to a group ID (_I_)
/// using a 4 byte _int_.
///
///     [IIIIIIIII IIIIIIIII IIIIIIIII IIIIIIIII]
///
/// A **ClusterEvent** is encodes the central position (_X_, _Y_) of a labelled
/// cluster (_I_) and also has a polarity (_P_) and channel (_C_)
///
///     [IIIIIIII IIIIIIII CYYYYYYY XXXXXXXP]
///
/// _NOTE: Cluster Event and AddressEventClustered could be consolidated in
/// some way_
///
/// A **ClusterEventGauss** extends a cluster event with a 2 dimensional
/// Gaussian distribution parameterised by (_sx_, _sy_, _sxy_) a count of events
/// falling in this distribution (_n_) and its velocity (ẋ, ẏ) using a
/// total of 12 bytes.
///
///     [sxysxysxysxysxysxysxysxy sxysxysxysxysxysxysxysxy nnnnnnnn nnnnnnnn
///      sxsxsxsxsxsxsxsx sxsxsxsxsxsxsxsx sysysysysysysysy sysysysysysysysy
///      ẋẋẋẋẋẋẋẋ ẋẋẋẋẋẋẋẋ ẏẏẏẏẏẏẏẏ ẏẏẏẏẏẏẏẏ]
///
/// A **CollisionEvent** uses 1 byte to encode collision position (_X_, _Y_),
/// the channel (_C_) and the two cluster IDs that collided (_I1_, _I2_)
///
///     [I1I1I1I1I1I1I1I1 I2I2I2I2I2I2I2I2 CXXXXXXX 0YYYYYYY]
///
/// Coding in YARP
/// --------------
///
/// The emorph::vBottle class wraps the encoding and decoding operations into a
/// yarp::os::Bottle such that an example vBottle will appear as:
///
///     AE (-2140812352 15133 -2140811609 13118) FLOW (-2140812301 13865 -1056003417 -1055801578)
///
/// _NOTE: The actual data sent by YARP for a bottle includes signifiers for
/// data type and data length, adding extra data to the bottle as above._
///
///     256 4 4 2 'A' 'E' 257 4 -2140812352 15133 -2140811609 13118 4 4 'F' 'L' 'O' 'W' 257 4 -2140812301 13865 -1056003417 -1055801578
///
/// Coding in ROS
/// -------------
///
/// Here explain how the coding would/will be performed for ROS. How does the
/// YARP/ROS interface consolidate the above two formats.
///

#ifndef __EMORPH_VCODEC_H__
#define __EMORPH_VCODEC_H__

#include <iCub/emorph/vtsHelper.h>
#include <string>

#define encoderr std::cerr << "Warning: code bits not sufficient" << std::endl;

namespace emorph {

class vEvent;

vEvent * createEvent(const std::string type);
/**************************************************************************/
class vEvent
{
private:
    const static int localWordsCoded = 1;


protected:

    const static unsigned int max_stamp = 16777215; //2^24
    int stamp;

public:
    //!blank constructor required at base level
    vEvent() : stamp(0), refcount(0) { }
    //!copy constructor
    vEvent(const vEvent &event);
    virtual ~vEvent() {}

    int refcount;
    void referto() { refcount++; }
    //bool destroy() { return !(--refcount > 0); }
    void destroy() { if(!(--refcount > 0)) delete this; }

    virtual std::string getType() const { return "TS";}


    void setStamp(const long int stamp)   { this->stamp=(int)(stamp%max_stamp);}
    int getStamp() const            { return stamp;         }

    virtual int getChannel() const  { return -1;            }
    virtual int getPolarity() const { return -1;            }

    virtual vEvent &operator=(const vEvent &event);
    virtual bool operator==(const vEvent &event);
    virtual bool operator<(const vEvent &event) const
                 {return this->stamp < event.stamp; }
    virtual bool operator>(const vEvent &event) const
                 {return this->stamp > event.stamp; }
    virtual vEvent* clone();

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual int nBytesCoded() const {return localWordsCoded * sizeof(int);}

    template<class T> T* getAs() {
        return dynamic_cast<T*>(this);
    }

    template<class T> T* getUnsafe() {
        return (T*)(this);
    }

};


/**************************************************************************/
class AddressEvent : public vEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    //add new member variables here
    char channel;
    unsigned char polarity;
    unsigned char x;
    unsigned char y;

public:

    //these are new the member get functions
    virtual std::string getType() const { return "AE";}
    int getChannel() const                      { return channel;       }
    int getPolarity() const                     { return polarity;      }
    unsigned char getX() const                  { return x;             }
    unsigned char getY() const                  { return y;             }

    void setChannel(const unsigned char channel)    {
        if(channel>1) encoderr; this->channel=channel; }
    void setPolarity(const unsigned char polarity)  {
        if(polarity > 1) encoderr; this->polarity=polarity; }
    void setX(const unsigned char x)                {
        if(x > 127) encoderr; this->x=x; }
    void setY(const unsigned char y)                {
        if(y > 127) encoderr; this->y=y; }

    //these functions need to be defined correctly for inheritance
    AddressEvent() : vEvent(), channel(0), polarity(0), x(0), y(0) {}
    AddressEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const AddressEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const AddressEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + vEvent::nBytesCoded();                 }


};

/**************************************************************************/
class AddressEventClustered : public AddressEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    int clID;

public:

    virtual std::string getType() const { return "AE-C";}
    int getID() const                       { return clID;              }

    void setID(const int clID)              { this->clID = clID;        }

    AddressEventClustered() : AddressEvent(), clID(0) {}
    AddressEventClustered(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const AddressEventClustered &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const AddressEventClustered&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + AddressEvent::nBytesCoded(); }

};

/**************************************************************************/
class CollisionEvent : public vEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    unsigned char x; //7
    unsigned char y; //7
    char channel; //1
    unsigned char clid1; //8
    unsigned char clid2; //8


public:

    //accessors
    virtual std::string getType() const { return "COL";}
    void setX(unsigned char x) {if(x>127)encoderr; this->x = x;}
    void setY(unsigned char y) {if(y>127)encoderr; this->y = y;}
    void setChannel(unsigned char channel) {
        if (channel != 0 || channel != 1) encoderr; this->channel = channel;}
    void setClid1(unsigned char clid1) {
        if(clid1>255)encoderr; this->clid1 = clid1;}
    void setClid2(unsigned char clid2) {
        if(clid2>255)encoderr; this->clid2 = clid2;}

    unsigned char getX()        {return x;}
    unsigned char getY()        {return y;}
    virtual int getChannel()    {return channel;}
    unsigned char getClid1()    {return clid1;}
    unsigned char getClid2()    {return clid2;}


    CollisionEvent() : vEvent(), x(0), y(0), channel(0), clid1(0), clid2(0) {}
    CollisionEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const CollisionEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const CollisionEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + vEvent::nBytesCoded(); }

};

/**************************************************************************/
class ClusterEvent : public vEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    //add new member variables here
    short int id;
    unsigned char channel;
    unsigned char xCog;
    unsigned char yCog;
    unsigned char polarity;

public:

    //these are new the member get functions
    virtual std::string getType() const { return "CLE";}
    int getChannel() const             { return channel;        }
    int getID()      const             { return id;             }
    int getXCog()    const             { return xCog;           }
    int getYCog()    const             { return yCog;           }
    int getPolarity()const             { return polarity;           }

    void setChannel(const int channel)   { if(channel>1) encoderr;
        this->channel = channel;  }
    void setID(const int id)             { if(id>1023) encoderr;
        this->id = id;            }
    void setXCog(const int xCog)         { if(xCog>127) encoderr;
        this->xCog = xCog;        }
    void setYCog(const int yCog)         { if(yCog>127) encoderr;
        this->yCog = yCog;        }
    void setPolarity(const int polarity) { if(polarity > 1) encoderr;
        this->polarity = polarity;}


    //these functions need to be defined correctly for inheritance
    ClusterEvent() :
        vEvent(), id(0), channel(0), xCog(0), yCog(0), polarity(1) {}
    ClusterEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const ClusterEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const ClusterEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + vEvent::nBytesCoded(); }

};


/**************************************************************************/
class ClusterEventGauss : public ClusterEvent
{
private:
    const static int localWordsCoded = 3;

protected:

    //add new member variables here
    unsigned short int numAE;
    short int xySigma;
    unsigned short int xSigma2;
    unsigned short int ySigma2;
    short int xVel;
    short int yVel;

public:

    //these are new the member get functions
    virtual std::string getType() const { return "CLEG";}
    int getNumAE()      const               { return numAE;            }
    int getXSigma2()    const               { return xSigma2;          }
    int getYSigma2()    const               { return ySigma2;          }
    int getXYSigma()    const               { return xySigma;          }
    int getXVel()       const               { return xVel;             }
    int getYVel()       const               { return yVel;             }

    void setNumAE(const int numAE)          { this->numAE=numAE;       }
    void setXSigma2(const int xSigma2)      { this->xSigma2=xSigma2;   }
    void setYSigma2(const int ySigma2)      { this->ySigma2=ySigma2;   }
    void setXYSigma(const int xySigma)      { this->xySigma=xySigma;   }
    void setXVel(const int xVel)            { this->xVel=xVel;   }
    void setYVel(const int yVel)            { this->yVel=yVel;   }

    //these functions need to be defined correctly for inheritance
    ClusterEventGauss() : ClusterEvent(), numAE(0), xSigma2(0), ySigma2(0),
        xVel(0), yVel(0) {}
    ClusterEventGauss(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const ClusterEventGauss &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const ClusterEventGauss&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + ClusterEvent::nBytesCoded(); }
};


/**************************************************************************/
class FlowEvent : public AddressEvent
{
private:
    const static int localWordsCoded = 3;

protected:

    //add new member variables here
    float vx;
    float vy;
    int death;

public:

    //these are new the member get functions
    virtual std::string getType() const { return "FLOW";}
    float getVx() const                     { return vx;                }
    float getVy() const                     { return vy;                }
    int getDeath() const { return death; }

    void setVx(float vx)                    { this->vx=vx;              }
    void setVy(float vy)                    { this->vy=vy;              }
    void setDeath();

    //these functions need to be defined correctly for inheritance
    FlowEvent() : AddressEvent(), vx(0), vy(0), death(0) {}
    FlowEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const FlowEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const FlowEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + vEvent::nBytesCoded(); }


};

/**************************************************************************/
class InterestEvent : public AddressEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    int intID;

public:

    virtual std::string getType() const { return "AE-INT";}
    int getID() const                       { return intID;              }

    void setID(const int intID)              { this->intID = intID;        }

    InterestEvent() : AddressEvent(), intID(0) {}
    InterestEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const InterestEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const InterestEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + AddressEvent::nBytesCoded(); }


};

/**************************************************************************/
class NeuronIDEvent : public vEvent
{
private:
    const static int localWordsCoded = 1;

protected:

    int neurID;

public:

    virtual std::string getType() const { return "N_ID";}
    int getID() const                       { return neurID;              }

    void setID(const int neurID)              { this->neurID = neurID;        }

    NeuronIDEvent() : vEvent(), neurID(0) {}
    NeuronIDEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const NeuronIDEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const NeuronIDEvent&>(event)); }
    virtual void encode(yarp::os::Bottle &b) const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

    //this is the total number of bytes used to code this event
    virtual int nBytesCoded() const         { return localWordsCoded *
                sizeof(int) + vEvent::nBytesCoded(); }

};

}

#endif


