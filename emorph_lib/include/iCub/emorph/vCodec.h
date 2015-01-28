// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini, modified by Arren Glover(10/14)
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

#ifndef __EMORPH_ECODEC_H__
#define __EMORPH_ECODEC_H__

#include <yarp/os/all.h>
#include <string>
#include <deque>

namespace emorph {

//forward declaration
class vEvent;

vEvent * createEvent(const std::string type);

/**************************************************************************/
class vQueue : public std::deque<vEvent*>
{
private:
    // copy-constructor and overaloaded "=" operator are made
    // private on purpose to avoid the problem of dangling
    // pointers since the destructor frees up the memory
    // allocated for events
    vQueue(const vQueue&);
    vQueue &operator=(const vQueue&);
    static bool temporalSort(const vEvent *e1, const vEvent *e2);

protected:
    bool owner;  

public:
    vQueue()                   { owner=true;        }
    vQueue(const bool _owner)  { owner=_owner;      }
    void setOwner(const bool owner) { this->owner=owner; }
    bool getOwner()                 { return owner;      }
    ~vQueue();

    void sort();


};


/**************************************************************************/
class vEvent
{
protected:

    bool valid; //this is just in here so i don't have to change other
                //derived classes just yet. delete it when i can
    std::string type;
    int stamp;

    virtual int nBytesCoded() const { return 1;             }

public:
    //!blank constructor required at base level
    vEvent() : type("TS"), stamp(0) { }
    //!copy constructor
    vEvent(const vEvent &event);

    std::string getType() const     { return type;          }


    void setStamp(const int stamp)  { this->stamp=stamp;    }
    int getStamp() const            { return stamp;         }

    virtual int getChannel() const  { return -1;            }

    virtual vEvent &operator=(const vEvent &event);
    virtual bool operator==(const vEvent &event);
    virtual bool operator<(const vEvent &event) const
                 {return this->stamp < event.stamp; }
    virtual bool operator>(const vEvent &event) const
                 {return this->stamp > event.stamp; }
    virtual vEvent* clone();

    virtual yarp::os::Bottle   encode() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;

    template<class T> T* getAs() {
        return dynamic_cast<T*>(this);
    }

};


/**************************************************************************/
class AddressEvent : public vEvent
{
protected:

    //add new member variables here
    int channel;
    int polarity;
    int x;
    int y;

    //this is the number of *extra* bytes (in addition to base class) to code
    virtual int nBytesCoded() const         { return 1;                 }

public:

    //these are new the member get functions
    int getChannel() const                  { return channel;           }
    int getPolarity() const                 { return polarity;          }
    int getX() const                        { return x;                 }
    int getY() const                        { return y;                 }

    void setChannel(const int channel)      { this->channel=channel;    }
    void setPolarity(const int polarity)    { this->polarity=polarity;  }
    void setX(const int x)                  { this->x=x;                }
    void setY(const int y)                  { this->y=y;                }

    //these functions need to be defined correctly for inheritance
    AddressEvent() : vEvent(), channel(0), polarity(0), x(0), y(0) {this->type = "AE";}
    AddressEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const AddressEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const AddressEvent&>(event)); }
    yarp::os::Bottle   encode() const ;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);




};

/**************************************************************************/
class AddressEventClustered : public AddressEvent
{
protected:

    int clID;

    virtual int nBytesCoded() const { return 1;}

public:

    int getID() const                       { return clID;              }

    void setID(const int clID)              { this->clID = clID;        }

    AddressEventClustered() : AddressEvent(), clID(0) {this->type = "AE-C";}
    AddressEventClustered(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const AddressEventClustered &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const AddressEventClustered&>(event)); }
    yarp::os::Bottle   encode() const ;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

};

/**************************************************************************/
class ClusterEvent : public vEvent
{
protected:

    //add new member variables here
    int id;
    int channel;
    int xCog;
    int yCog;
    int polarity;

    //this is the number of *extra* bytes (in addition to base class) to code
    virtual int nBytesCoded() const { return 1; }

public:

    //these are new the member get functions
    int getChannel() const             { return channel;        }
    int getID()      const             { return id;             }
    int getXCog()    const             { return xCog;           }
    int getYCog()    const             { return yCog;           }
    int getPolarity()const             { return polarity;           }

    void setChannel(const int channel) { this->channel = channel; }
    void setID(const int id)           { this->id = id;     }
    void setXCog(const int xCog)       { this->xCog = xCog;       }
    void setYCog(const int yCog)       { this->yCog = yCog;       }
    void setPolarity(const int polarity)       { this->polarity = polarity;       }


    //these functions need to be defined correctly for inheritance
    ClusterEvent() : vEvent(), id(0), channel(0), xCog(0), yCog(0), polarity(1)
                     {this->type = "CLE";}
    ClusterEvent(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const ClusterEvent &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const ClusterEvent&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);

};


/**************************************************************************/
class ClusterEventGauss : public ClusterEvent
{
protected:

    //add new member variables here
    int numAE;
    char xSigma2;
    char ySigma2;
    char xySigma;
    char xVel;
    char yVel;

    //this is the number of *extra* bytes (in addition to base class) to code
    virtual int nBytesCoded() const { return 3; }

public:

    //these are new the member get functions
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
        xVel(0), yVel(0) {this->type = "CLEG";}
    ClusterEventGauss(const vEvent &event);
    vEvent &operator=(const vEvent &event);
    virtual vEvent* clone();

    bool operator==(const ClusterEventGauss &event);
    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const ClusterEventGauss&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
};

}

#endif


