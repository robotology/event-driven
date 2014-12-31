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
    vEvent() : type("TS"), stamp(0) { }
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

    virtual yarp::os::Bottle   encode() const;
    virtual vEvent * decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;

    template<class T> T* getAs() {
        return dynamic_cast<T*>(this);
    }
    
};


/**************************************************************************/
class AddressEvent : public vEvent
{
protected:
    int channel;
    int polarity;
    int x;
    int y;

    virtual int nBytesCoded() const         { return 1;                 }

public:
    AddressEvent();
    AddressEvent(const AddressEvent &event);
    AddressEvent(const vEvent &event);

    vEvent &operator=(const vEvent &event);
    bool operator==(const AddressEvent &event);

    int getChannel() const                  { return channel;           }
    int getPolarity() const                 { return polarity;          }
    int getX() const                        { return x;                 }
    int getY() const                        { return y;                 }

    void setChannel(const int channel)      { this->channel=channel;    }
    void setPolarity(const int polarity)    { this->polarity=polarity;  }
    void setX(const int x)                  { this->x=x;                }
    void setY(const int y)                  { this->y=y;                }


    bool operator==(const vEvent &event) {
        return operator==(dynamic_cast<const AddressEvent&>(event)); }
    yarp::os::Bottle   encode() const ;
    yarp::os::Property getContent() const;
    virtual vEvent * decode(const yarp::os::Bottle &packet, int &pos);
};

/**************************************************************************/
/* AddressEventFlow --- to be done                                        */
/**************************************************************************/
class AddressEventFeatures : public AddressEvent
{
protected:
    int orientation;
    int xFlow;
    int yFlow;

public:
    AddressEventFeatures();
    AddressEventFeatures(const AddressEventFeatures &event);
    AddressEventFeatures(const yarp::os::Bottle &packets, const int pos=0);

    AddressEventFeatures &operator=(const AddressEventFeatures &event);
    bool operator==(const AddressEventFeatures &event);

    int getOrientation() const                 { return orientation;            }
    int getXFlow() const                       { return xFlow;                  }
    int getYFlow() const                       { return yFlow;                  }

    void setOrientation(const int orientation) { this->orientation=orientation; }
    void setXFlow(const int xFlow)             { this->xFlow=xFlow;             }
    void setYFlow(const int yFlow)             { this->yFlow=yFlow;             }

    int getLength() const { return 2; }
    bool operator==(const vEvent &event) { return operator==(dynamic_cast<const AddressEventFeatures&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent : public vEvent
{
protected:
    int id;
    int channel;
    int xCog;
    int yCog;

public:
    ClusterEvent();
    ClusterEvent(const ClusterEvent &event);
    ClusterEvent(const vEvent &event);

//    vEvent &operator=(const vEvent &event); // to be done
    
    ClusterEvent &operator=(const ClusterEvent &event);
    bool operator==(const ClusterEvent &event);

    int getChannel() const             { return channel;        }
    int getId()      const             { return id;             }
    int getXCog()    const             { return xCog;           }
    int getYCog()    const             { return yCog;           }

    void setChannel(const int channel) { this->channel = channel; }
    void setId(const int id)           { this->id = id;     }
    void setXCog(const int xCog)       { this->xCog = xCog;       }
    void setYCog(const int yCog)       { this->yCog = yCog;       }

    bool operator==(const vEvent &event) { return operator==(dynamic_cast<const ClusterEvent&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
    virtual vEvent * decode(const yarp::os::Bottle &packet, int &pos);
};

/**************************************************************************/
class ClusterEventGauss : public ClusterEvent
{
protected:
    int numAE;
    int xSigma2;
    int ySigma2;
    int xySigma;
    int xVel;
    int yVel;
    virtual int nBytesCoded() const         { return 4;                 }

public:
    ClusterEventGauss();
    ClusterEventGauss(const ClusterEventGauss &event);
    
    ClusterEventGauss &operator=(const ClusterEventGauss &event);
    bool operator==(const ClusterEventGauss &event);

    int getActivity()   const               { return numAE;             }
    int getXSigma2()    const               { return xSigma2;           }
    int getYSigma2()    const               { return ySigma2;           }
    int getXYSigma()    const               { return xySigma;           }
    int getXVel()       const               { return xVel;              }
    int getYVel()       const               { return yVel;              }

    void setActivity(const int numAE)       { this->numAE=numAE;        }
    void setXSigma2(const int xSigma2)      { this->xSigma2=xSigma2;    }
    void setYSigma2(const int ySigma2)      { this->ySigma2=ySigma2;    }
    void setXYSigma(const int xySigma)      { this->xySigma=xySigma;    }
    void setXVel(const int xVel)            { this->xVel=xVel;          }
    void setYVel(const int yVel)            { this->yVel=yVel;          }

    int getLength() const { return 4; }
    bool operator==(const vEvent &event) { return operator==(dynamic_cast<const ClusterEventGauss&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
    virtual vEvent * decode(const yarp::os::Bottle &packet, int &pos);
    
};

/**************************************************************************/
class AddressEventCluster : public AddressEvent
{
protected:
    int clId;
    
    virtual int nBytesCoded() const         { return 2;                 }
    
public:
    AddressEventCluster();
    AddressEventCluster(const AddressEventCluster &event);
    AddressEventCluster(const AddressEvent &event);
    virtual ~AddressEventCluster();
    
    AddressEventCluster &operator=(const AddressEventCluster &event);
    AddressEventCluster &operator=(const AddressEvent &event);
    bool operator==(const AddressEventCluster &event);
    
    // manage cluster ID
    int getId() const                               { return clId;                  }
    void setId(const int clId)                      { this->clId=clId;              }
    void assignCluster(const ClusterEvent &event)   { this->clId=event.getId();     }
    
    int getLength() const { return 2; }
    bool operator==(const vEvent &event) { return operator==(dynamic_cast<const AddressEventCluster&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
    virtual vEvent * decode(const yarp::os::Bottle &packet, int &pos);
};
    
    
}

#endif


