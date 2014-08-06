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

#ifndef __EMORPH_EVENTCODEC_H__
#define __EMORPH_EVENTCODEC_H__

#include <yarp/os/all.h>
#include <string>
#include <deque>

namespace emorph
{

namespace ecodec
{


// forward declaration
class eEventQueue;


/**************************************************************************/
class eEvent
{
protected:
    bool valid;
    std::string type;

public:
    eEvent() : valid(false), type("") { }
    bool isValid() const        { return valid; }
    std::string getType() /*const*/ ;

    virtual int getLength() const=0;
    virtual bool operator==(const eEvent &event)=0;
    virtual yarp::os::Bottle   encode() const=0;
    virtual yarp::os::Property getContent() const=0;

    static bool decode(const yarp::os::Bottle &packets, eEventQueue &events);
};


/**************************************************************************/
class eEventQueue : public std::deque<eEvent*>
{
private:
    // copy-constructor and overaloaded "=" operator are made
    // private on purpose to avoid the problem of dangling
    // pointers since the destructor frees up the memory
    // allocated for events
    eEventQueue(const eEventQueue&);
    eEventQueue &operator=(const eEventQueue&);

protected:
    bool owner;

public:
    eEventQueue()                   { owner=true;        }
    eEventQueue(const bool _owner)  { owner=_owner;      }
    void setOwner(const bool owner) { this->owner=owner; }
    bool getOwner()                 { return owner;      }
    ~eEventQueue();
};


/**************************************************************************/
class TimeStamp : public eEvent
{
protected:
    int stamp;

public:
    TimeStamp();
    TimeStamp(const TimeStamp &event);
    TimeStamp(const yarp::os::Bottle &packets, const int pos=0);

    TimeStamp &operator=(const TimeStamp &event);
    bool operator==(const TimeStamp &event);

    int getStamp() const           { return stamp;      }
    void setStamp(const int stamp) { this->stamp=stamp; }

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const TimeStamp&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class AddressEvent : public eEvent
{
protected:
    int channel;
    int polarity;
    int x;
    int y;

public:
    AddressEvent();
    AddressEvent(const AddressEvent &event);
    AddressEvent(const yarp::os::Bottle &packets, const int pos=0);

    AddressEvent &operator=(const AddressEvent &event);
    bool operator==(const AddressEvent &event);

    int getChannel() const               { return channel;          }
    int getPolarity() const              { return polarity;         }
    int getX() const                     { return x;                }
    int getY() const                     { return y;                }

    void setChannel(const int channel)   { this->channel=channel;   }
    void setPolarity(const int polarity) { this->polarity=polarity; }
    void setX(const int x)               { this->x=x;               }
    void setY(const int y)               { this->y=y;               }

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const AddressEvent&>(event)); }
    yarp::os::Bottle   encode() const ;
    yarp::os::Property getContent() const;
};


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
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const AddressEventFeatures&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class AddressEvent3D : public eEvent
{
protected:
    int disparity;
    int polarity;
    int x;
    int y;

public:
    AddressEvent3D();
    AddressEvent3D(const AddressEvent3D &event);
    AddressEvent3D(const yarp::os::Bottle &packets, const int pos=0);

    AddressEvent3D &operator=(const AddressEvent3D &event);
    bool operator==(const AddressEvent3D &event);

    int getDisparity() const               { return disparity;          }
    int getPolarity() const                { return polarity;           }
    int getX() const                       { return x;                  }
    int getY() const                       { return y;                  }

    void setDisparity(const int disparity) { this->disparity=disparity; }
    void setPolarity(const int polarity)   { this->polarity=polarity;   }
    void setX(const int x)                 { this->x=x;                 }
    void setY(const int y)                 { this->y=y;                 }

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const AddressEvent3D&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class AddressEvent3DFeatures : public AddressEvent3D
{
protected:
    int orientation;
    int xFlow;
    int yFlow;

public:
    AddressEvent3DFeatures();
    AddressEvent3DFeatures(const AddressEvent3DFeatures &event);
    AddressEvent3DFeatures(const yarp::os::Bottle &packets, const int pos=0);

    AddressEvent3DFeatures &operator=(const AddressEvent3DFeatures &event);
    bool operator==(const AddressEvent3DFeatures &event);

    int getOrientation() const                 { return orientation;            }
    int getXFlow() const                       { return xFlow;                  }
    int getYFlow() const                       { return yFlow;                  }

    void setOrientation(const int orientation) { this->orientation=orientation; }
    void setXFlow(const int xFlow)             { this->xFlow=xFlow;             }
    void setYFlow(const int yFlow)             { this->yFlow=yFlow;             }

    int getLength() const { return 2; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const AddressEvent3DFeatures&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent : public eEvent
{
protected:
    int id;
    int channel;
    int xCog;
    int yCog;

public:
    ClusterEvent();
    ClusterEvent(const ClusterEvent &event);
    ClusterEvent(const yarp::os::Bottle &packets, const int pos=0);

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

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEvent&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEventFeatures1 : public ClusterEvent
{
public:
    ClusterEventFeatures1();
    ClusterEventFeatures1(const yarp::os::Bottle &packets, const int pos=0);

    int getLength() const { return 2; }
    yarp::os::Bottle encode() const;
};


/**************************************************************************/
class ClusterEventFeatures2 : public ClusterEventFeatures1
{
protected:
    int shapeType;
    int xSize;
    int ySize;
    int numAE;

public:
    ClusterEventFeatures2();
    ClusterEventFeatures2(const ClusterEventFeatures2 &event);
    ClusterEventFeatures2(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEventFeatures2 &operator=(const ClusterEventFeatures2 &event);
    bool operator==(const ClusterEventFeatures2 &event);

    int getShapeType() const               { return shapeType;          }
    int getXSize() const                   { return xSize;              }
    int getYSize() const                   { return ySize;              }
    int getNumAE() const                   { return numAE;              }

    void setShapeType(const int shapeType) { this->shapeType=shapeType; }
    void setXSize(const int xSize)         { this->xSize=xSize;         }
    void setYSize(const int ySize)         { this->ySize=ySize;         }
    void setNumAE(const int numAE)         { this->numAE=numAE;         }

    int getLength() const { return 3; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEventFeatures2&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEventFeatures3 : public ClusterEventFeatures2
{
protected:
    int shapeProb;
    int xVel;
    int yVel;
    int numAE;

public:
    ClusterEventFeatures3();
    ClusterEventFeatures3(const ClusterEventFeatures3 &event);
    ClusterEventFeatures3(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEventFeatures3 &operator=(const ClusterEventFeatures3 &event);
    bool operator==(const ClusterEventFeatures3 &event);

    int getShapeProb() const               { return shapeProb;          }
    int getXVel()      const               { return xVel;               }
    int getYVel()      const               { return yVel;               }
    int getNumAE()     const               { return numAE;              }

    void setShapeProb(const int shapeProb) { this->shapeProb=shapeProb; }
    void setXVel(const int xVel)           { this->xVel=xVel;           }
    void setYVel(const int yVel)           { this->yVel=yVel;           }
    void setNumAE(const int numAE)         { this->numAE=numAE;         }

    int getLength() const { return 4; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEventFeatures3&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent3D : public eEvent
{
protected:
    int channel;
    int disparity;
    int xCog;
    int yCog;

public:
    ClusterEvent3D();
    ClusterEvent3D(const ClusterEvent3D &event);
    ClusterEvent3D(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEvent3D &operator=(const ClusterEvent3D &event);
    bool operator==(const ClusterEvent3D &event);

    int getChannel() const                 { return channel;            }
    int getDisparity() const               { return disparity;          }
    int getXCog() const                    { return xCog;               }
    int getYCog() const                    { return yCog;               }

    void setChannel(const int channel)     { this->channel=channel;     }
    void setDisparity(const int disparity) { this->disparity=disparity; }
    void setXCog(const int xCog)           { this->xCog=xCog;           }
    void setYCog(const int yCog)           { this->yCog=yCog;           }

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEvent3D&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent3DFeatures1 : public ClusterEvent3D
{
protected:
    int numAE;

public:
    ClusterEvent3DFeatures1();
    ClusterEvent3DFeatures1(const ClusterEvent3DFeatures1 &event);
    ClusterEvent3DFeatures1(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEvent3DFeatures1 &operator=(const ClusterEvent3DFeatures1 &event);
    bool operator==(const ClusterEvent3DFeatures1 &event);

    int  getNumAE() const          { return numAE;      }
    void setNumAE(const int numAE) { this->numAE=numAE; }

    int getLength() const { return 2; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEvent3DFeatures1&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent3DFeatures2 : public ClusterEvent3DFeatures1
{
protected:
    int shapeType;
    int xSize;
    int ySize;

public:
    ClusterEvent3DFeatures2();
    ClusterEvent3DFeatures2(const ClusterEvent3DFeatures2 &event);
    ClusterEvent3DFeatures2(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEvent3DFeatures2 &operator=(const ClusterEvent3DFeatures2 &event);
    bool operator==(const ClusterEvent3DFeatures2 &event);

    int getShapeType() const               { return shapeType;          }
    int getXSize() const                   { return xSize;              }
    int getYSize() const                   { return ySize;              }

    void setShapeType(const int shapeType) { this->shapeType=shapeType; }
    void setXSize(const int xSize)         { this->xSize=xSize;         }
    void setYSize(const int ySize)         { this->ySize=ySize;         }

    int getLength() const { return 3; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEvent3DFeatures2&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


/**************************************************************************/
class ClusterEvent3DFeatures3 : public ClusterEvent3DFeatures2
{
protected:
    int shapeProb;
    int xVel;
    int yVel;

public:
    ClusterEvent3DFeatures3();
    ClusterEvent3DFeatures3(const ClusterEvent3DFeatures3 &event);
    ClusterEvent3DFeatures3(const yarp::os::Bottle &packets, const int pos=0);

    ClusterEvent3DFeatures3 &operator=(const ClusterEvent3DFeatures3 &event);
    bool operator==(const ClusterEvent3DFeatures3 &event);

    int getShapeProb() const               { return shapeProb;          }
    int getXVel() const                    { return xVel;               }
    int getYVel() const                    { return yVel;               }

    void setShapeProb(const int shapeProb) { this->shapeProb=shapeProb; }
    void setXVel(const int xVel)           { this->xVel=xVel;           }
    void setYVel(const int yVel)           { this->yVel=yVel;           }

    int getLength() const { return 4; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const ClusterEvent3DFeatures3&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


 /**************************************************************************/
class HoughEvent : public eEvent
{
 protected:
  int channel;             // reference to the camera                  (bit0)
  int xCoc;                // x position of the center of the circle   (bit7-1)
  int yCoc;                // y position of the center of the circle   (bit15-8)
  int radius;              // reference to the radius of the circle    (bit25-16)

public:
    HoughEvent();
    HoughEvent(const HoughEvent &event);
    HoughEvent(const yarp::os::Bottle &packets, const int pos=0);

    HoughEvent &operator=(const HoughEvent &event);
    bool operator==(const HoughEvent &event);

    /* returns the camera that produced the Hough event */
    int getChannel() const                 { return channel;            }

    /* returns the radius of the circle */
    int getRadius() const                  { return radius;             }

    /* returns the x coordinate of the center of circle*/
    int getXCoc() const                    { return xCoc;               }

    /* returns the y coordinate of the center of circle*/
    int getYCoc() const                    { return yCoc;               }

    /*set channel information*/
    void setChannel(const int channel)     { this->channel = channel;    }

    /*set the radius information*/
    void setRadius(const int radius)       { this->radius = radius;      }

    /*set the x coordinate of the center of circle */
    void setXCoc(const int xCoc)           { this->xCoc = xCoc;          }

    /*set the y coordinate of the center of circle */
    void setYCoc(const int yCoc)           { this->yCoc = yCoc;          }

    int getLength() const { return 1; }
    bool operator==(const eEvent &event) { return operator==(dynamic_cast<const HoughEvent&>(event)); }
    yarp::os::Bottle   encode() const;
    yarp::os::Property getContent() const;
};


}

}

#endif


