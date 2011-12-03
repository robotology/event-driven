
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
    bool isValid()        { return valid; }
    std::string getType() { return type;  }

    virtual int getLength()=0;
    virtual yarp::os::Bottle encode()=0;
    virtual yarp::os::Property getContent()=0;

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

public:
    eEventQueue() { }
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

    int getStamp()                 { return stamp;      }
    void setStamp(const int stamp) { this->stamp=stamp; }

    int getLength() { return 1; }
    yarp::os::Bottle encode();
    yarp::os::Property getContent();
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

    int getChannel()                     { return channel;          }
    int getPolarity()                    { return polarity;         }
    int getX()                           { return x;                }
    int getY()                           { return y;                }

    void setChannel(const int channel)   { this->channel=channel;   }
    void setPolarity(const int polarity) { this->polarity=polarity; }
    void setX(const int x)               { this->x=x;               }
    void setY(const int y)               { this->y=y;               }

    int getLength() { return 1; }
    yarp::os::Bottle encode();
    yarp::os::Property getContent();
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

    int getOrientation()                       { return orientation;            }
    int getXFlow()                             { return xFlow;                  }
    int getYFlow()                             { return yFlow;                  }

    void setOrientation(const int orientation) { this->orientation=orientation; }
    void setXFlow(const int xFlow)             { this->xFlow=xFlow;             }
    void setYFlow(const int yFlow)             { this->yFlow=yFlow;             }

    int getLength() { return 2; }
    yarp::os::Bottle encode();
    yarp::os::Property getContent();
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

    int getDisparity()                     { return disparity;          }
    int getPolarity()                      { return polarity;           }
    int getX()                             { return x;                  }
    int getY()                             { return y;                  }

    void setDisparity(const int disparity) { this->disparity=disparity; }
    void setPolarity(const int polarity)   { this->polarity=polarity;   }
    void setX(const int x)                 { this->x=x;                 }
    void setY(const int y)                 { this->y=y;                 }

    int getLength() { return 1; }
    yarp::os::Bottle encode();
    yarp::os::Property getContent();
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

    int getOrientation()                       { return orientation;            }
    int getXFlow()                             { return xFlow;                  }
    int getYFlow()                             { return yFlow;                  }

    void setOrientation(const int orientation) { this->orientation=orientation; }
    void setXFlow(const int xFlow)             { this->xFlow=xFlow;             }
    void setYFlow(const int yFlow)             { this->yFlow=yFlow;             }

    int getLength() { return 2; }
    yarp::os::Bottle encode();
    yarp::os::Property getContent();
};


///**************************************************************************/
//class ClusterEvent : public eEvent
//{
//};
//
//
///**************************************************************************/
//class ClusterEvent3D : public eEvent
//{
//};
//
//
///**************************************************************************/
//class TimeStamp : public eEvent
//{
//};


}

}

#endif


