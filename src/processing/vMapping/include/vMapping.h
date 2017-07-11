//
// Created by miacono on 11/07/17.
//

#ifndef ICUB_EVENT_DRIVEN_VMAPPING_H
#define ICUB_EVENT_DRIVEN_VMAPPING_H

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>

//TODO There is a copy of this class in the autosaccade module. Find a better location
class EventBottleManager : public yarp::os::BufferedPort<ev::vBottle> {
private:
    
    //for helping with timestamp wrap around
    ev::vtsHelper unwrapper;
    
    //rate counters
    yarp::os::Semaphore mutex;
    unsigned long int latestStamp;
    unsigned int vCount;
    double yRate;
    bool isReading;
    ev::vQueue vQueue;

public:
    
    EventBottleManager();
    
    bool    open(const std::string &name);
    
    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);
    
    //the getting functions of the parent class
    unsigned long int getTime();
    unsigned long int popCount();
    double getEventRate() { return yRate; }
    
    ev::vQueue getEvents() ;
    bool start();
    bool stop();
    
};

class ImagePort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > {

private:
    yarp::sig::ImageOf<yarp::sig::PixelBgr> image;
    yarp::os::Mutex mutex;
    bool imageReady;
public:
    
    ImagePort() : imageReady(false) { this->useCallback(); }
    virtual void onRead( yarp::sig::ImageOf<yarp::sig::PixelBgr> &inImg );
    yarp::sig::ImageOf<yarp::sig::PixelBgr> getImage();
    bool isImageReady() { return imageReady;}
};


class vMappingModule : public yarp::os::RFModule {
private :
    
    ImagePort imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr > > imagePortOut;
    EventBottleManager vPort;

public :
    
    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the modulereturn
    
    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
    
};



#endif //ICUB_EVENT_DRIVEN_VMAPPING_H
