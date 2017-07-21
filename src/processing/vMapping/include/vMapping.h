//
// Created by miacono on 11/07/17.
//

#ifndef ICUB_EVENT_DRIVEN_VMAPPING_H
#define ICUB_EVENT_DRIVEN_VMAPPING_H

#include <iostream>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
# include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <opencv2/opencv.hpp>

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

class EventCollector : public yarp::os::Thread {
private:
    EventBottleManager vPort;
public:
    bool open (const std::string &name){ return vPort.open(name); }
    void close() { vPort.close();}
    void interrupt() {vPort.interrupt(); }
    ev::vQueue getEvents(){return vPort.getEvents();}
    void run(){vPort.start();}
};

class ImageCollector : public yarp::os::Thread {
private:
    ImagePort imagePort;
public:
    bool open (const std::string &name) { return imagePort.open(name); }
    void close() { imagePort.close();}
    void interrupt() { imagePort.interrupt(); }
    bool isImageReady() { return imagePort.isImageReady();}
    yarp::sig::ImageOf <yarp::sig::PixelBgr> getImage() {return imagePort.getImage(); }
    void run(){}
};

class vMappingModule : public yarp::os::RFModule {
private :
    
    //Variables for calibration
    bool calibrate;
    int nIter;
    int maxIter;
    
    std::string confFileName;
    ImageCollector imageCollector;
    ImageCollector vImageCollector; //used for calibration
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr > > imagePortOut;
    EventCollector eventCollector;
    yarp::sig::Matrix homography;
public :
    
    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the module return
    
    
    virtual bool updateModule();
    virtual double getPeriod();
    
};



#endif //ICUB_EVENT_DRIVEN_VMAPPING_H
