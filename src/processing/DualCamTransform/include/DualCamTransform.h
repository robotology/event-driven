/*
 * Copyright (C) 2010 iCub Facility
 * Authors: Massimiliano Iacono
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

#ifndef ICUB_EVENT_DRIVEN_DUALCAMTRANSFORM_H
#define ICUB_EVENT_DRIVEN_DUALCAMTRANSFORM_H

#include <iostream>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
# include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <opencv2/opencv.hpp>

class EventPort : public yarp::os::BufferedPort<ev::vBottle> {
private:
    yarp::os::Semaphore mutex;
    ev::vQueue vLeftQueue;
    ev::vQueue vRightQueue;
    bool isReading{ false };
    
public:
    
    EventPort() {this->useCallback();}
    
    void startReading() { isReading = true; }
    void stopReading() { isReading = false; }
    bool isPortReading() {return isReading;}
    void clearQueues();
    void onRead(ev::vBottle &bot);
    ev::vQueue getEventsFromChannel(int channel);
};

class ImagePort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > {

private:
    yarp::sig::ImageOf<yarp::sig::PixelBgr> image;
    yarp::os::Mutex mutex;
    bool imageReady{ false };
public:
    
    ImagePort() { this->useCallback(); }
    virtual void onRead( yarp::sig::ImageOf<yarp::sig::PixelBgr> &inImg );
    yarp::sig::ImageOf<yarp::sig::PixelBgr> getImage();
    bool isImageReady() { return imageReady;}
    
};

class EventCollector : public yarp::os::Thread {
private:
    EventPort vPort;
public:
    
    bool open (const std::string &name){ return vPort.open(name); }
    void close() { vPort.close();}
    void interrupt() {vPort.interrupt(); }
    ev::vQueue getEventsFromChannel(int channel){return vPort.getEventsFromChannel(channel);}
    void run(){}
    void clearQueues() {vPort.clearQueues();}
    void startReading() {vPort.startReading();}
    void stopReading() {vPort.stopReading();}
    bool isPortReading() {return vPort.isPortReading();}
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
    void getImageSize(int &width, int& height);
    void run(){}
};

class DualCamTransformModule : public yarp::os::RFModule {
private :
    
    //Variables for calibration
    bool calibrateLeft;
    bool calibrateRight;
    bool cropToImage;
    int nIter;
    int maxIter;
    ImageCollector vLeftImageCollector;
    ImageCollector vRightImageCollector;
    
    std::string confFileName;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr > > leftImagePortOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr > > rightImagePortOut;
    yarp::os::BufferedPort<ev::vBottle> vPortOut;
    ImageCollector leftImageCollector;
    ImageCollector rightImageCollector;
    EventCollector eventCollector;
    yarp::sig::Matrix leftH;
    yarp::sig::Matrix rightH;
    int height;
    int width;
    int leftCanvasHeight;
    int leftCanvasWidth;
    int rightCanvasHeight;
    int rightCanvasWidth;
    int leftXOffset;
    int leftYOffset;
    int rightXOffset;
    int rightYOffset;
    
    
public :
    
    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the module return
    
    
    virtual bool updateModule();
    virtual double getPeriod();
    
    bool performCalibStep( yarp::sig::ImageOf<yarp::sig::PixelBgr> &frame
                           , yarp::sig::ImageOf<yarp::sig::PixelBgr> &vImg, yarp::sig::Matrix &homography ) const;
    
    void finalizeCalibration( yarp::sig::Matrix &homography, std::string groupName);
    
    bool readConfigFile( const yarp::os::ResourceFinder &rf, std::string groupName
                         , yarp::sig::Matrix &homography ) const;
    
    void transform( yarp::sig::ImageOf<yarp::sig::PixelBgr> &img, const ev::vQueue &vQueue
                    , const yarp::sig::Matrix &homography, int xOffset, int yOffset ) ;
    
    void getCanvasSize( const yarp::sig::Matrix &homography, int &canvasWidth, int &canvasHeight, int &xOffset
                            , int &yOffset ) const;
};



#endif //ICUB_EVENT_DRIVEN_DUALCAMTRANSFORM_H
