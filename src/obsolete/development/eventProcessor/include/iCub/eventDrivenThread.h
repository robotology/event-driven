// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
 * website: www.robotcub.org 
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

/**
 * @file eventDrivenThread.h
 * @brief Definition of a thread that receives AER information from aexGrabber and unmasks it
 * eventDriven eventDriven (see eventDrivenModule.h).
 */


#ifndef _EVENTDRIVEN_THREAD_H_
#define _EVENTDRIVEN_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventConversion.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#define BUCKET_THRESHOLD 113
#define COMMAND_VOCAB_PSAC VOCAB4('p','s','a','c')
#define IMAGE_WD 128
#define IMAGE_HT 128

using namespace emorph::ebuffer;

template <class eventBuffer>

class eventPort : public yarp::os::BufferedPort<eventBuffer> {

public:
    eventBuffer packet;
    int* bufTmp;
    int* bufTmpRight;
    yarp::os::BufferedPort<yarp::os::Bottle > eventCoord;                               // for coordinates of events above threshold 
    unmask unmaskOneEvent; 
    yarp::os::BufferedPort<yarp::os::Bottle > pcSz; 
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > eventPlot;     // output port to plot event

    yarp::os::Semaphore readMutex;

    
    
      

    eventPort() {
             
            
    }   
    ~eventPort() {
        
    }


    /*
    void plotEventBuffer(int* buffer) {
    yarp::sig::ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();
    unsigned char* imageTmp = imageForEventBuffer.getRawImage();
    int* bufTmp = buffer;
    for(int i = 0; i < IMAGE_HT; ++i) {
        for(int j = 0; j < IMAGE_WD; ++j) {
            *imageTmp = *bufTmp;
            imageTmp++;
            bufTmp++;
        }
    } 
    eventPlot.write();   

    }*/





/*************************************************/

    virtual void onRead(eventBuffer& packet) {
        
        //readMutex.wait();
        int packetSize = packet.get_sizeOfPacket();
        if(packetSize<1) {
            return;
        }
        yarp::os::Bottle& b = pcSz.prepare();
        b.clear();
        b.addInt(packetSize);
        pcSz.write();

        //printf("%d\n",packetSize);
        short oneX,oneY,onePol, camera;

        unmaskOneEvent.unmaskData(packet.get_packet(), packetSize);
        bufTmp = unmaskOneEvent.getEventBuffer(true);  // true for left camera
        bufTmpRight = unmaskOneEvent.getEventBuffer(false);  // false for right camera

        int maxVal = -IMAGE_WD -1; 
        int maxX = -1; 
        int maxY = -1;
    
        int maxValR = -IMAGE_WD -1; 
        int maxXR = -1; 
        int maxYR = -1;

        for(int i = 0; i < IMAGE_HT; ++i) {
            for(int j = 0; j < IMAGE_WD; ++j) {
                if ((*bufTmp) > BUCKET_THRESHOLD && (*bufTmp) > maxVal ) {
                    maxVal = *bufTmp;
                    maxX = i;
                    maxY = j;
                }
                if ((*bufTmpRight) > BUCKET_THRESHOLD && (*bufTmpRight) > maxVal ) {
                    maxValR = *bufTmpRight;
                    maxXR = i;
                    maxYR = j;
                }
                bufTmpRight++;
                bufTmp++;
            }
        }

        if(maxVal < -IMAGE_WD) {
            printf("No points above threshold!\n");
        }
        else {

            yarp::os::Bottle& coord = eventCoord.prepare();
            coord.clear();                    
            coord.addInt(maxVal);
            coord.addInt(maxX);
            coord.addInt(maxY);
            coord.addString(" R:");
            coord.addInt(maxValR);
            coord.addInt(maxXR);
            coord.addInt(maxYR);                    
            eventCoord.write();
            
            
        }
        maxVal = -IMAGE_WD -1;        
        packetSize = 0;
        //readMutex.post(); 
        //yarp::os::Time::delay(.05);    
             
    }

    /**
    * function that converts an event (set of tuples (x, y, polarity) into a binary image frame
    * @param eventBuf pointer to event buffer
    * @param eventSize size of the event
    * @param retImage pointer to openCV image thus formed
    */
    void event2Frame(int* buff, int eventSize, IplImage* retImage) ;
};


class eventDrivenThread : public yarp::os::Thread {
private:
    unsigned long* timeBuf;         // buffer for timestamp, ?? Stack
    int* eventsBuf;                 // buffer for events
    
    eventBuffer currentEvent;       // the current event that will be read and unmasked 
    
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    eventPort<eventBuffer> EportIn;                                                  // buffered port listening to events through callback
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > eventPlot;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread
    IplImage* eventFrame;
    
public:
    /**
    * constructor default
    */
    eventDrivenThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    eventDrivenThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~eventDrivenThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
    * function that plots the eventBuffer as a monochromatic image based on polarity
    * @param buffer pointer to event buffer
    * @param dim1   first dimension of the event buffer
    * @param dim2   second dimension of the event buffer
    */
    void plotEventBuffer(int* buffer);

    

};

#endif  //_EVENTDRIVEN_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

