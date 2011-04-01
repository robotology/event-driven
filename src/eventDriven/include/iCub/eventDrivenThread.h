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
#include <iostream>

#include "/usr/local/src/robot/iCub/contrib/src/eMorph/emorph_lib/include/eventBuffer.h"
#include "/usr/local/src/robot/iCub/contrib/src/eMorph/emorph_lib/include/eventConversion.h"


template <class eventBuffer>
class eventPort : public yarp::os::BufferedPort<eventBuffer> {

public:
    eventBuffer event;
    bool hasNewEvent;

    virtual void onRead(eventBuffer& event) {
    hasNewEvent = true;             // to be set to false once done with the events
    }

};


class eventDrivenThread : public yarp::os::Thread {
private:
    unsigned long* timeBuf;         // buffer for timestamp, ?? Stack
    int* eventsBuf;                 // buffer for events
    
    eventBuffer currentEvent;       // the current event that will be read and unmasked 
    unmask unmaskEvent;             // to unmask the event           
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    

    eventPort<eventBuffer> EportIn;              // buffered port listening to events through callback

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > eventPlot;     // output port to plot event    
    
    std::string name;               // rootname of all the ports opened by this thread
    
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

    
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    void setInputPortName(std::string inpPrtName);

    /**
    * function that plots the eventBuffer as a monochromatic image based on polarity
    * @param buffer pointer to event buffer
    * @param dim1   first dimension of the event buffer
    * @param dim2   second dimension of the event buffer
    */
    void plotEventBuffer(int* buffer, int dim1, int dim2);


    
};

#endif  //_EVENTDRIVEN_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

