// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file cfCollectorThread.h
 * @brief Definition of a thread that receive events from DVS camera and extracts frame-based representations of the readings
 * (see dvsGrabberModule.h).
 */

#ifndef _CF_COLLECTOR_THREAD_H_
#define _CF_COLLECTOR_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/all.h>
#include <iCub/cartesianFrameConverter.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <inttypes.h>

#define u64 uint64_t


class cfCollectorThread : public yarp::os::RateThread {
private:
    
    int count;                          // loop counter of the thread
    struct timeval tvstart,tvend;
    struct timespec start_time, stop_time;
    u64 Tnow;
    
    double microseconds;
    double microsecondsPrev;
    int countDivider;                   // divider of the count
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPort;       // port whre the output (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPortRight;       // port whre the output (right) is sent
    std::string name;                   // rootname of all the ports opened by this thread
    bool synchronised;                       // flag to check whether the microsecond counter has been synchronised
    unsigned long minCount;              // minimum timestamp allowed for the current frame
    unsigned long maxCount;              // maximum timestamp allowed for the current frame
    double startTimer;
    double endTimer;
    yarp::os::Semaphore mutex;          // semaphore thar regulates the access to the buffer resource
    clock_t endTime,startTime;
    long T1,T2;
    cFrameConverter* cfConverter; //receives real-time events
public:
    /**
    * default constructor
    */
    cfCollectorThread();

    /**
     * destructor
     */
    ~cfCollectorThread();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

};

#endif  //_CF_COLLECTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

