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
#include <iCub/plotterThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdlib.h>

//typedef unsigned long long int uint64_t;
#define u64 uint64_t


class cfCollectorThread : public yarp::os::RateThread {
private:
    
    int count;                          // loop counter of the thread
    struct timeval tvstart,tvend;
    //struct timespec start_time, stop_time;
    u64 Tnow;
    unsigned long precl;
    unsigned long lc;
    unsigned long lcprev;
    unsigned long rcprev;
    unsigned long rc;
    FILE* raw;                          // file dumper for debug
    double microseconds;
    double microsecondsPrev;
    int countStop;                      // counter of equal timestamp
    int countDivider;                   // divider of the count
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    int synchPeriod;                    // synchronization period between events and viewer
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPort;            // port whre the output (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPortRight;       // port whre the output (right) is sent
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageLeft;                                  //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageRight;                                 //image representing the signal on the right camera
    std::string name;                   // rootname of all the ports opened by this thread
    bool verb;
    bool synchronised;                   // flag to check whether the microsecond counter has been synchronised
    bool greaterHalf;                    // indicates whether the counter has passed the half of the range
    bool idle;                           // controls idle mode
    bool firstRun;                       // flag that check whether the run is a first useful run    
    bool logPolar;                       // flag that indicates whether the viewer represent logpolar information
    bool stereo;                         // flag that indicates whether the synchronization is stereo 
    unsigned long minCount;              // minimum timestamp allowed for the current frame
    unsigned long maxCount;              // maximum timestamp allowed for the current frame
    unsigned long minCountRight;
    unsigned long maxCountRight;
    double startTimer;
    double interTimer;
    double endTimer;
    yarp::os::Semaphore mutex;          // semaphore thar regulates the access to the buffer resource
    clock_t endTime,startTime;
    long T1,T2;
    plotterThread* pThread;                 // plotterThread for the trasformation of the event in images
    cFrameConverter* cfConverter;           // receives real-time events
    unmask unmask_events;                   // object that unmask events
    char* bufferRead;                       // buffer of events read from the port
    char* bufferCopy;                       // local copy of the events read
    FILE* fout;                             // file for temporarely savings of events
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

    /**
    * @brief returns a mono image of the output of the dvs camera (either left or right)
    * @param pixelMono reference to the image contains the counts of events
    * @param minCount reference to the min timestamp in the frame
    * @param maxCount reference to the max timestamp in the frame
    * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
    */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera);

    /**
     * @brief function that describes whether the synchronization is stereo
     * @param value boolean value to assign to the variable
     */
    void setStereo(bool value) {stereo = value; };

    /**
     * @brief function that sets the synchronization period between the events and viewer
     * @param value integer representing the synchronization period (minim. 1 runcycle)
     */
    void setSynchPeriod(int value) {synchPeriod = value; };

    /**
     * @brief function that indicates whether the viewer reppresent logpolar information
     */
    void setLogPolar(int value) {logPolar = value; };
    
};

#endif  //_CF_COLLECTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

