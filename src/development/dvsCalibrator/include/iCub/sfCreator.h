// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email:shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file spatialFrameCreator.h
 * @brief Definition of a thread that takes creates a spatially complete image (no coordinate events are lost) over a small time-window
 */


#ifndef _SFC_THREAD_H_
#define _SFC_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdlib.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventConversion.h>
#include <iCub/emorph/eventCodec.h>

//typedef unsigned long long int uint64_t;
#define u64 uint64_t
#ifndef CHUNKSIZE 
#define CHUNKSIZE 32768
#endif


using namespace emorph::ebuffer;

class sfCreator : public yarp::os::BufferedPort<yarp::os::Bottle>{
private:
    
    int count;                          // loop counter of the thread
    bool cameraType;
    struct timeval tvstart,tvend;
    //struct timespec start_time, stop_time;    
    int receivedBufferSize;
    char* receivedBuffer;
    emorph::ecodec::eEventQueue queueEventsRcvd;
    AER_struct* listOfEvents;
    double deltaTimeWindow;
    double currentTime;
    bool updateWindow;
    bool isAccessing;
    bool isUpdating;
    bool isInitialized;
    int retinalSize;                    // dimension of the retina device
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPort;            // port whre the output (left/right) is sent
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imagePort;                                  // current image representing the signal on the left/right camera
    std::string name;                   // rootname of all the ports opened by this thread
    yarp::os::Semaphore mutexForRead, mutexForGetImage;           // semaphore thar regulates the access to the buffer resource
    unmask* unmask_events;               // object that unmask events
    FILE* fout;                          // file for temporarely savings of events
    FILE* raw;                           // file dumper for debug
public:
    /**
    * constructor
    */
    sfCreator(bool cam);

    /**
     * destructor
     */
    ~sfCreator();

    void updateMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image,double timeStampNow);
    
    void plotMonoImage();
    
    //void plotMonoImage(bool camera=true);
    
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image);
    
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
    
    void setRetinalSize(int value) {
        retinalSize = value;
    }
    
    void onRead(yarp::os::Bottle& rcvdPackets);
    
    bool isSFCreatorInitialized() {
        return isInitialized;
    }
    
    void setIsAccessing(bool flag){
        isAccessing = flag;
    }
    bool getIsAccessing(){
        return isAccessing;
    }
    bool getIsUpdating(){
        return isUpdating;
    }
    
    bool getCamera(){
        return cameraType;
    }
        
    
};

#endif  //_SFC_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

