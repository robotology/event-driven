// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file eventCartesianCollector.h
 * @brief A class inherited from the bufferefPort class created in order to read events.
 */

#ifndef _EVENT_CARTESIAN_COLLECTOR_H
#define _EVENT_CARTESIAN_COLLECTOR_H


#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventConversion.h>
#include <iCub/emorph/eventBottle.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>

class eventCartesianCollector:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer> {
public:
    eventCartesianCollector();
    ~eventCartesianCollector();

    /**
    * overwritten function for handling events as soon as they arrive
    */
    virtual void onRead(emorph::ebuffer::eventBuffer& b);

    /**
    * overwritten function for handling events as soon as they arrive as eventBottle
    */  
    void onRead(eventBottle& i_ub);

    /**
    * function that returns a pointer to read buffer 
    * @return pointer to the buffer that has to be read
    */
    char* getBuffer() { return pcRead; };

    /**
    * function that copies a chunk of elements
    * @param pointer to the buffer that has to be read
    */
    void copyChunk(char* pointer);

    /**
    * @brief returns a mono image of the output of the dvs camera (either left or right)
    * @param pixelMono reference to the image contains the counts of events
    * @param minCount reference to the min timestamp in the frame
    * @param maxCount reference to the max timestamp in the frame
    * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
    */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera);
    
    /**
    * function that return the last time stamp saved in the buffer for left camera
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getLastTimeStamp();

    /**
    * function that return the last time stamp saved in the buffer for right camera
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getLastTimeStampRight();

    /**
    * function that return the last time stamp saved in the buffer
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getEldestTimeStamp();

    /**
    * @brief clears monoImage collection of events
    * @param pixelMono reference to the image contains the counts of events
    */
     void clearMonoImage();

    /**
       function that restarts from zero the list of memorised timestamp
     */
    void resetTimestamps();

    /* 
     *function that resets the buffer of conversion
     */
    void reset();

    /**
     * function return whether the conversion is valid
     */
    bool isValid(){return valid;};


    /**
     * @brief function thatset the dimension of the output image
     * @param value the dimension in pixels of the retina device
     */
    void setRetinalSize(int value) {
        retinalSize = value;
    }

    /**
     * extract a bottle of events
     */
    void extractBottle(yarp::os::Bottle* tempBottle);



private:
    bool valid;
    short state;
    int retinalSize;                                            // dimension of the retina default 128x128
    int totDim;                                                 // total dimension of the read buffer
    int outputWidth, outputHeight;                              // dimension of the output image default 320x240
    int receivedBufferSize;                                     // dimension of the received packet
    
    int extractPosition;                                        // position in the buffer where the bottle is extracted 
    int insertPosition;                                         // position in the buffer where the bottle is saved   
    unsigned long previousTimeStamp;                            // timestamp at the previous run
    char* converterBuffer;                                      // buffer used as saved
    char* converterBuffer_copy;                                 // copy of the buffer pointer, points to the location for freeing
    char* receivedBuffer;                                       // temporarely pointer to the received buffer
    char* pcRead;                                               // pointer to the location where to read events
    char* pcBuffer;                                             // pointer where to buffer events
    yarp::os::Bottle** bufferBottle;                            // buffer of received bottles

    static const int bottleBufferDimension = 10;                // dimension of the bottleBuffer

    //unmask unmask_events;         // object in charge of unmasking the events
    //converter convert_events;       // object in charge of converting the events into an image
    yarp::os::Semaphore mutex;      // semaphore for thehandling resource buffer
    clock_t start_u;
    clock_t start_p;
    clock_t stop;

    yarp::os::Semaphore** semBottleBuffer;                      // semaphore for the buffer of bottles


    FILE* readEvents;
    FILE* fout;
};

#endif //_EVENT_CARTESIAN_COLLECTOR_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

