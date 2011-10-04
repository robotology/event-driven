// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file processingThread.h
 * @brief Definition of a thread that processes train of events
 * (see processingthread.h).
 */

#ifndef _PROCESSING_THREAD_H_
#define _PROCESSING_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <string>
#include <iCub/config.h>


class processingThread : public yarp::os::RateThread {
private:    
    int count;                           // loop counter of the thread
    int retinalSize;                     // dimension of the retina
    int countEM1;                        // counter of bytes in the bufferEM1
    int countEM2;                        // counter of bytes in the bufferEM1
    int countEM3;                        // counter of bytes in the bufferEM1
    int countEM4;                        // counter of bytes in the bufferEM1
    int dimEM;
    float lambda;                        // integration factor
    int width, height;                   // dimension of the extended input image (extending)
    int height_orig, width_orig;         // original dimension of the input and output images
    std::string name;                         // nameroot for ports
    bool synchronised;                   // flag to check whether the microsecond counter has been synchronised
    struct aer* bufferEM1;               // local copy of the events read
    struct aer* bufferEM2;               // local copy of the events read
    struct aer* bufferEM3;               // local copy of the events read
    struct aer* bufferEM4;               // local copy of the events read
    struct aer* cartEM;                  // vector of mean values across EMs in cartesian space
    struct aer* pEM;                     // pointer to EM vector  
    struct aer* pointerEM;         
public:
    /**
    * default constructor
    */
    processingThread();

    /**
     * destructor
     */
    ~processingThread();

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
     * @brief function that resets the counter of EMs
     */     
    void resetTOTEM() {memset(cartEM, 0, retinalSize * retinalSize * sizeof(unsigned long)); };

    /**
     * function that passes the pointer to EM buffers
     * @param bufferEM1 pointer to the buffer of events
     * @param bufferEM2 pointer to the buffer of events
     * @param bufferEM3 pointer to the buffer of events
     * @param bufferEM4 pointer to the buffer of events
     */
    void setEM(aer* bufferEM1, aer* bufferEM2, aer* bufferEM3, aer* bufferEM4); 

    /**
     * function that returns the pointer to the buffer of mean exposure measures
     */
    void getEM(aer** pointerEM, int* dimEM);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);


    unsigned long look4opposite(aer* buffer,int initPos, int countTOT);

    void addBufferEM(aer* event);

};

#endif  //_PROCESSING_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

