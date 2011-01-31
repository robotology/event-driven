// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
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
 * @file unmask.h
 * @brief code for unmasking the events
 */

#ifndef UNMASK_H
#define UNMASK_H

#include <iostream>
#include <sstream>
#include <ctime>

#include <yarp/os/all.h>

//Other dependency
#include <iCub/config.h>

class unmask : public yarp::os::RateThread{
private:
    int id;
    int nb_trame;

    int sz;
    int* buffer;                          // buffer representing the event in image plane (left)
    unsigned long* timeBuffer;        // buffer contains the timestamp of the particular location (left)
    int* bufferRight;                     //buffer representing the event in image plane (right)
    unsigned long* timeBufferRight;   // buffer contains the timestamp of the particular location (right)
    //int* fifoEvent;
    //int* fifoEvent_temp;
    //int* fifoEvent_temp2;
    unsigned long timestamp;         // 16 bits variable to save the timestamp
    unsigned long timestamplong;         // variable 32 bits to save the timestamp
    unsigned long lasttimestamp;
    unsigned long eldesttimestamp;        // timestamp of the eldest event in the buffer 
    short cartX, cartY, polarity, camera;

    int wrapAdd;
    unsigned int xmask;              // 16 bits mask for unmasking of the address
    unsigned int ymask;             // 16 bits mask for unmasking of the address
    long int xmasklong;         // 32 bits mask for unmasking of the event
    long int ymasklong;         // 32 bits mask for unmasking of the event
    int yshift;                     // shift of 8 bits for getting the y in 16 bits address
    int xshift;                     // shift of 1 bit to get the x in 16 bit (polarity)
    int yshift2;                     // shift of 16 bits for getting the y in 32 bits address
    int polshift;                    // shift necessary to cast the unmasked value to short
    int polmask;                     // mask necessary to extract the polarity
    int camerashift;                 // shift for cast back into short the camera reference 
    int cameramask;                  // mask necessary to extract which camera has produced the output
    int retinalSize;                 // size of the retina
    int minValue;
    int maxValue;
    int countEvent;                     //counter of the number of events saved in the buffer1
    int countEvent2;                     //counter of the number of events saved in the buffer2
    int numKilledEvents;                //number of the element that have be removed from the buffer
    bool temp1;                         //boolean flag that indicates where the events have to be saved

    FILE* uEvents;
    yarp::os::Semaphore countEventLocker;
    yarp::os::Semaphore countEventLocker2;

public:
    /**
    * @brief Function returns the pointer to the buffer that containes events counts
    * @param none
    * @return pointer to the buffer of event counts
    */
    int* getEventBuffer();

    /**
    * @brief Function returns the pointer to the buffer that containes timestamps
    * @param none
    * @return pointer to the buffer of timestamps
    */
    unsigned long* getTimeBuffer();

    /**
    * @brief Function that cleans buffer that containes events counts
    * @param none
    * @return none
    */
    void cleanEventBuffer();
    
    /**
    * @brief get the min number of negative events
    * @return minvalue
    */
    int getMinValue();

    /**
    * @brief get the max number of negative events
    * @return minvalue
    */
    int getMaxValue();

    /**
    * returns the eldest timestamp
    */
    unsigned long getEldestTimeStamp();

    /**
    * returns the last timestamp
    */
    unsigned long getLastTimestamp();

    /**
    * force the values of the lasttimestamp 
    * @param the value to be set
    */
     void setLastTimestamp(unsigned long value);

    /**
    * default constructor
    */
    unmask();

    /**
    * destructor
    */
    ~unmask();

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
    * function that given a reference to the list of char read from the port and the number of packet received
    * unmasks the event in term of x,y, polarity and time stamp and update the buffer
    * @param data reference to the vector of char (the read data)
    * @param size size of the last reading from the port
    */
    void unmaskData(char* data, int size);

    /**
    * function that given a reference to the list of long int(32 bits) read from the port and the number of packet received
    * unmasks the event in term of x,y, polarity and time stamp and update the buffer
    * @param evPU blob representing the position of the pixel the polarity and the camera produced from
    * @param x position of the event along the x axis
    * @param y position of the event along the y axis
    * @param pol polarity of the event +1, -1
    * @param camera reference to the camera that has produced the event
    */
    void unmaskEvent(long int evPU, short& x, short& y, short& pol, short& camera);

    /**
    * @brief This method unmasked the raw which come from the TCP socket
    * This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
    * @param *evPU A pointer on the raw casted from char* to int*
    * @param x Set with the x coordinate of the pixel
    * @param y Set with the y coordinate of the pixel
    * @param pol Set with the ON/OFF polarity of the pixel.
    */
    void unmaskEvent(unsigned int evPu, short& x, short& y, short& pol, short& camera);
};

#endif //UNMASK_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

