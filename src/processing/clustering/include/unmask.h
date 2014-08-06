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

// general includes
#include <iostream>
#include <sstream>
#include <ctime>
#include <sys/types.h>
//#include <inttypes.h>

// yarp includes
#include <yarp/os/all.h>

//Other dependency
#include "config.h"
#include "eventCodec.h"

//using namespace emorph::ecodec;

typedef yarp::os::NetUint16 u16;
typedef yarp::os::NetUint32 u32;

class unmask : public yarp::os::RateThread{
private:
    int id;
    int nb_trame;
    int count;                            // counter of the unmasked events
    int countCLE;                         // counter CLE
    int countCLERight;
    int sz;
    int* buffer;                          // buffer representing the event in image plane (left)
    unsigned long* timeBuffer;            // buffer contains the timestamp of the particular location (left)
    int* bufferRight;                     // buffer representing the event in image plane (right)
    unsigned long* timeBufferRight;       // buffer contains the timestamp of the particular location (right)

    unsigned long timestamp;              // 16 bits variable to save the timestamp
    unsigned long timestamplong;          // variable 32 bits to save the timestamp
    unsigned long lasttimestamp;          // last timestamp acquired for the left camera
    unsigned int lastRecTimestamp;       // last timestamp received from the device driver (left or right independent)
    unsigned long lasttimestampright;     // last timestamp acquired for the right camera
    unsigned long eldesttimestamp;        // timestamp of the eldest event in the buffer
    short cartX, cartY, polarity;         // address x, y and polarity event information
    short camera;                         // camera information event information

    int wrapAdd;
    unsigned int xmask;              // 16 bits mask for unmasking of the address
    unsigned int ymask;              // 16 bits mask for unmasking of the address
    unsigned int xmaskshort;
    unsigned int ymaskshort;
    unsigned int polmaskshort;
    unsigned int cameramaskshort;
    long int xmasklong;              // 32 bits mask for unmasking of the event
    long int ymasklong;              // 32 bits mask for unmasking of the event
    int yshift;                      // shift of 8 bits for getting the y in 16 bits address
    int xshift;                      // shift of 1 bit to get the x in 16 bit (polarity)
    int yshift2;                     // shift of 16 bits for getting the y in 32 bits address
    int polshift;                    // shift necessary to cast the unmasked value to short
    int polmask;                     // mask necessary to extract the polarity
    int camerashift;                 // shift for cast back into short the camera reference
    int cameramask;                  // mask necessary to extract which camera has produced the output
    int retinalSize;                 // size of the retina
    int minValue;
    int maxValue;
    int countEvent;                  // counter of the number of events saved in the buffer1
    int countEvent2;                 // counter of the number of events saved in the buffer2
    int numKilledEvents;             // number of the element that have be removed from the buffer
    int responseGradient;            // dimension of the responseGradient
    bool temp1;                      // boolean flag that indicates where the events have to be saved
    bool validLeft,validRight;       // flag for validity of the events
    bool verb;                       // flag that indicates whether timestamp in forced to renewal
    bool dvsMode;                    // flag that initialises the typology of unmaske
    bool asvMode;                    // flag that represent the operational mode of the ASV chip
    bool wrapOcc;                    // flag that indicates when a wrapAraund has just occured
    FILE* uEvents;                   // uEvents file for debug
    FILE* maskEvents;                // maskEvents file for debug

    reprCLE *bufferCLELeft;          // pointer to CLE to represent
    reprHGE *bufferHGELeft;          // pointer to HGE to represent
    reprCLE *_bufferCLELeft;         // temporarely pointer to the buffer
    reprHGE *_bufferHGELeft;         // temporarely pointer to the buffe

    reprCLE *bufferCLERight;          // pointer to CLE to represent
    reprHGE *bufferHGERight;          // pointer to HGE to represent
    reprCLE *_bufferCLERight;         // temporarely pointer to the buffer
    reprHGE *_bufferHGERight;         // temporarely pointer to the buffe

    yarp::os::Semaphore countEventLocker;
    yarp::os::Semaphore countEventLocker2;

    /*semaphore for the additional information HGE and CLE*/
    yarp::os::Semaphore mutexHGELeft,mutexCLELeft;

public:
    /**
    * @brief Function returns the pointer to the buffer that containes events counts
    * @param indicates whether the required buffer belongs to the left or right camera (LEFT 1, RIGHT 0)
    * @return pointer to the buffer of event counts
    */
    int* getEventBuffer(bool camera);

    /**
    * @brief Function returns the pointer to the buffer that containes timestamps
    * @param indicates whether the required buffer belongs to the left or right camera (LEFT 1, RIGHT 0)
    * @return pointer to the buffer of timestamps
    */
    unsigned long* getTimeBuffer(bool camera);

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
     * @brief return when a wrap around event is occured
     */
    bool getWrapOcc(){
        return wrapOcc;
    }

    /**
    * returns the eldest timestamp
    */
    unsigned long getEldestTimeStamp();

    /**
     * return the valid left
     */
    bool getValidLeft() {return validLeft;};

    /**
     * return the valid left
     */
    bool getValidRight() {return validRight;};

    /**
    * returns the last timestamp for the left camera
    */
    unsigned long getLastTimestamp();

    /**
     *  return the last timestamp for the right camera
     */
    unsigned long getLastTimestampRight();

    /**
    * force the values of the lasttimestamp
    * @param the value to be set
    */
     void setLastTimestamp(unsigned long value);

    /**
     * @brief function thatset the dimension of the output image
     * @param value the dimension in pixels of the retina device
     */
    void setRetinalSize(int value) {
        retinalSize = value;
    }

    /**
     * @brief function that sets the response gradient
     * @param value the dimension of the gradient
     */
    void setResponseGradient(int value) {
        responseGradient = value;
    }

    /**
     * @brief function that set the flag for the ASV chip
     * @param value value to assign to the flag
     */
    void setASVMode(bool value) {
        printf("unmask::setASVMode: setting the value %d \n", value);
        asvMode = value;
    }

    /**
     * @brief function that set the flag for the portable DVS chip
     * @param value value to assign to the flag
     */
    void setDVSMode(bool value) {
        dvsMode = value;
    }

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
    * @param reset reset the timestamp
    */
    void unmaskData(char* data, int size, bool reset);


    /**
    * function that given a reference to bottle of events
    * unmasks any event using the data type protocol and update the buffer
    * @param packets reference to the bottle that contains single events
    */
    void unmaskData(yarp::os::Bottle* packets);

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
    * @brief This method unmasked the raw which come from the TCP socket from unsigned int (16 bits)
    * This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
    * @param *evPU A pointer on the raw casted from char* to int*
    * @param x Set with the x coordinate of the pixel
    * @param y Set with the y coordinate of the pixel
    * @param pol Set with the ON/OFF polarity of the pixel.
    */
    void unmaskEvent(unsigned int evPu, short& x, short& y, short& pol, short& camera);

    /**
    * @brief This method unmasked the raw which come from the TCP socket from unsigned int (16 bits)
    * This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
    * @param *evPU A pointer on the raw casted from char* to int*
    * @param x Set with the x coordinate of the pixel
    * @param y Set with the y coordinate of the pixel
    * @param pol Set with the ON/OFF polarity of the pixel.
    */
    void unmaskEvent(unsigned int evPu, short& x, short& y, short& pol);

    /**
     * function that set to zero the vector of timestamp of positions
     */
    void resetTimestamps();

    /**
     * function that set to zero the vector of timestamp of positions
     */
    void resetTimestampLeft();

    /**
     * function that set to zero the vector of timestamp of positions
     */
    void resetTimestampRight();

    /**
     * @brief fucntion that update the image given the lastTimestamp received
     * @param ptr pointer to the AddressEvent that updates the image
     */
    void updateImage(emorph::ecodec::AddressEvent* ptr);

    /**
     * @brief function that returns the pointer to the HGE Left
     */
    reprCLE* getCLELeft() {

        if(countCLE == 0) {
            return 0;
        }
        _bufferCLELeft--;


        return _bufferCLELeft;

    };

        /**
     * @brief function that returns the pointer to the HGE Left
     */
    reprCLE* getCLERight() {
        if(countCLERight == 0) {
            return 0;
        }
        _bufferCLERight--;

        return _bufferCLERight;
    };

    /**
     * @brief function that returns the pointer to the HGE Left
     */
    reprHGE* getHGELeft() {return _bufferHGELeft;};

    /**
     * @brief function that sets the pointer to the HGE Left
     */
    void setHGELeft() { _bufferHGELeft = bufferHGELeft; };

    /**
     * @brief function that sets the pointer to the CLE Left
     */
    void setCLELeft() { _bufferCLELeft = bufferCLELeft;countCLE =0 ; };

    /**
     * @brief function that sets the pointer to the CLE right
     */
    void setCLERight() { _bufferCLERight  = bufferCLERight ; countCLERight =0 ; };

    /**
     * @brief set the pointer to the original value
     */
    void setOrigHGELeft(reprHGE* b) {_bufferHGELeft = bufferHGELeft = b;};

    /**
     * @brief set the pointer to the original value
     */
    void setOrigCLELeft(reprCLE* b) {_bufferCLELeft = bufferCLELeft = b;};

    /**
     * @brief set the pointer to the original value
     */
    void setOrigCLERight(reprCLE* b) {_bufferCLERight = bufferCLERight = b;};

    /**
     * add the cluster event and all its features
     */
    void addCLE(emorph::ecodec::eEvent* ptr);

    /**
     * add the hough event
     */
    void addHGE(emorph::ecodec::eEvent* ptr);
};

#endif //UNMASK_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

