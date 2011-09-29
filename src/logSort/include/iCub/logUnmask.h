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
 * @file logUnmask.h
 * @brief code for unmasking the events
 */

#ifndef LOG_UNMASK_H
#define LOG_UNMASK_H

#include <iostream>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <inttypes.h>
#include <sys/types.h>
#include <cassert>

// yarp includes
#include <yarp/os/all.h>

//typedef unsigned long uint32_t;


#define u32 uint32_t

/*struct aer {
    u32 timestamp;
    u32 address;
};
*/

struct aer {
    u32 address;
    u32 timestamp;
};

//Other dependency
#include <iCub/config.h>

typedef short feature[4];

class logUnmask : public yarp::os::RateThread{
private:
    int id;
    int nb_trame;
    int count;                            // counter of the unmasked events
    FILE* fout;                           // file for dumping the events out
    int sz;
    int* buffer;                          // buffer representing the event in image plane (left)
    unsigned long* timeBuffer;            // buffer contains the timestamp of the particular location (left)
    int* bufferRight;                     // buffer representing the event in image plane (right)
    unsigned long* timeBufferRight;       // buffer contains the timestamp of the particular location (right)
    u32 monBufSize_b;                     // dimension of the event buffers
    int countCD;                          // dimension of the event buffers
    int countEM1;                         // dimension of the event buffers
    int countEM2;                         // dimension of the event buffers
    int countEM3;                         // dimension of the event buffers
    int countEM4;                         // dimension of the event buffers
    int countIF;                          // dimension of the event buffers
    struct aer* bufferCD;                 // buffer for change detector
    struct aer* bufferIF;                 // buffer for integrate and fire
    struct aer* bufferEM1;                // buffer for EM1
    struct aer* bufferEM2;                // buffer for EM2
    struct aer* bufferEM3;                // buffer for EM3
    struct aer* bufferEM4;                // buffer for EM4
    unsigned long* cartEM;                 // mean value across EMs in cartesian space

    //int* fifoEvent;
    //int* fifoEvent_temp;
    //int* fifoEvent_temp2;

    unsigned long previous_timestamp;     // timestamp for the previous event
    unsigned long timestamp;              // 16 bits variable to save the timestamp
    unsigned long timestamplong;          // variable 32 bits to save the timestamp
    unsigned long lasttimestamp;          // last timestamp acquired for the left camera
    unsigned long lasttimestampright;     // last timestamp acquired for the right camera
    unsigned long eldesttimestamp;        // timestamp of the eldest event in the buffer 
    short cartX, cartY, polarity, type;
    int maxx, maxy;

    int wrapAdd;
    unsigned short xmask;              // 16 bits mask for unmasking of the address
    unsigned short ymask;              // 16 bits mask for unmasking of the address
    long int xmasklong;                // 32 bits mask for unmasking of the event
    long int ymasklong;                // 32 bits mask for unmasking of the event
    short yshift;                      // shift of 8 bits for getting the y in 16 bits address
    short xshift;                      // shift of 1 bit to get the x in 16 bit (polarity)
    short yshift2;                     // shift of 16 bits for getting the y in 32 bits address
    short polshift;                    // shift necessary to cast the unmasked value to short
    int polmask;                       // mask necessary to extract the polarity
    short camerashift;                 // shift for cast back into short the camera reference 
    int cameramask;                    // mask necessary to extract which camera has produced the output
    int retinalSize;                   // size of the retina
    int minValue;
    int maxValue;
    int countEvent;                     //counter of the number of events saved in the buffer1
    int countEvent2;                    //counter of the number of events saved in the buffer2
    int numKilledEvents;                //number of the element that have be removed from the buffer
    bool temp1;                         //boolean flag that indicates where the events have to be saved
    bool validLeft,validRight;          //flag for validity of the events
    bool verb;
    FILE* uEvents;
    yarp::os::Semaphore countEventLocker;
    yarp::os::Semaphore countEventLocker2;

    feature* logChip_LUT;

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
    * default constructor
    */
    logUnmask();

    /**
    * destructor
    */
    ~logUnmask();

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
    void logUnmaskData(char* data, int size, bool reset);

    /**
    * function that given a reference to the list of long int(32 bits) read from the port and the number of packet received
    * unmasks the event in term of x,y, polarity and time stamp and update the buffer
    * @param evPU blob representing the position of the pixel the polarity and the camera produced from
    * @param x position of the event along the x axis
    * @param y position of the event along the y axis
    * @param pol polarity of the event +1, -1
    * @param camera reference to the camera that has produced the event
    */
    void logUnmaskEvent(unsigned long evPU, short& x, short& y, short& pol, short& camera);

    /**
    * @brief This method unmasked the raw which come from the TCP socket
    * This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
    * @param *evPU A pointer on the raw casted from char* to int*
    * @param x Set with the x coordinate of the pixel
    * @param y Set with the y coordinate of the pixel
    * @param pol Set with the ON/OFF polarity of the pixel.
    */
    void logUnmaskEvent(unsigned int evPu, short& x, short& y, short& pol, short& camera);

    /**
     * @brief masking the coordinates into a 32bits address 
     */
    void logMaskEvent(short metax, short metay, short pol, unsigned long & evPU);

    /**
     * @brief function that returns the pointer to the buffer of CHANGE DETECTOR EVENT
     * @param pointerCD char* pointer to the beginning of the buffer
     * @param dimCD number of the events counted in the buffer
     */     
    void getCD(aer** pointerCD, int* dimCD);

    /**
     * @brief function that returns the pointer to the buffer of Integrate & Fire
     * @param pointerIF char* pointer to the beginning of the buffer
     * @param dimIF number of the events counted in the buffer
     */     
    void getIF(aer** pointerIF, int* dimIF);

    /**
     * @brief function that returns the pointer to the buffer of EXPOSURE MEASURE
     * @param pointerEM char* pointer to the beginning of the buffer
     * @param dimEM number of the events counted in the buffer
     */     
    void getEM(aer** pointerEM, int* dimEM);
    
    /**
     * @brief function that resets the counter of CDs
     */     
    void resetCD() {countCD = 0; };

    /**
     * @brief function that resets the counter of IFs
     */     
    void resetIF() {countIF = 0; };

    /**
     * @brief function that resets the counter of EMs
     */     
    void resetEM1() {countEM1 = 0; };

    /**
     * @brief function that resets the counter of EMs
     */     
    void resetEM2() {countEM2 = 0; };

        /**
     * @brief function that resets the counter of EMs
     */     
    void resetEM3() {countEM3 = 0; };

    /**
     * @brief function that resets the counter of EMs
     */     
    void resetEM4() {countEM4 = 0; };

    /**
     * @brief function that resets the counter of EMs
     */     
    void resetTOTEM() {memset(cartEM, 0, retinalSize * retinalSize * sizeof(unsigned long)); };

    
    /**
     * @brief function that returns the pointer to the buffer of INTEGRATE 'n' FIRE EVENT
     * @param pointerIF char* pointer to the beginning of the buffer
     * @param dimIF number of the events counted in the buffer
     */   
    void getIF(aer* pointerIF, int* dimIF);

    /** 
     * @brief function that extract the timestamp value of the opposite blob
     * @param buffer buffer in which the search is carried out
     * @param position of the original blob. Search is carried out after
     * @param number of event saved in the buffer
     */
    unsigned long look4opposite(aer* buffer,int initPos, int countTOT);       
    

    /**
     * function that set to zero the vector of timestamp of positions
     */
    void resetTimestamps();
};

#endif //LOG_UNMASK_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

