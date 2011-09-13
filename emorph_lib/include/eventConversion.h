// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Francesco Rea, Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __evenConversionh__
#define __evenConversionh__


#include <iostream>
#include <sstream>
#include <ctime>

#include <yarp/os/all.h>

typedef struct s_AER_struct {
    int x;
    int y;
    int pol;
    unsigned int ts;
}AER_struct;


/**
 * \section change_log CHANCE LOG
 * 22/08/11 : made the unmasking class as general as possible                               \author Rea \n
 * 22/08/11 : added two functions for timestamp reset                                       \author Rea \n
 * 23/08/11 : added different unmasking for dvs cameras without iHead                       \author Rea \n   
 * 13/09/11 : added an pointer to the unmasked  for the method unmaskData                   \author Rea \n         
 */


/** 
 * a simple class to handle event unmasking.
 */
class unmask {    ///: public yarp::os::RateThread {
private:
    int id;
    int nb_trame;
    int count;                       // counter of the unmasked events
    
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
    unsigned long lasttimestamp;          //last timestamp acquired for the left camera
    unsigned long lasttimestampright;     //last timestamp acquired for the right camera
    unsigned long eldesttimestamp;        // timestamp of the eldest event in the buffer 
    short cartX, cartY, polarity, camera;

    int wrapAdd;
    unsigned int xmask;            // 16 bits mask for unmasking of the address
    unsigned int ymask;            // 16 bits mask for unmasking of the address
    unsigned int xmaskshort;       // 16 bits mask for unmasking of the address
    unsigned int ymaskshort;       // 16 bits mask for unmasking of the address
    unsigned int polmaskshort;     // 16 bits mask for unmasking of the address
    long int xmasklong;            // 32 bits mask for unmasking of the event
    long int ymasklong;            // 32 bits mask for unmasking of the event
    int yshift;                    // shift of 8 bits for getting the y in 16 bits address
    int xshift;                    // shift of 1 bit to get the x in 16 bit (polarity)
    int yshift2;                   // shift of 16 bits for getting the y in 32 bits address
    int polshift;                  // shift necessary to cast the unmasked value to short
    int polmask;                   // mask necessary to extract the polarity
    int camerashift;               // shift for cast back into short the camera reference 
    int cameramask;                // mask necessary to extract which camera has produced the output
    int retinalSize;               // size of the retina
    int minValue;
    int maxValue;
    int countEvent;                // counter of the number of events saved in the buffer1
    int countEvent2;               // counter of the number of events saved in the buffer2
    int numKilledEvents;           // number of the element that have be removed from the buffer
    bool temp1;                    // boolean flag that indicates where the events have to be saved
    bool validLeft,validRight;     // flag for validity of the events
    bool verbosity;
    bool dvsMode;                  // flag that initialises the typology of unmasking

    FILE* uEvents;
    yarp::os::Semaphore countEventLocker;
    yarp::os::Semaphore countEventLocker2;

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
    unmask();

    /**
    * destructor
    */
    ~unmask();

    /**
    * function that initialise the thread
    */
    //bool threadInit();

    /**
    * function called when the thread is stopped
    */
    //void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    //void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    //void interrupt();

    /**
    * function that given a reference to the list of char read from the port and the number of packet received
    * unmasks the event in term of x,y, polarity and time stamp and update the buffer
    * @param data reference to the vector of char (the read data)
    * @param size size of the last reading from the port
    * @param output pointer to the collection of event unmasked
    */
    void unmaskData(char* data, int size, AER_struct* output);

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
     * function that set to zero the vector of timestamp  (left)
     */
    void resetTimestampLeft();
    
    /**
     * function that set to zero the vector of timestamp  (right)
     */
    void resetTimestampRight();
};

#endif /* __evenConversionh__ */

//----- end-of-file --- ( next line intentionally left blank ) ------------------
