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
 * @file device2yarp.h
 * @brief Definition of the ratethread that receives events from DVS camera
 */

#ifndef _DEVICE2YARP_H
#define _DEVICE2YARP_H

//yarp include
//#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <unistd.h>
#include <cmath>

#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventCodec.h>
#include "config.h"

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t

struct aer {
    u16 timestamp;
    u16 address;
};


//class device2yarp : public yarp::os::RateThread {
class device2yarp : public yarp::os::Thread {
public:
    device2yarp(std::string deviceNumber, bool save, std::string filename);
    
    ~device2yarp();
    
    /**
     * main active function of the thread
     */
    virtual void run();
    
    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function used to set the name of the port device where biases are sent
    * @param name name of the port of the device
    */
    void setDeviceName(std::string name);

    /**
     * @brief function that sets verbosity parameter
     * @param value value to assign
     */
    void setVerbosity(bool value) { verbosity = value; };

     /**
     * @brief function that sets the incremental number of the port device for multiple devices
     * @param value value to assign
     */
    void setDeviceNumber(int value) { deviceNum = value; };

private:

    yarp::os::BufferedPort<emorph::ebuffer::eventBuffer> port;
    //yarp::os::BufferedPort<yarp::os::Bottle > port;
    FILE* raw;

    u64 seqTime;
    u64 ec;
    u32 monBufEvents;
    u32 monBufSize_b;

    int deviceNum;                               // number to append to the port name for multiple devices
    int file_desc,len,sz;
    unsigned char buffer_msg[64];
    short enabled;
    char buffer[SIZE_OF_DATA];
    char outbuffer[SIZE_OF_DATA];
    //u32 newBuffer[SIZE_OF_DATA];
    yarp::os::Bottle packetsSent;
    FILE* tmpFile;
    struct aer *pmon;
    int err;
    int numberOfTimeWraps;                       // 14 are bits used for timestamp, 26 are bits at our disposal, so actual time = 2^14 * #timeWraps + timeStamp, hence this counter tracks till 2^12
    unsigned int timestamp;
    unsigned int ptimestamp;
    unsigned int wrapAround;
    short wrapOccured;
    short cartX, cartY, polarity;

    unsigned int xmask;
    unsigned int ymask;
    int yshift;
    int xshift;
    int polshift;
    int polmask;
    int retinalSize;
    std::string portDeviceName;
    std::string i_fileName;                      // name of the file where to save
    
    bool save;
    bool verbosity;                              // flag that indicates whether to write events out

    std::stringstream str_buf;
    emorph::ecodec::AddressEvent adEv;
    emorph::ecodec::TimeStamp tmSt;
};

#endif //_DEVICE2YARP_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

