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
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdint.h>

#include "sending_buffer.h"

#define u8  uint8_t
#define u32 uint32_t
#define u64 uint64_t

struct aer {
    u32 timestamp;
    u32 address;
};

class device2yarp : public yarp::os::RateThread {
public:
    device2yarp(std::string deviceNumber, bool save, std::string filename);
    ~device2yarp();
    virtual void run();
    virtual void threadRelease();

    /**
    * function used to set the name of the port device where biases are sent
    * @param name name of the port of the device
    */
    void setDeviceName(std::string name);
    
    /**
    * function used to append to a list of commands every single bit necessary to program bias
    */
    void progBias(std::string name, int bits, int value);

    /**
    * function used to append to a list of bits necessary to send the latch commit
    */
    void latchCommit();

    /**
    * function used to append to a list of bits necessary to send the latch commit (version 2)
    */
    void latchCommitAEs();

    /**
    * function used to reset the device to default after a bias is sent
    */
    void resetPins();

    /**
    * function that sends the powerdown to the correct value after the biases have been programmed
    */
    void releasePowerdown();

    /**
    * function that sends the powerdown to the correct value before switching the device off
    */
    void setPowerdown();

    /**
    * correct sequence of signals necessary to program a bit
    * @param bitvalue value of the bit to be programmed
    */
    void progBit(int bitvalue);

    /**
    * correct sequence of signals necessary to program a bit (version 2)
    * @param bitvalue value of the bit to be programmed
    */
    void progBitAEs(int bitvalue);

    
    void biasprogtx(int time, int latch, int clock, int data, int powerdown = 1);
    

    /** 
     * function that monitors the output for seconds
     * @param secs number of seconds to wait
     */
    void monitor(int secs);

    /**
     * fuction that connects to the device and write the sequence of signals to the device
     */
    void sendingBias();

    void setPR(double value) {pr = value;};
    
    void setFOLL(double value) {foll = value;};

    void setDIFF(double value) {diff = value;};

    void setDIFFON(double value) {diffon = value;};

    void setPUY(double value) {puy = value;};

    void setREFR(double value) {refr = value;};

    void setREQ(double value) {req = value;};

    void setDIFFOFF(double value) {diffoff = value;};
    
    void setPUX(double value) {pux = value;};
    
    void setREQPD(double value) {reqPd = value;};

    void setINJGND(double value) {injg = value;};

    void setCAS(double value) {cas = value;};


    double getPR() {return pr ;};
    
    double getFOLL() {return foll ;};

    double getDIFF() {return diff;};

    double getDIFFON() {return diffon;};

    double getPUY() {return puy;};

    double getREFR() {return refr ;};

    double getREQ() {return req ;};

    double getDIFFOFF() {return diffoff;};
    
    double getPUX() {return pux;};
    
    double getREQPD() {return reqPd;};

    double getINJGND() {return injg;};

    double getCAS() {return cas;};



private:

    yarp::os::BufferedPort<sendingBuffer> port;
    FILE* raw;

    u64 seqTime;
    u64 ec;
    u32 seqAlloced_b;
    u32 seqSize_b, seqEvents, seqDone_b;
    u32 monBufEvents;
    u32 monBufSize_b;
    struct aer *pseq;
    struct aer *pmon;

    int file_desc,len,sz;
    unsigned char buffer_msg[64];
    short enabled;
    char buffer[SIZE_OF_DATA];

    int err;
    unsigned int timestamp;
    short cartX, cartY, polarity;

    unsigned int xmask;
    unsigned int ymask;
    int yshift;
    int xshift;
    int polshift;
    int polmask;
    int retinalSize;
    std::string portDeviceName;

    bool save;

    yarp::os::Port interfacePort;             //port dedicated to the request of values set through interface

    std::stringstream str_buf;

    int pr;
    int foll;
    int diff;
    int diffon;
    int puy;
    int refr;
    int reqPd;
    int diffoff;
    int pux;
    int req;
    int injg;
    int cas;
};

#endif //_DEVICE2YARP_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

