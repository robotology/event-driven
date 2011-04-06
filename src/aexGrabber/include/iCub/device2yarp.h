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
#include <yarp/os/Semaphore.h>

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
    * function that prepares biases either reading them from file or reading the default value
    */
    void prepareBiases();

    /**
    * function that prepares biases either reading them from file or reading the default value
    */
    void prepareBiasesRight();

    /**
    * function used to set the name of the port device where biases are sent
    * @param name name of the port of the device
    */
    void setDeviceName(std::string name);

    /**
     * function that correcly closes the device
     */
    void closeDevice();
    
    /**
    * function used to append to a list of commands every single bit necessary to program bias
    * @param name name of the bias to be set
    * @param bits data of the programming
    * @param value value of the programming
    * @camera reference to the camera (left 1, right 0)
    */
    void progBias(std::string name, int bits, int value, int camera = 1);

    /**
    * function used to append to a list of bits necessary to send the latch commit
    * @camera reference to the camera (left 1, right 0)
    */
    void latchCommit(int camera = 1);

    /**
    * function used to append to a list of bits necessary to send the latch commit (version 2)
    * @camera reference to the camera (left 1, right 0)
    */
    void latchCommitAEs(int camera = 1);

    /**
    * function used to reset the device to default after a bias is sent
    * @camera reference to the camera (left 1, right 0)
    */
    void resetPins(int camera = 1);

    /**
    * function that sends the powerdown to the correct value after the biases have been programmed
    * @camera reference to the camera (left 1, right 0)
    */
    void releasePowerdown(int camera = 1);

    /**
    * function that sends the powerdown to the correct value before switching the device off
    * @camera reference to the camera (left 1, right 0)
    */
    void setPowerdown(int camera = 1);

    /**
    * correct sequence of signals necessary to program a bit
    * @param bitvalue value of the bit to be programmed
    * @camera reference to the camera (left 1, right 0)
    */
    void progBit(int bitvalue, int camera = 1);

    /**
    * correct sequence of signals necessary to program a bit (version 2)
    * @param bitvalue value of the bit to be programmed
    * @camera reference to the camera (left 1, right 0)
    */
    void progBitAEs(int bitvalue, int camera = 1);

    /**
    * function that send biases to FPGA.The FPGA then first waits for the sequencer time to expire as for every other AE, then the lowest four bits of the AE,
    * RIGHT/LEFT in 0xFF0000RL would be interpreted as new values for the bias programming pins.
    * @param time sequence time to expire before programming data
    * @param latch control of the latch pin
    * @param clock control of the clock
    * @param data data that is sent
    * @param powerdown control of the powerdown
    * @param camera defines which camera is going to be programmed (Left 1, Right 0)
    */
    void biasprogtx(int time, int latch, int clock, int data, int powerdown = 1, int camera = 1);
    

    /** 
     * function that monitors the output for seconds
     * @param secs number of seconds to wait
     * @camera reference to the camera (left 1, right 0)
     */
    void monitor(int secs, int camera = 1);

    /**
     * fuction that connects to the device and write the sequence of signals to the device FPGA
     * no matter what camera has been set the biases go to the board
     */
    void sendingBias();

    /**
    * set the flag that regulates whether the biases are read from binary or copied from default values
    */
    void setFromBinary(bool value) { biasFromBinary = value; };

    /**
     * function that sets the reference of the input binary file
     * @param f pointer to the FILE of the binary
     */      
    void setBinaryFile(FILE* f) {binInput = f; };

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setPR(double value) {pr = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setFOLL(double value) {foll = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setDIFF(double value) {diff = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setDIFFON(double value) {diffon = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setPUY(double value) {puy = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setREFR(double value) {refr = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setREQ(double value) {req = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setDIFFOFF(double value) {diffoff = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setPUX(double value) {pux = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setREQPD(double value) {reqPd = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setINJGND(double value) {injg = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCAS(double value) {cas = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setPRRight(double value) {prRight = value;};
    
    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setFOLLRight(double value) {follRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setDIFFRight(double value) {diffRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setDIFFONRight(double value) {diffonRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setPUYRight(double value) {puyRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setREFRRight(double value) {refrRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setREQRight(double value) {reqRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setDIFFOFFRight(double value) {diffoffRight = value;};
    
    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setPUXRight(double value) {puxRight = value;};
    
    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setREQPDRight(double value) {reqPdRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setINJGNDRight(double value) {injgRight = value;};

    /**
    * function that sets the bias Right
    * @param value value of the bias
    */
    void setCASRight(double value) {casRight = value;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getPR() {return pr ;};
    
    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getFOLL() {return foll ;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getDIFF() {return diff;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getDIFFON() {return diffon;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getPUY() {return puy;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getREFR() {return refr ;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getREQ() {return req ;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getDIFFOFF() {return diffoff;};
    
    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getPUX() {return pux;};
    
    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getREQPD() {return reqPd;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getINJGND() {return injg;};

    /**
    * function that returns the bias for Left camera
    * @return value of the bias
    */
    double getCAS() {return cas;};


private:

    yarp::os::BufferedPort<sendingBuffer> port;
    int r;                                           //dimension of the received buffer of event for display
    FILE* raw;
    FILE* binInput;
    bool biasFromBinary;
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
    std::string portDeviceName;              // name of the device which the module will connect to
    std::string biasFileName;                // name of the file that contains the biases

    bool save;

    yarp::os::Port interfacePort;             //port dedicated to the request of values set through interface
    yarp::os::Semaphore mutex;                 //semaphore for file reading

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

    int prRight;
    int follRight;
    int diffRight;
    int diffonRight;
    int puyRight;
    int refrRight;
    int reqPdRight;
    int diffoffRight;
    int puxRight;
    int reqRight;
    int injgRight;
    int casRight;

};

#endif //_DEVICE2YARP_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

