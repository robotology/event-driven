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
 * @file asvGrabberThread.h
 * @brief Definition of the ratethread that receives events from DVS camera
 */

#ifndef _ASV_GRABBER_THREAD_H
#define _ASV_GRABBER_THREAD_H

//yarp include
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <fstream>

#include <iCub/emorph/eventBuffer.h>

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t

#define SIZE_OF_EVENT 8192
#define SIZE_OF_DATA 65536

struct aer {
    u32 timestamp;
    u32 address;
};

class asvGrabberThread : public yarp::os::RateThread {
public:
    asvGrabberThread(std::string deviceNumber, bool save, std::string filename);
    ~asvGrabberThread();
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
    * function used to set the path of the working directory
    * @param value working directory path to be set
    */
    void setWorkingDirectory(std::string value) { workingDirectory = value; };

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
    * indicates whether the event has to be saved in a file
    */
    void setDumpEvent(bool value);

    /**
    * reference to the name of the file where events are dumped
    */
    bool setDumpFile(std::string value);

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynThr(double value) {SynThr = value;};

    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynTau(double value) {SynTau = value; printf("tau %f \n",value);};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynPxlTau(double value) {SynPxlTau = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynPxlThr(double value) {SynPxlThr = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setTestPbias(double value) {testPbias = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDRefr(double value) {CDRefr= value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDSf(double value) {CDSf = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDPr(double value) {CDPr = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setReqPuX(double value) {ReqPuX = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setReqPuY(double value) {ReqPuY = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setIFRf(double value) {IFRf = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setIFThr(double value) {IFThr = value;};


    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setIFLk(double value) {IFLk = value;};


    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDOffThr(double value) {CDOffThr = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynPxlW(double value) {SynPxlW = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSynW(double value) {SynW = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDOnThr(double value) {CDOnThr = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDDiff(double value) {CDDiff = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setEMCompH(double value) {EMCompH = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setEMCompT(double value) {EMCompT = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDIoff(double value) {CDIoff = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDRGnd(double value) {CDRGnd = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setSelf(double value) {self = value;};
    
    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setFollBias(double value) {FollBias = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setArbPd(double value) {ArbPd = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setEMVrefL(double value) {EMVrefL = value;};

    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setCDCas(double value) {CDCas = value;};


    /**
    * function that sets the bias
    * @param value value of the bias
    */
    void setEMVrefH(double value) {EMVrefH = value;};



    //***************************************************************************
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynThr() {return SynThr ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynTau() {return SynTau ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynPxlTau() {return SynPxlTau ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynPxlThr() {return SynPxlThr ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getTestPBias() {return testPbias ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDRefr() {return CDRefr ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDSf() {return CDSf ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDPr() {return CDPr ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getReqPuX() {return ReqPuX ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getReqPuY() {return ReqPuY ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getIFRf() {return IFRf ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getIFThr() {return IFThr ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getIFLk() {return IFLk ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDOffThr() {return CDOffThr; };
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynPxlW() {return SynPxlW; };

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSynW() {return SynW; };
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDOnThr() {return CDOnThr; };

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDDiff() {return CDDiff ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getEMCompH() {return EMCompH ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getEMCompT() {return EMCompT ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDIoff() {return CDIoff ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDRGnd() {return CDRGnd ;};

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getSelf() {return self ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getFollBias() {return FollBias ;};
    
    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getArbPd() {return ArbPd ;};


    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getEMVrefL() {return EMVrefL ;};
    

    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getCDCas() {return CDCas ;};


    /**
    * function that returns the bias 
    * @return value of the bias
    */
    double getEMVrefH() {return EMVrefH ;};



private:
    yarp::os::BufferedPort<eventBuffer> port;              //port sending events
    yarp::os::BufferedPort<yarp::os::Bottle> portDimension;  //port sending dimension of packets   
    int r;                                                   //dimension of the received buffer of event for display
    FILE* raw;
    FILE* binInput;
    FILE* fout;                                              // file for dumping the events out
    bool biasFromBinary;                                     // indicates whether the bias programmed are read from a file
    u64 seqTime;
    u64 ec;
    u32 seqAlloced_b;
    u32 seqSize_b, seqEvents, seqDone_b;
    u32 monBufEvents;
    u32 monBufSize_b;
    struct aer *pseq;                               //pointer to the sequence of events 
    struct aer *pmon;                               //pointer to the sequence of events (monitor)

    int file_desc,len,sz;
    unsigned char buffer_msg[64];
    short enabled;
    char buffer[SIZE_OF_DATA];                      // buffer of char to be set on the outport

    int err;
    unsigned int timestamp;
    unsigned long previous_timestamp;       // timestamp for the previous event
    unsigned long lasttimestamp;            // latest timestamp
    unsigned long previous_blob;            // address for the previous address
    short cartX, cartY, polarity;

    unsigned int xmask;                     // mask for extracting the x position
    unsigned int ymask;                     // mask for extracting the y position
    int yshift;                             // mask for extracting the x position
    int xshift;                             // mask for extracting the y position
    int polshift;                           // shift to determine polarity
    int polmask;                            // mask to determine polarity
    int retinalSize;                        // dimension of the retina
    std::string portDeviceName;             // name of the device which the module will connect to
    std::string biasFileName;               // name of the file that contains the biases
    std::string dumpfile;                   // name of the file where events are going to be dumped
    std::string workingDirectory;           // path where the files will be saved
    

    bool save;                              // bool that indicates whether the 

    yarp::os::Port interfacePort;           // port dedicated to the request of values set through interface
    yarp::os::Semaphore mutex;              // semaphore for file reading
    double startTime;                       // time of the programming
    double stopTime;                        // stop time for the timing
    std::stringstream str_buf;              // reference to the object for output file stream 
                              
    int SynThr;                             // value programmed for the bias    
    int SynTau;                             // value programmed for the bias    
    int SynPxlTau;                          // value programmed for the bias
    int SynPxlThr;                          // value programmed for the bias
    int CDRefr;                             // value programmed for the bias                                // value programmed for the bias
    int CDSf;                               // value programmed for the bias
    int CDPr;                             // value programmed for the bias
    int ReqPuX;                             // value programmed for the bias
    int ReqPuY;                             // value programmed for the bias
    int IFRf;                             // value programmed for the bias
    int IFThr;                                  // value programmed for the bias
    int IFLk;                             // value programmed for the bias
    int CDOffThr;                             // value programmed for the bias
    int SynPxlW;                             // value programmed for the bias
    int testPbias;                             // value programmed for the bias
    int SynW;                             // value programmed for the bias
    int CDOnThr;                             // value programmed for the bias
    int CDDiff;                             // value programmed for the bias
    int EMCompH;                             // value programmed for the bias
    int EMCompT;                             // value programmed for the bias
    int CDIoff;                             // value programmed for the bias
    int CDRGnd;                             // value programmed for the bias                             // value programmed for the bias
    int self;                             // value programmed for the bias
    int FollBias;                             // value programmed for the bias
    int ArbPd;                             // value programmed for the bias
    int EMVrefL;                             // value programmed for the bias
    int CDCas;                             // value programmed for the bias
    int EMVrefH;                             // value programmed for the bias
    int I2V;                             // value programmed for the bias
};

#endif //_ASV_GRABBER_THREAD_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

