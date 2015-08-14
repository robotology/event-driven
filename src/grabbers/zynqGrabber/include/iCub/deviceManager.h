//
//  deviceManager.h
//  eMorph
//
//  Created by Chiara Bartolozzi on 30/07/15.
//
//

#ifndef __eMorph__deviceManager__
#define __eMorph__deviceManager__

#include <stdio.h>
#include <string>
#include <vector>
#include <yarp/os/all.h>
#include <fstream>

#define DEBUG
/**
 * @brief The deviceManager class opens the device and implements a constant
 * read
 */
class deviceManager : yarp::os::Thread {

private:

    //device member variables
    int devDesc;            // file descriptor for device
    std::string deviceName; // name of the device
#ifdef DEBUG
    std::ofstream writeDump;
    std::ofstream readDump;
#endif

    //hardware specific
    typedef struct sp2neu_gen_reg {
        unsigned int offset;
        char         rw;
        unsigned int data;
    } sp2neu_gen_reg_t;

    void write_generic_sp2neu_reg (int devDesc, unsigned int offset,
                                   unsigned int data);
    unsigned int read_generic_sp2neu_reg (int devDesc, unsigned int offset);
    void usage (void);

    bool readFifoFull();
    bool readFifoEmpty();
    bool writeFifoAFull();
    bool writeFifoFull();
    bool writeFifoEmpty();
    int timeWrapCount();


    //read thread related functions and variables
    unsigned int readCount;
    unsigned int maxBufferSize;
    std::vector<char> readBuffer;
    std::vector<char> accessBuffer;
    yarp::os::Semaphore safety;
    virtual void run();

    
public:

    deviceManager(std::string deviceName, unsigned int maxBufferSize);

    bool openDevice();
    void closeDevice();
    //int readDevice(std::vector<unsigned int> &deviceData);
    int writeDevice(std::vector<unsigned int> &deviceData);

    std::vector<char> &readDevice();

};

#endif /* defined(__eMorph__deviceManager__) */
