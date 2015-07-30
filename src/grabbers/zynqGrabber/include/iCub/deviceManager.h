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

class deviceManager{
    
public:
    deviceManager(std::string deviceName);

    typedef struct sp2neu_gen_reg {
        unsigned int offset;
        char         rw;
        unsigned int data;
    } sp2neu_gen_reg_t;

    int             devDesc;                      // file descriptor for device
    //int             devData;                        // return of the read/write functions
    //std::string     portDeviceName;                 // name of the device which the module will connect to
    std::string     deviceName;                     // name of the device


    void write_generic_sp2neu_reg (int devDesc, unsigned int offset, unsigned int data);
    unsigned int read_generic_sp2neu_reg (int devDesc, unsigned int offset);
    void usage (void);
    
    /**
     * function that sets the device name
     */
    //void setDeviceName(std::string name);
    
    /**
     * function that correcly closes the device
     */
    void closeDevice();

    /**
     * function that correcly opens the device
     */
    bool openDevice();
    
    int readDevice(std::vector<unsigned int> &deviceData);
    int writeDevice(std::vector<unsigned int> &deviceData);

    bool readFifoFull();
    bool readFifoEmpty();
    bool writeFifoAFull();
    bool writeFifoFull();
    bool writeFifoEmpty();
    int timeWrapCount();

};

#endif /* defined(__eMorph__deviceManager__) */
