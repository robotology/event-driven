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
#include <iCub/vsCtrl.h>

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
//    typedef union vsctrl_ioctl_arg_t {
//        struct {
//            uint8_t addr;
//            char rw;
//            uint32_t data;
//        } regs;
//        struct {
//            uint32_t prescaler_value;
//            uint8_t setup_hold_time;
//            uint8_t clock_active_time;
//            uint8_t latch_setup_time;
//            uint8_t latch_active_time;
//        } bg_timings;
//        struct {
//            uint8_t cfg_ack_rel_delay;
//            uint8_t cfg_sample_delay;
//            uint8_t cfg_ack_set_delay;
//        } aer_timings;
//    } vsctrl_ioctl_arg_t;
//
//    
//    typedef struct sp2neu_gen_reg {
//        unsigned int offset;
//        char         rw;
//        unsigned int data;
//    } sp2neu_gen_reg_t;
    
    
    

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
    bool bufferedRead;
    unsigned int readCount;
    unsigned int maxBufferSize;

    std::vector<char> *readBuffer;
    std::vector<char> *accessBuffer;
    std::vector<char> buffer1;
    std::vector<char> buffer2;

    yarp::os::Semaphore safety;
    virtual void run();

    // vsctrl
    vsctrl_ioctl_arg_t iocVsctrlArg;

    
public:

    fpgaStatus_t *fpgaStat;
    
    deviceManager(std::string deviceName, bool bufferedRead = false, unsigned int maxBufferSize = 16777216);

    bool openDevice();
    void closeDevice();
    //int readDevice(std::vector<unsigned int> &deviceData);
    int writeDevice(std::vector<unsigned int> &deviceData);

    const std::vector<char>& readDevice(int &nBytesRead);
    
    // ---- ioctl for i2c device ---- //
    int chipReset();
    int chipPowerDown();
    int chipPowerUp();
    
    int getFpgaStatus();
    int clearFpgaStatus(std::string clr);
    int getFpgaRel();
    int getFpgaInfo();
    int writeAerTimings(uint8_t ack_rel, uint8_t sample, uint8_t ack_set);
    
    int writeBgTimings(uint8_t prescaler, uint8_t hold, uint8_t ck_active, uint8_t latch_setup, uint8_t latch_active);
    int getBgTimings();
    int getAerTimings();
    int initDevice(std::string chipName);
    int writeGPORegister(uint32_t data);
    int readGPORegister();

};

#endif /* defined(__eMorph__deviceManager__) */
