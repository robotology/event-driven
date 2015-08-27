//
//  configManager.h
//  eMorph
//
//  Created by Chiara Bartolozzi on 12/08/15.
//
//

#ifndef __eMorph__configManager__
#define __eMorph__configManager__

#include <stdio.h>

#include <string>
#include <vector>
#include <map>
#include <iCub/deviceManager.h>
#include <inttypes.h>
// --- for aex2 --- //

struct aer {
    uint32_t timestamp;
    uint32_t address;
};

#define SIZE_OF_EVENT 8192


class configManager{
    
    deviceManager* devManager;
    
    std::string chip;
    std::string channel;
    std::map<std::string, unsigned int> mBiases;
    unsigned int header;
    std::vector<std::string> biasNames; // ordered
    std::vector<int> biasValues;
    
    //bool prepareHeader(); // prepares the header with chip and channel
    
    
    unsigned int getBias(std::string biasName);
    void printBiases();
    std::vector<unsigned int> prepareBiases();
    
    
    
    
    
    //std::vector<unsigned int> prepareReset(bool value);
    
    
    // ---- for aex2 ---- //
    struct aer *pseq;                               // pointer to the sequence of events
    uint32_t seqAlloced_b, seqSize_b, seqEvents;
    std::vector<unsigned int> prepareBiasesAex();
    uint64_t seqTime;
    int camera;
    void progBiasAex(std::string name,int bits,int value);
    void latchCommitAEs();
    void progBitAEs(int bitvalue );
    void biasprogtx(int time,int latch,int clock,int data, int powerdown);
    void releasePowerdown();
    void setPowerdown();
    // void sendingBias();
    
public:
    configManager(std::string channel, std::string chip);
    void attachDeviceManager(deviceManager* devManager);
    bool programBiases();
    bool programBiasesAex();
    bool setBias(std::string biasName, unsigned int biasValue);
    int chipReset();
    int chipPowerDown();
    int chipPowerUp();
    
    int getFpgaStatus();
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




#endif /* defined(__eMorph__configManager__) */
