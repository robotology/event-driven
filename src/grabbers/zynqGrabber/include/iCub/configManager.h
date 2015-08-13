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

// --- for aex2 --- //

struct aer {
    uint32_t timestamp;
    uint32_t address;
};

#define SIZE_OF_EVENT 8192


class configManager{
    
    configManager(std::string channel, std::string device);
    
    std::string device;
    std::string channel;
    std::map<std::string, unsigned int> mBiases;
    unsigned int header;
    std::vector<std::string> biasNames; // ordered
    std::vector<int> biasValues;
    
    bool prepareHeader(); // prepares the header with device and channel
    
    bool setBias(std::string biasName, unsigned int biasValue);

    unsigned int getBias(std::string biasName);
    void printBiases();
    std::vector<unsigned int> prepareBiases();
    std::vector<unsigned int> prepareReset(bool value);

    
    // ---- for aex2 ---- //
    struct aer *pseq;                               // pointer to the sequence of events
    uint32_t seqAlloced_b, seqSize_b, seqEvents, seqDone_b;;
    void prepareBiasesAex();
    uint64_t seqTime;
    void progBiasAex(std::string name,int bits,int value, int camera);
    void latchCommitAEs(int camera );
    void progBitAEs(int bitvalue, int camera );
    void biasprogtx(int time,int latch,int clock,int data, int powerdown, int camera );
    void releasePowerdown(int camera );
    void setPowerdown(int camera );
    // void sendingBias();
};




#endif /* defined(__eMorph__configManager__) */
