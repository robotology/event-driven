//
//  configManager.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 12/08/15.
//
//

#include "iCub/configManager.h"
#include <string>
#include <iostream>
#include <inttypes.h>

#define timestep 1000 //10 us OneSecond/1000
//#define OneSecond 1000000 //1sec
#define latchexpand 100
#define powerdownexpand 100
//#define countBias 12
#define LATCH_KEEP 1
#define LATCH_TRANSPARENT 0

#define CLOCK_LO 0
#define CLOCK_HI 1
#define THRATE 1


configManager::configManager(std::string channel, std::string chip){
    
    // todo: load names and values from config file (this way to change chip we only have to change the file)
    this -> channel = channel;
    this -> chip = chip;
    
    if (channel == "left"){
        camera = 1;
        
    } else  if (channel == "right"){
        camera = 0;
        
    }

    
    mBiases["cas"] = 52458;
    mBiases["injg"] = 101508;
    mBiases["reqPd"] = 16777215;  
    mBiases["pux"] = 8053457;  
    mBiases["diffoff"] = 133;  
    mBiases["req"] = 160712;  
    mBiases["refr"] =  944;  
    mBiases["puy"] =  16777215;  
    mBiases["diffon"] =  639172;
    mBiases["diff"] =  30108;  
    mBiases["foll"] =  20;  
    mBiases["pr"] =  5;  
    
    biasNames.resize(mBiases.size());
    int i;
    i = 0;
    biasNames[i] = "cas";
    i++;
    biasNames[i] = "injg";
    i++;
    biasNames[i] = "reqPd";
    i++;
    biasNames[i] = "pux";
    i++;
    biasNames[i] = "diffoff";
    i++;
    biasNames[i] = "req";
    i++;
    biasNames[i] = "refr";
    i++;
    biasNames[i] = "puy";
    i++;
    biasNames[i] = "diffon";
    i++;
    biasNames[i] = "diff";
    i++;
    biasNames[i] = "foll";
    i++;
    biasNames[i] = "pr";
    
}

// --- write biases to device --- //

bool configManager::programBiases(){
    
    std::vector<unsigned int> vBiases = prepareBiases();
    
    int wroteData = devManager->writeDevice(vBiases);
    
    if (wroteData <= 0)
    {
        std::cout<<"Bias write: error writing to device"<<std::endl;
        return false;
    }
    
    return true;
}

// --- change the value of a single bias --- //
bool configManager::setBias(std::string biasName, unsigned int biasValue){
    
    int currBiasVal = mBiases[biasName];
    if (currBiasVal){
        mBiases[biasName] = biasValue;
        std::cout << "setting " << biasName.c_str() << " to value: " << mBiases[biasName] << std::endl;
        return true;
    } else {
        std::cout << "wrong bias name: please check spelling" << std::endl;
        return false;
    }
    
}

// --- set the full vector that needs to be written to the device to program the chip --- //
std::vector<unsigned int> configManager::prepareBiases(){

    std::vector<unsigned int> vBiases;
    
    // write the header
    unsigned int hd;
    // set to current header the correct bits for bias programming
    hd =  header | 200;
    vBiases.push_back(hd);
    
    // implement here the transformation between the bias value list and the vector that will be sent to the device (32 bits per bias sizeof unsigned int)
    int i;
    for (i = 0; i < biasNames.size(); i++) {
        
        vBiases.push_back(mBiases[biasNames[i]]);
        
    }
    
    return vBiases;
    
}

bool configManager::prepareHeader(){
    
    header = 0; // reset the header
    
    if (channel == "left")
    {
        header = 0;
    }
    else if (channel == "right")
    {
        header = 1;
    }
    else
    {
        std::cout << "unrecognised channel" << std::endl;
        return false;
    }
    
    if (chip == "dvs")
        {
            header = header|2;
        }
    else if (chip == "atis")
        {
            header = header|3;
        }
    else
    {
        std::cout << "unrecognised chip" << std::endl;
        return false;
    }
    
    return true;
}


std::vector<unsigned int> configManager::prepareReset(bool value){
    
    unsigned int hd;
    // set to current header the correct bits for command reset
    hd =  header | 200;
    std::vector<unsigned int> vCommand;
    vCommand.push_back(hd);
    
    // write registry address and command on/off
    
    return vCommand;
}

unsigned int configManager::getBias(std::string biasName){
    
    unsigned int biasValue;
    biasValue = mBiases[biasName];
    return biasValue;
}

void configManager::printBiases(){
    std::map<std::string, unsigned int>::iterator im;
    std::cout << "Current biases for " << chip << channel << ":" << std::endl;
    for (im = mBiases.begin(); im != mBiases.end(); im++) {
    
        std::cout << im -> first << im -> second << std::endl;
        
    }
    
}

void  configManager::attachDeviceManager(deviceManager* devManager) {
    this->devManager = devManager;
    
}


// --------------------- from AEX GRABBER needed for iHead board -------------------------- //

std::vector<unsigned int> configManager::prepareBiasesAex(){
    
    //initialisation
    const uint32_t seqAllocChunk_b = SIZE_OF_EVENT * sizeof(struct aer); //allocating the right dimension for biases
    
    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;
    
    seqEvents = 0;
    seqSize_b = 0;

    for (int i = 0; i < biasNames.size(); i++) {

        progBiasAex(biasNames[i],24,mBiases[biasNames[i]]);
        
    }
    
    std::vector<unsigned int> vBiases;   
    
    latchCommitAEs();
    releasePowerdown();

    for (int i = 0; i<seqSize_b;i++){
        //printf("%08o\n",flags);
        vBiases.push_back(pseq[i].timestamp);
        vBiases.push_back(pseq[i].address);
    }

    return vBiases;
    //sendingBias();
    
}


void configManager::progBiasAex(std::string name,int bits,int value) {
    int bitvalue;
    
    for (int i=bits-1;i>=0;i--) {
        int mask=1;
        for (int j=0; j<i; j++) {
            mask*=2;
        }
        //printf("mask %d ",mask);
        if (mask & value) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        progBitAEs(bitvalue);
    }
    //after each bias value, set pins back to default value
    //resetPins();
}

void configManager::latchCommitAEs() {
    //printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_HI, 0,0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0);
    //printf("exiting latch_commit \n");
}

void configManager::progBitAEs(int bitvalue) {
    
    //set data (now)
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, bitvalue,0);
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0);
    //and wait a little
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0);
}

void configManager::biasprogtx(int time,int latch,int clock,int data, int powerdown) {
    unsigned char addr[4];
    unsigned char t[4];
    //int err;
    //setting the time
    //printf("biasProgramming following time %d \n",time);
    t[0]= time & 0xFF000000;
    t[1]= time & 0x00FF0000;
    t[2]= time & 0x0000FF00;
    t[3]= time & 0x000000FF;
    
    //setting the addr
    addr[0] = 0xFF;
    addr[1] = 0x00;
    addr[2] = 0x00;
    if(data) {
        addr[3] += 0x01;
    }
    if(clock) {
        addr[3] += 0x02;
    }
    if (latch) {
        addr[3] += 0x04;
    }
    if (powerdown) {
        addr[3] += 0x08;
    }
    //printf("data:0x%x, 0x%x, 0x%x, 0x%x \n",addr[0],addr[1],addr[2],addr[3]);
    
    
    //uint32_t seqSize_b = sizeof(struct aer);
    uint32_t timeToSend, addressToSend;
    timeToSend=time;
    
    
    // performing trasmittion differently if the camera is left (1) or right (0)
    // keep the clock high for the other eye
    if(camera) {
        addressToSend=0x000000060;
        if(data) {
            addressToSend += 0x01;
        }
        if(clock) {
            addressToSend += 0x02;
        }
        if (latch) {
            addressToSend += 0x04;
        }
        if (powerdown) {
            addressToSend += 0x08;
        }
        addressToSend+=0xFF000000;
    }
    else {
        addressToSend=0x000000006;
        if(data) {
            addressToSend += 0x10;
        }
        if(clock) {
            addressToSend += 0x20;
        }
        if (latch) {
            addressToSend += 0x40;
        }
        if (powerdown) {
            addressToSend += 0x80;
        }
        addressToSend+=0xFF000000;
    }
    
    uint32_t hwival = (uint32_t)(timeToSend * 7.8125);
    pseq[seqEvents].address = addressToSend;
    pseq[seqEvents].timestamp = hwival;
    
    seqEvents++;
    seqTime += hwival;
    seqSize_b += sizeof(struct aer);

}

void configManager::releasePowerdown() {
    //now
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 0);
    biasprogtx(latchexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 0);
    //printf("exiting latch_commit \n");
}


void configManager::setPowerdown() {
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 1);
    biasprogtx(powerdownexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 1);
}

// --- write biases to device --- //

bool configManager::programBiasesAex(){
    
    std::vector<unsigned int> vBiases = prepareBiasesAex();

    for(int i = 0; i < vBiases.size(); i++) {
        for(int j = 0; j < sizeof(int); j++) {
            if((1<<j)&vBiases[i]) std::cout << "1";
            else std::cout << "0";
        }
        std::cout << " ";
    }
    std::cout << std::endl;
    
    int wroteData = devManager->writeDevice(vBiases);
    
    if (wroteData <= 0)
    {
        std::cout<<"Bias write: error writing to device (" << wroteData << ")"
                << std::endl;
        return false;
    }
    
    return true;
}


//void configManager::sendingBias() {
//    int busy;
//    seqDone_b = 0;
//    printf("-------------------------------------------- \n");
//    printf("trying to write to kernel driver %d %d \n", seqDone_b,seqSize_b);
//    while (seqDone_b < seqSize_b) {
//        // try writing to kernel driver
//        printf( "calling write fd: sS: %d  sD: %d \n", seqSize_b, seqDone_b);
//        
//        
////        int w = write(file_desc, seqDone_b + ((uint8_t*)pseq), seqSize_b - seqDone_b);
//        
////        if (w > 0) {
////            busy = 1;
////            seqDone_b += w;
////        } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
////            /* we were not busy, maybe someone else is... */
////        } else {
////            perror("invalid write");
////            //yarp::os::exit(1);
////            return;
////        }
//    }
//    printf("writing successfully ended \n");
//    
//    ////////////////////////////////////////////////////////////
//    
//    
//    double TmaxSeqTimeEstimate =
//    seqTime * 0.128 * 1.10 +   // summed up seq intervals in us plus 10%
//    seqEvents * 1.0;           // plus one us per Event
//    
//    
//    printf("seqEvents: %d \n", seqEvents);
//    printf("seqTime * 0.128: %d \n", (int)(u64)(seqTime * 0.128));
//    
//    
//    ////////////////////////////////////////////////////////////
//    
//}










