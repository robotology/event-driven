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
 * @file logFrameConverter.cpp
 * @brief A class inherited from the bufferefPort (see header logFrameConverter.h)
 */

#include <cassert>
#include <cstdlib>

#include <iCub/config.h>
#include <iCub/logFrameConverter.h>

// note that bufferdim has to be 1 chunksize bigger TH3 to avoid overflows
//#define BUFFERDIM 65536 //4096 //6144  //36864
//#define TH1 16384 //1024 //2048 //12
//#define TH2 32768 //2048 //4096  
//#define TH3 49152 //3072
//#define CHUNKSIZE 16384 //1024//2048

//#define VERBOSE

//#define CHUNKSIZE 32768 
//#define TH1       32768  
//#define TH2       65536
//#define TH3       98304
//#define BUFFERDIM 131702

//#define CHUNKSIZE 8192 
//#define TH1       8192  
//#define TH2       16384
//#define TH3       24576
//#define BUFFERDIM 32768

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

logFrameConverter::logFrameConverter():convert_events(128,128) {
    countBuffer = 0;
    valid = false;
    retinalSize=128;
    totDim = 0;
    pcRead = 0;
    state = 0;
    receivedBuffer = 0;
    printf ("allocating memory \n");
    converterBuffer_copy = (char*) malloc(BUFFERDIM); // allocates bytes
    unreadBuffer         = (char*) malloc(BUFFERDIM); // allocates flag buffer
    converterBuffer = converterBuffer_copy;
    if (converterBuffer == 0)
        printf("null pointer \n");
    
    printf("setting memory \n");
    memset(converterBuffer,0,BUFFERDIM);         // set unsigned char
    memset(unreadBuffer,  0, BUFFERDIM);        
    pcBuffer = converterBuffer;      //saving events
    pcRead   = converterBuffer + TH1;//reading events
    flagCopy = unreadBuffer;         //saving flag
    flagRead = unreadBuffer + TH1;   //reading flag
    unmask_events.start();
    printf("unmask event just started");
    previousTimeStamp = 0;
    fout = fopen("dumpConverter.txt", "w+");
}

logFrameConverter::~logFrameConverter() {
    printf("logFrameConverter:stopping the unmasker \n");
    unmask_events.stop();
    //delete &unmask_events;
    //delete &convert_events;
    printf("logFrameConverter:freeing converterBuffer \n");
    free(converterBuffer_copy);
    printf("logFrameConverter:freeing unreadBuffer \n");
    free(unreadBuffer);
    fclose(fout);
}

void logFrameConverter::copyChunk(char* bufferCopy, char* flagBuffer) {            
    mutex.wait();  
    char* limit = converterBuffer +  BUFFERDIM - CHUNKSIZE;
    int value = limit - pcRead;
    if(pcRead >= limit) {
        memcpy(bufferCopy, pcRead, converterBuffer +  BUFFERDIM - pcRead );
        memset(pcRead, 0,   converterBuffer +  BUFFERDIM - pcRead );
        memcpy(flagBuffer,flagRead,converterBuffer +  BUFFERDIM - pcRead );
        memset(flagRead, 0, converterBuffer +  BUFFERDIM  - pcRead);
        pcRead   = converterBuffer + TH1; 
        flagCopy = unreadBuffer;
    }
    else {       
        memcpy(bufferCopy, pcRead,   CHUNKSIZE);
        memset(pcRead, 0, CHUNKSIZE);  // zeroing events already read
        memcpy(flagBuffer, flagRead, CHUNKSIZE);        
        memset(flagRead, 0, CHUNKSIZE);
        pcRead   += CHUNKSIZE;
        flagCopy += CHUNKSIZE;
    }
    mutex.post(); 

#ifdef VERBOSE
    int num_events = CHUNKSIZE >> 3 ;
    uint32_t* buf2 = (uint32_t*)bufferCopy;
    uint32_t* bufflag = (uint32_t*) flagBuffer;
    uint32_t* bufflag2 = (uint32_t*) flagCopy;
    //plotting out
    for (int evt = 0; evt < num_events; evt++) {
        unsigned long blob      = buf2[2 * evt];
        unsigned long t         = buf2[2 * evt + 1];
        unsigned long flag1     = bufflag[2 * evt];
        unsigned long flag2     = bufflag[2 * evt + 1];
        unsigned long flag1b     = bufflag2[2 * evt];
        unsigned long flag2b     = bufflag2[2 * evt + 1];
        if(blob != 0) {
            fprintf(fout,"%08X %08X \n",blob,t);
            fprintf(fout,"%08X %08X  \n",flag1,flag2);
            fprintf(fout,"%08X %08X  \n",flag1b,flag2b);
        }
    }
    fprintf(fout,"################## \n");
#endif
    
    //countBuffer -= CHUNKSIZE;
    //flagBuffer = flagCopy;
}

// reading out from a circular buffer with 2 entry points
void logFrameConverter::onRead(eventBuffer& i_ub) {
    valid = true;
    //printf("onRead ");
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
    
    if(dim == 0) {
        return;
    }

    //fprintf(fout, "iiiiiiiiiiiiiiiiiiii \n");
    
    mutex.wait();
    receivedBuffer = i_ub.get_packet();    

    //mem copying
    memcpy(pcBuffer,receivedBuffer,dim);
    memset(flagCopy,1,dim);

#ifdef VERBOSE
    //int num_events = dim >> 3 ;
    //uint32_t* buf2 = (uint32_t*)pcBuffer;
    //plotting out
    //    for (int evt = 0; evt < num_events; evt++) {
    //        unsigned long blob      = buf2[2 * evt];
    //        unsigned long t         = buf2[2 * evt + 1];
    //        fprintf(fout,"%08X %08X \n",blob,t);        
    //    }
#endif
    
    if (totDim < TH1) {
        pcBuffer += dim;
        flagCopy += dim;
    }
    else if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        //printf("greater than TH1 \n");
        pcBuffer = converterBuffer + TH1;
        flagCopy = unreadBuffer    + TH1;
        pcRead   = converterBuffer + TH2;
        flagRead = unreadBuffer    + TH2;
        state = 1;
    }
    else if(totDim >= TH2) {
        //printf("greater that TH2 \n");
        pcBuffer = converterBuffer;
        flagCopy = unreadBuffer;
        pcRead   = converterBuffer + TH1;
        flagRead = unreadBuffer    + TH1;
        totDim = 0;
        state = 0;
    }
    // the thrid part of the buffer is free to avoid overflow
    totDim += dim;
    mutex.post();

    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead); 
}


/*
//three entry points
void logFrameConverter::onRead(eventBuffer& i_ub) {

    // pcBuffer where the new reading is copied
    // pcRead   pointer to the chunk that is exported
    // in this version the pcBuffer is behind one chunckSize respect the pcRead

    // there is a second couple of pointers for the unreadBuffer that move tighly with the previous couple 


    valid = true;
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("reading -------------------  %d  -------------------------- \n", dim);
    if(dim == 0){
        return;
    }
     
    mutex.wait();
    receivedBuffer = i_ub.get_packet();

    //pcBuffer = i_ub.get_packet();
    //struct aer *pmon;
    //pmon = (aer*) receivedBuffer;
    memcpy(pcBuffer,receivedBuffer,dim);
    memset(flagRead,1,dim);

#ifdef VERBOSE
    //verbosity
    //reading the received buffer
    //uint32_t* buf2 = (uint32_t*)receivedBuffer;    
    //reading the copied buffer
    uint32_t* buf2 = (uint32_t*)pcBuffer;
    int num_events = dim / 8; 
    u32 a, t;  
    for (int evt = 0; evt < num_events; evt++) {
        //a = pmon[evt].address;
        //t = pmon[evt].timestamp;
        
        // logUnmask the data ( first 4 bytes blob, second 4 bytes timestamp)
        unsigned long blob      = buf2[2 * evt];
        unsigned long timestamp = buf2[2 * evt + 1];        
                
        // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        //blob &= 0xFFFF;
        //printf("logFrameConverter: read rough data events : %08X %08X \n",blob,timestamp);
        bool save = true;
        if (save) {
            fprintf(fout,"%08X %08X \n",blob,timestamp); 
            //fout<<hex<<a<<" "<<hex<<t<<endl;
        }        
    }
    fprintf(fout, "######## \n");
#endif
    
    //if (totDim < TH1) {
    //    pcBuffer += dim;
    //    flagRead += dim;
    //}
    //else 
    
    pcBuffer += dim;
    flagRead += dim;
    totDim   += dim;
        
    if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        pcBuffer = converterBuffer + TH1; 
        flagRead = unreadBuffer    + TH1;
        pcRead   = converterBuffer + TH2;
        flagCopy = unreadBuffer    + TH2;
        state    = 1;
    }
    else if((totDim >= TH2)&&(totDim < TH3)&&(state!=2)) {
        pcBuffer = converterBuffer + TH2;
        flagRead = unreadBuffer    + TH2;
        pcRead   = converterBuffer;
        flagCopy = unreadBuffer   ;       
        state    = 2;
    }
    else if(totDim >= TH3) {
        pcBuffer = converterBuffer;
        flagRead = unreadBuffer   ;
        pcRead   = converterBuffer + TH1;
        flagCopy = unreadBuffer    + TH1;
        totDim   = 0;
        state    = 0;
    }
    // the thrid part of the buffer is free to avoid overflow    
    mutex.post();
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead);
}
*/


/*
// reading out from a circular buffer with 3 entry points
void cFrameConverter::onRead(eventBuffer& i_ub) {
    valid = true;
    //printf("onRead ");
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
   
    mutex.wait();
    receivedBuffer = i_ub.get_packet(); 
    memcpy(pcBuffer,receivedBuffer,dim);
    
    
    pcBuffer += dim;
    
    
    if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        pcBuffer = converterBuffer + TH1; 
        pcRead = converterBuffer + TH2;
        state = 1;
    }else if((totDim >= TH2)&&(totDim < TH3)&&(state!=2)) {
        pcBuffer = converterBuffer + TH2;
        pcRead   = converterBuffer;       
        state    = 2;
    }
    else if(totDim >= TH3) {
        pcBuffer = converterBuffer;
        pcRead   = converterBuffer + TH1;
        totDim   = 0;
        state    = 0;
    }
    // after the thrid part of the buffer is free to avoid overflow
    totDim += dim; 

    mutex.post();
    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead);
   
}
*/

void logFrameConverter::resetTimestamps() {
    unmask_events.resetTimestamps();
}

void logFrameConverter::getMonoImage(ImageOf<PixelMono>* image, unsigned long minCount, unsigned long maxCount, bool camera){
    assert(image!=0);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    

    // determining whether the camera is left or right
    int* pBuffer = unmask_events.getEventBuffer(camera);
    unsigned long* pTime   = unmask_events.getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            unsigned long timestampactual = *pTime;
            if ((timestampactual  > minCount)&&(timestampactual  < maxCount)) {   //(timestampactual != lasttimestamp)
                *pImage++ = (unsigned char) 127 + value;
               
            }
            else {
                *pImage++ = (unsigned char) 127;
               
                }
            pBuffer ++;
            pTime ++;
        }
        pImage+=imagePadding;
    }
    //unmask_events.setLastTimestamp(0);
}

unsigned long logFrameConverter::getLastTimeStamp() {
    return unmask_events.getLastTimestamp();
}

unsigned long logFrameConverter::getLastTimeStampRight() {
    return unmask_events.getLastTimestampRight();
}

unsigned long logFrameConverter::getEldestTimeStamp() {
    return unmask_events.getEldestTimeStamp();
}

void logFrameConverter::clearMonoImage() {
    unmask_events.cleanEventBuffer();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

