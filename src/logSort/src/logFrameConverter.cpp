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

#include <iCub/logFrameConverter.h>
#include <cassert>
#include <cstdlib>


// note that bufferdim has to be 1 chunksize bigger TH3 to avoid overflows
#define BUFFERDIM 4096 //6144  //36864
#define TH1 1024 //2048 //12
#define TH2 2048 //4096  
#define TH3 3072
#define CHUNKSIZE 1024//2048

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
    pcBuffer = converterBuffer;
    pcRead   = converterBuffer + TH1;
    flagCopy = unreadBuffer;
    flagRead = unreadBuffer + TH1;
    unmask_events.start();
    printf("unmask event just started");
    previousTimeStamp = 0;
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
}

void logFrameConverter::copyChunk(char* bufferCopy, char* flagBuffer) {            
    mutex.wait();    
    if(pcRead >= converterBuffer +  BUFFERDIM - CHUNKSIZE) {
        memcpy(bufferCopy, pcRead, converterBuffer + BUFFERDIM - pcRead );
        memset(flagCopy, 0, converterBuffer + BUFFERDIM - pcRead);
        pcRead   = converterBuffer;
        flagCopy = unreadBuffer;
    }
    else {
        memcpy(bufferCopy, pcRead, CHUNKSIZE);
        memcpy(flagBuffer,flagCopy, CHUNKSIZE);
        memset(flagCopy, 0, CHUNKSIZE);
        pcRead   += CHUNKSIZE;
        flagCopy += CHUNKSIZE;
    }
    //countBuffer -= CHUNKSIZE;
    //flagBuffer = flagCopy;
    mutex.post(); 
}

void logFrameConverter::onRead(sendingBuffer& i_ub) {

    // pcBuffer where the new reading is copied
    // pcRead   pointer to the chunk that is exported
    // in this version the pcBuffer is behind one chunckSize respect the pcRead

    // there is a second couple of pointers for the unreadBuffer that move tighly with the previous couple 


    valid = true;
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    if(dim == 0){
        return;
    }

    printf("reading %d \n", dim); 
    mutex.wait();
    receivedBuffer = i_ub.get_packet();
    memcpy(pcBuffer,receivedBuffer,dim);
    memset(flagRead,1,dim);
    
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
            if (((timestampactual * 1.25) > minCount)&&((timestampactual * 1.25) < maxCount)) {   //(timestampactual != lasttimestamp)
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

