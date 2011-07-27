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
 * @file cartesianFrameConverter.cpp
 * @brief A class inherited from the bufferefPort (see header cartesianFrameConverter.h)
 */

#include <iCub/cartesianFrameConverter.h>
#include <cassert>
#include <cstdlib>

//#define BUFFERDIM 24576 
//#define TH1 8192 
//#define TH2 16384
//#define CHUNKSIZE 8192  

//#define CHUNKSIZE 65536 
//#define TH1       65536
//#define TH2       131072
//#define BUFFERDIM 196608

#define CHUNKSIZE 1024 
#define TH1       1024
#define TH2       2048
#define BUFFERDIM 3072

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

cFrameConverter::cFrameConverter():convert_events(128,128) {
    valid = false;
    retinalSize=128;
    totDim = 0;
    pcRead = 0;
    state = 0;
    receivedBuffer = 0;
    printf ("allocating memory \n");
    converterBuffer_copy = (char*) malloc(BUFFERDIM); // allocates bytes
    converterBuffer = converterBuffer_copy;
    if (converterBuffer == 0)
        printf("null pointer \n");
    pcBuffer = converterBuffer;
    printf("setting memory \n");
    memset(converterBuffer,0,BUFFERDIM);              // set unsigned char
    pcRead = converterBuffer;
    unmask_events.start();
    printf("unmask event just started");
    previousTimeStamp = 0;
}

cFrameConverter::~cFrameConverter() {
    printf("cFrameConverter:stopping the unmasker \n");
    unmask_events.stop();
    //delete &unmask_events;
    //delete &convert_events;
    printf("cFrameConverter:freeing converterBuffer \n");
    //free(converterBuffer_copy);
}

void cFrameConverter::copyChunk(char* bufferCopy) {        
    //printf("copy \n ");
    mutex.wait();
    if(pcRead > converterBuffer +  BUFFERDIM - CHUNKSIZE) {
        memcpy(bufferCopy, pcRead, converterBuffer + BUFFERDIM - pcRead );
        pcRead = converterBuffer;
    }
    else {
        memcpy(bufferCopy, pcRead, CHUNKSIZE);
        pcRead += CHUNKSIZE;
    }
    mutex.post();
    //printf("copy chunk: ended \n ");
}

void cFrameConverter::onRead(sendingBuffer& i_ub) {
    valid = true;
    //printf("onRead ");
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
   
    mutex.wait();
    receivedBuffer = i_ub.get_packet();
    /*
    unsigned int blob, timestamp;
    for (int i = 0 ; i < dim ; i+=4) {
        unsigned int part_1 = 0xFF & receivedBuffer[i];    //extracting the 1 byte        
        unsigned int part_2 = 0xFF & receivedBuffer[i+1];  //extracting the 2 byte        
        unsigned int part_3 = 0xFF & receivedBuffer[i+2];  //extracting the 3 byte
        unsigned int part_4 = 0xFF & receivedBuffer[i+3];  //extracting the 4 byte
        //float blob = (part_1)|(part_2<<8);
        blob      = (part_1)|(part_2<<8);          //16bits
        //float timestamp = ((part_3)|(part_4<<8));
        timestamp = ((part_3)|(part_4<<8));        //16bits
        printf(">>>>>>>>> %08X %08X \n",blob,timestamp); 
        if(i == 100){
            //printf("Saving in file \n");
            //printf(">>>>>>>>> %08X %08X \n",blob,timestamp); 
            //fwrite(&sz, sizeof(int), 1, raw);
            //fwrite(buffer, 1, sz, raw);
        }
    }
    */
    memcpy(pcBuffer,receivedBuffer,dim);
    
    if (totDim < TH1) {
        pcBuffer += dim;
    }
    else if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        //printf("greater than TH1 \n");
        pcBuffer = converterBuffer + TH1; 
        pcRead = converterBuffer + TH2;
        state = 1;
    }
    else if(totDim >= TH2) {
        //printf("greater that TH2 \n");
        pcBuffer = converterBuffer;
        pcRead = converterBuffer + TH1;
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
void cFrameConverter::onRead(sendingBuffer& i_ub) {
    // receives the buffer and saves it
    //cout << "C_yarpViewer::onRead(unmaskedbuffer& i_ub)" << endl;
    //start_u = clock();
    //i_ub.get_sizeOfPacket() size of the packet in bits
    unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
    //start_p = clock();
    //stop = clock();
}
*/

void cFrameConverter::resetTimestamps() {
    unmask_events.resetTimestamps();
}

void cFrameConverter::getMonoImage(ImageOf<PixelMono>* image, unsigned long minCount, unsigned long maxCount, bool camera){
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

unsigned long cFrameConverter::getLastTimeStamp() {
    return unmask_events.getLastTimestamp();
}

unsigned long cFrameConverter::getLastTimeStampRight() {
    return unmask_events.getLastTimestampRight();
}

unsigned long cFrameConverter::getEldestTimeStamp() {
    return unmask_events.getEldestTimeStamp();
}

void cFrameConverter::clearMonoImage() {
    unmask_events.cleanEventBuffer();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

