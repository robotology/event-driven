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
 * @file eventBottleHandler.cpp
 * @brief A class inherited from the bufferefPort (see header eventBottleHandler.h)
 */

#include <iCub/eventBottleHandler.h>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>

//#include <iCub/config.h>

#define VERBOSE
#define BUFFERDIM 10000
#define CHUNKSIZE 10000

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

eventBottleHandler::eventBottleHandler() {
    
    valid           = false;
    retinalSize     = 128;
    totDim          = 0;
    pcRead          = 0;
    state           = 0;
    receivedBuffer  = 0;
    insertPosition  = 5;
    extractPosition = 0;

    printf ("allocating memory \n");
    converterBuffer_copy = (char*) malloc(BUFFERDIM); // allocates bytes
    converterBuffer = converterBuffer_copy;
    if (converterBuffer == 0) {
        printf("null pointer \n");
    }
    pcBuffer = converterBuffer;
    
    printf("setting memory \n");
    memset(converterBuffer,0,BUFFERDIM);              // set unsigned char
    pcRead = converterBuffer;
    
    //unmask_events.start();
    bufferBottle = new Bottle*[bottleBufferDimension];
    semBottleBuffer = new Semaphore*[bottleBufferDimension];
    
    for (int i = 0; i < bottleBufferDimension; i++) {
        printf("setting the bufferBottle %d \n", i);
        //bufferBottle[i] = 0;
        bufferBottle[i] = new Bottle();
        semBottleBuffer[i] = new Semaphore();
    }
    
    printf("unmask event just started \n");
    previousTimeStamp = 0;
    readEvents = fopen("./readEvents","w");
    fout = fopen("EventSniffer.dumpCartCollector.txt", "w+");

    printf("eventBottleHandler initialisation correctly ultimated \n");
}



void eventBottleHandler::reset() {
    memset(converterBuffer_copy,0,BUFFERDIM);
}

void eventBottleHandler::copyChunk(char* bufferCopy) {        
    mutex.wait();
    if(pcRead >= converterBuffer +  BUFFERDIM - CHUNKSIZE) {
        memcpy(bufferCopy, pcRead, converterBuffer + BUFFERDIM - pcRead );
        pcRead = converterBuffer;
    }
    else {
        memcpy(bufferCopy, pcRead, CHUNKSIZE);
        pcRead += CHUNKSIZE;
    }
    mutex.post();
}


void eventBottleHandler::extractBottle(Bottle* tempBottle) {
    // reading the bottle
    //printf("reading the bottle \n");
    
    //---------------------------------------
    //printf("sem address in extract %08x \n",semBottleBuffer[extractPosition] );
    
    semBottleBuffer[extractPosition]->wait();
    tempBottle->copy(*bufferBottle[extractPosition]);   // copying it in a temporary Bottle*
    //bufferBottle[extractPosition] = 0;               // setting it to zero as already read Bottle*
    bufferBottle[extractPosition]->clear();            // removes the content of the bottle.
    //printf("next istructyion will post the semaphore in extract \n");
    semBottleBuffer[extractPosition]->post();
    //----------------------------------------
    
    //printf("%d tempBottle: %08X \n",extractPosition, tempBottle);
    // updating the position of where the next extraction will happen
    mutex.wait();
    extractPosition = (extractPosition + 1) % bottleBufferDimension;
    mutex.post();
}

// reading out from a circular buffer with 2 entry points and wrapping
void eventBottleHandler::onRead(eventBottle& i_ub) {    
    valid = true;
    //printf("OnRead \n");
    
    //---------------------------------------------   
    //printf("sem address onRead %08x \n",semBottleBuffer[extractPosition] );
    //printf("trying the wait method in onRead \n");
    semBottleBuffer[insertPosition]->wait();

        
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of words     
    receivedBufferSize = dim;
    //printf("%d dim : %d \n", insertPosition,dim);
    //receivedBottle = new Bottle(*i_ub.get_packet());
    //printf("%s \n ", i_ub.get_packet()->toString().c_str());
    //receivedBottle->copy(*i_ub.get_packet());
    //printf("after copy");
    //printf("%d receivedBottle size : %d \n", insertPosition,receivedBottle->size());
    //printf("%d packet->size %d \n",insertPosition,receivedBottle->size() );
    //receivedBottle = (Bottle*) receivedBuffer;
    //printf("bufferBottle[%d] %08x i_ub.get_packet() %08X \n",insertPosition, bufferBottle[insertPosition], i_ub.get_packet()  );
    //delete bufferBottle[insertPosition];
    //bufferBottle[insertPosition] = receivedBottle;
    //printf("receivedBottle  %08x \n",receivedBottle);        
    bufferBottle[insertPosition]->copy(*i_ub.get_packet());          

    //printf("insertPosition %d  \n", insertPosition);
#ifdef VERBOSE
    int num_events = dim >> 3 ;
    fprintf(fout, "dim: %d \n",dim);
    //plotting out
    string str;
    int chksum;

    /*
    for (int i=0; i < bufferBottle[insertPosition]->size(); i++) {
        
        if(bufferBottle[insertPosition]->get(i).asInt() >= 0x80000000) {
            fprintf(fout,"%08X ", bufferBottle[insertPosition]->get(i).asInt());
        }
        else {
            fprintf(fout,"%08X \n ", bufferBottle[insertPosition]->get(i).asInt());
        }
        //printf("%08X \n", receivedBottle->get(i).asInt());
        int chksum = bufferBottle[insertPosition]->get(i).asInt() % 255;
        //str[i] = (char) chksum;
    }
    //fprintf(fout,"chksum: %s \n", str.c_str());
    fprintf(fout,"----------------------------- \n");
    */

    eEventQueue rxQueue;    // the ownership is true by default
    bool   ok = eEvent::decode(packets,rxQueue);
    for (size_t i=0; i<rxQueue.size(); i++)
    {
        // to identify the type of the packet
        // user can rely on the getType() method
        if (rxQueue[i]->getType()=="AE")
        {
            AddressEvent* ptr=dynamic_cast<AddressEvent*>(rxQueue[i]);
            fprintf(fout,"%s \n",rxQueue[i]->getContent().toString().c_str());
        }
    
#endif
    

    semBottleBuffer[insertPosition]->post();
    //----------------------------------------------

    // changing the value of the insert position
    mutex.wait();
    insertPosition = (insertPosition + 1) % bottleBufferDimension;
    mutex.post();
}
/*
void eventBottleHandler::onRead(eventBottle& i_ub) {    
    valid = true;

    
    mutex.wait();
    
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of words
    //printf("eventBottleHandler::  %d \n", dim);
    
    receivedBufferSize = dim;
    receivedBottle = i_ub.get_packet();
    //receivedBottle = (Bottle*) receivedBuffer;
    
    
#ifdef VERBOSE
    int num_events = dim >> 3 ;
    //printf("size of the received bottle %d \n",receivedBottle->size() );
    //plotting out
    for (int i=0; i < receivedBottle->size(); i++) {
        fprintf(fout,"%08X \n", receivedBottle->get(i).asInt());      
    }
    fprintf("################ \n");
#endif

    mutex.post();

    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead); 
}
*/


void eventBottleHandler::resetTimestamps() {
    //unmask_events.resetTimestamps();
}

void eventBottleHandler::getMonoImage(ImageOf<PixelMono>* image, unsigned long minCount, unsigned long maxCount, bool camera){
    assert(image!=0);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    
    // determining whether the camera is left or right
    // int* pBuffer = unmask_events.getEventBuffer(camera);
    int* pBuffer = 0;
    // unsigned long* pTime   = unmask_events.getTimeBuffer(camera);
    unsigned long* pTime;
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            unsigned long timestampactual = *pTime;
            if (((timestampactual * 1.25) > minCount)&&((timestampactual * 1.25) < maxCount)) {   //(timestampactual != lasttimestamp)
                *pImage = (unsigned char) 127 + value;
               
            }
            else {
                *pImage = (unsigned char) 127;
               
                }
            // Connect nearby points in the image provided they lie on a line. primitive method  here just to check
            int dist = 20;
            bool lookForHor,lookForVert,lookForSlantLeft, lookForSlantRight;
            lookForHor = lookForVert = lookForSlantLeft = lookForSlantRight = true;
            
            if(r>dist && c >dist){
                for(int i=1;i<=dist;++i){
                    if(lookForSlantLeft && *(image->getPixelAddress(r-i,c-i)) == 127 && *(image->getPixelAddress(r-i-1,c-i-1)) == *(image->getPixelAddress(r,c))){
                        *pImage = *(image->getPixelAddress(r-i-1,c-i-1));
                        lookForSlantLeft = false;
                    }
                    if(lookForVert && *(image->getPixelAddress(r-i,c)) == 127 && *(image->getPixelAddress(r-i-1,c)) == *(image->getPixelAddress(r,c))){
                        *pImage = *(image->getPixelAddress(r-i-1,c));
                        lookForVert = false;
                    }
                    if(lookForHor && *(image->getPixelAddress(r,c-i)) == 127 && *(image->getPixelAddress(r,c-i-1)) == *(image->getPixelAddress(r,c))){
                        *pImage = *(image->getPixelAddress(r,c-i-1));
                        lookForHor = false;
                    }
                    if(lookForSlantRight && *(image->getPixelAddress(r-i,c+i)) == 127 && *(image->getPixelAddress(r-i-1,c+i+1)) == *(image->getPixelAddress(r,c))){
                        *pImage = *(image->getPixelAddress(r-i-1,c+i+1));
                        lookForSlantRight = false;
                    }
                }                
            }
            pImage++;
            pBuffer ++;
            pTime ++;
        }
        pImage+=imagePadding;
    }

    
    //unmask_events.setLastTimestamp(0);
    
}

unsigned long eventBottleHandler::getLastTimeStamp() {
    //return unmask_events.getLastTimestamp();
    return 0;
}

unsigned long eventBottleHandler::getLastTimeStampRight() {
    //return unmask_events.getLastTimestampRight();
    return 0;
}

unsigned long eventBottleHandler::getEldestTimeStamp() {
    //return unmask_events.getEldestTimeStamp();
    return 0;
}

void eventBottleHandler::clearMonoImage() {
    //unmask_events.cleanEventBuffer();
}

eventBottleHandler::~eventBottleHandler() {
    printf("eventBottleHandler:stopping the unmasker \n");
    //unmask_events.stop();
    //delete &unmask_events;
    //delete &convert_events;
    printf("eventBottleHandler:freeing converterBuffer \n");
    free(converterBuffer_copy);
    fclose(fout);
    
    printf("eventBottleHandler:freeing the buffer of bottle \n");
    delete[] bufferBottle;
    delete[] semBottleBuffer;
    printf("eventBottleHandler:success in converting \n");
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------

