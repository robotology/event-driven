// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file velocityBottleHandler.cpp
 * @brief A class inherited from the bufferefPort (see header velocityBottleHandler.h)
 */

#include <iCub/velocityBottleHandler.h>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>
#include <string>

//#define VERBOSE
#define BUFFERDIM 1000
#define CHUNKSIZE 1000

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

velocityBottleHandler::velocityBottleHandler() {
    
    valid           = false;
    retinalSize     = 128;
    totDim          = 0;
    pcRead          = 0;
    state           = 0;
    receivedBuffer  = 0;
    insertPosition  = 5;
    extractPosition = 0;

    // allocationg memory
    printf ("allocating memory \n");
    converterBuffer_copy = (char*) malloc(BUFFERDIM); // allocates bytes
    converterBuffer = converterBuffer_copy;
    if (converterBuffer == 0)
        printf("null pointer \n");
    pcBuffer = converterBuffer;
    bufferVelocity  = new VelocityBuffer*[bottleBufferDimension];                 //allocated memory for an array of bottle pointers
    semBottleBuffer = new Semaphore*[bottleBufferDimension];
    

    // setting memory
    printf("setting memory \n");
    memset(converterBuffer,0,BUFFERDIM);              // set unsigned char
    pcRead = converterBuffer;
    
    for (int i = 0; i < bottleBufferDimension; i++) {
        bufferVelocity[i]  = 0;
        semBottleBuffer[i] = new Semaphore();
    }

    printf("unmask event just started \n");
    previousTimeStamp = 0;
    readEvents = fopen("./readEvents","w");
    fout = fopen("velocityExtractor.velocityBottleHandler.txt", "w+");
}

void velocityBottleHandler::reset() {
    memset(converterBuffer_copy,0,BUFFERDIM);
}

void velocityBottleHandler::copyChunk(char* bufferCopy) {        
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

VelocityBuffer* velocityBottleHandler::extractBottle(VelocityBuffer* tempBottle) {
    // reading the bottle
    //printf("extractBottle \n");
    
    //---------------------------------------
    //printf("sem address in extract %08x \n",semBottleBuffer[extractPosition] );
    
    semBottleBuffer[extractPosition]->wait();
    if( bufferVelocity[extractPosition] == 0) {
        tempBottle = 0;
    }
    else {
        if(tempBottle!=0) delete tempBottle;
        tempBottle = new VelocityBuffer();
        *tempBottle = *(bufferVelocity[extractPosition]);    //copying operator of the class
        bufferVelocity[extractPosition] = 0;                 // setting it to zero as already read Bottle*
        //bufferVelocity[extractPosition]->clear();          // removes the content of the bottle.
    }

    //printf("next instruction will post the semaphore in extract \n");
    semBottleBuffer[extractPosition]->post();
    //----------------------------------------
    
    //printf("%d tempBottle: %08X \n",extractPosition, tempBottle);
    // updating the position of where the next extraction will happen
    mutex.wait();
    extractPosition = (extractPosition + 1) % bottleBufferDimension;
    mutex.post();
    
    //printf("success in velocityBottleHandler \n");
    return tempBottle;
    
}

// reading out from a circular buffer with 2 entry points and wrapping
void velocityBottleHandler::onRead(VelocityBuffer& i_ub) {    
    //printf("OnRead \n");
    valid = true;
    
    
    
    //---------------------------------------------   
    //printf("sem address onRead %08x \n",semBottleBuffer[extractPosition] );
    //printf("trying the wait method in onRead \n");
    semBottleBuffer[insertPosition]->wait();
        
    // receives the buffer and saves it    
    int dim = i_ub.getSize();
    //printf("dim : %d                           \r", dim);
    
    bufferVelocity[insertPosition] =  &i_ub;          
    

    
#ifdef VERBOSE
    //fprintf(fout, "dim: %d \n",dim);
    //plotting out
    
    for (int i=0; i < bufferVelocity[insertPosition]->getSize(); i++) {
        
        fprintf(fout,"%d ", bufferVelocity[insertPosition]->getX(i));
        fprintf(fout,"%d ", bufferVelocity[insertPosition]->getY(i));
        fprintf(fout,"%f ", bufferVelocity[insertPosition]->getVx(i));
        fprintf(fout,"%f ", bufferVelocity[insertPosition]->getVy(i));
        fprintf(fout,"  \n");
        
        //printf("%08X \n", receivedBottle->get(i).asInt());
        //int chksum = bufferBottle[insertPosition]->get(i).asInt() % 255;
        //str[i] = (char) chksum;
    }
    //fprintf(fout,"chksum: %s \n", str.c_str());
    //fprintf(fout,"----------------------------- \n");
#endif
    

    semBottleBuffer[insertPosition]->post();
    //----------------------------------------------

    
    // changing the value of the insert position
    mutex.wait();
    insertPosition = (insertPosition + 1) % bottleBufferDimension;
    mutex.post();
    
    
}

void velocityBottleHandler::resetTimestamps() {
    //unmask_events.resetTimestamps();
}

void velocityBottleHandler::getMonoImage(ImageOf<PixelMono>* image, unsigned long minCount, unsigned long maxCount, bool camera){
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

unsigned long velocityBottleHandler::getLastTimeStamp() {
    //return unmask_events.getLastTimestamp();
    return 0;
}

unsigned long velocityBottleHandler::getLastTimeStampRight() {
    //return unmask_events.getLastTimestampRight();
    return 0;
}

unsigned long velocityBottleHandler::getEldestTimeStamp() {
    //return unmask_events.getEldestTimeStamp();
    return 0;
}

void velocityBottleHandler::clearMonoImage() {
    //unmask_events.cleanEventBuffer();
}

velocityBottleHandler::~velocityBottleHandler() {
    printf("velocityBottleHandler:stopping the unmasker \n");
    //unmask_events.stop();
    //delete &unmask_events;
    //delete &convert_events;
    printf("velocityBottleHandler:freeing converterBuffer \n");
    //free(converterBuffer_copy);
    fclose(fout);

    printf("velocityBottleHandler:freeing the buffer of bottle \n");
    delete[] bufferVelocity;
    delete[] semBottleBuffer;
    printf("velocityBottleHandler:success in converting \n");
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------

