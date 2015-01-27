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
 * @file processingThread.cpp
 * @brief Implementation of the thread that process the events in the collected buffers (see header plotterThread.h)
 */


#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cstdio>
#include <cxcore.h>


#include <iCub/processingThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 5 
//#define MAXLIMIT   1
//#define MINLIMIT   0
//#define retinalSize 128

processingThread::processingThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    retinalSize =  24;
    countEM1 = 0;
    countEM2 = 0;
    countEM3 = 0;
    countEM4 = 0;
    bufferEM1 = 0;
    bufferEM2 = 0;
    bufferEM3 = 0;
    bufferEM4 = 0;
    cartEM     = new aer[24 * 24];
    pEM        = new aer[24 * 24];
    cartCount  = new int[24 * 24];    
    memset(cartEM,    0, 24 * 24 * sizeof(aer));
    memset(pEM,       0, 24 * 24 * sizeof(aer));
    memset(cartCount, 0, 24 * 24 * sizeof(int));

    maxlimit = 0.01;
    minlimit = 0.001;
}

processingThread::~processingThread() {
    printf("freeing memory in collector");
    delete[] cartEM;
    delete[] pEM;
    delete[] cartCount;
}

bool processingThread::threadInit() {
    printf("\n starting the thread.... \n");    
    return true;
}

void processingThread::interrupt() {
}

void processingThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string processingThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void processingThread::resetTOTEM() {
    memset(cartEM,    0, 24 * 24 * sizeof(aer));
    memset(pEM,       0, 24 * 24 * sizeof(aer));
    memset(cartCount, 0, 24 * 24 * sizeof(int));
}

void processingThread::getEM(aer** pointerEM, int* dimEM) {
    //printf("counted EM %d \n", countEM);
    //*pointerEM = bufferEM1;
    //*dimEM = countEM1 + countEM2 + countEM3 + countEM4;
    *dimEM = 0;
    
    for (int pos = 0; pos < 24 * 24; pos++){
        if(cartEM[pos].address != 0) {
            //printf("cartEM[pos] %08x \n",cartEM[pos].address);
            pEM[*dimEM] = cartEM[pos];
            (*dimEM)++;            
        }
    }
    *pointerEM = pEM;
    //delete[] pEM;
    memset(cartEM,    0, 24 * 24 * sizeof(aer));
    memset(cartCount, 0, 24 * 24 * sizeof(int));
}

void processingThread::setEM(aer* bEM1, aer* bEM2, aer* bEM3, aer* bEM4) {
    bufferEM1 = bEM1;
    bufferEM2 = bEM2;
    bufferEM3 = bEM3;
    bufferEM4 = bEM4;
}

void processingThread::addBufferEM(aer* event){
    // extracts coordinate in the output image size
    unsigned long current_blob      = event->address;
    unsigned long current_timestamp = event->timestamp;
    unsigned int current_value      = (current_blob & 0xffff0000) >> 16;
   
    unsigned short x = (current_blob & 0x00FE) >> 1;
    unsigned short y = (current_blob & 0xFF00) >> 8;
    //printf("current_blob %08x position x %d y %d value %d \n",current_blob, x, y, current_value);
         
    // compare with the  event was inside
    int position = x + y * 24;
    if((cartEM[position].address == 0)&&(cartEM[position].timestamp == 0)){
        cartEM[position].address   = event->address;
        cartEM[position].timestamp = event->timestamp;
    }
    else {
        //taking the mean value of the two events;
        unsigned long old_blob      = cartEM[position].address;
        unsigned long old_timestamp = cartEM[position].timestamp;
        unsigned int  old_value     = (old_blob & 0xffff0000) >> 16;
        cartCount[position]++;
        unsigned int  new_value     = (int)floor((current_value + old_value)/cartCount[position]);
        //printf("old_blob %x old_value %d current_value %d  new_value %d \n",old_blob, old_value,current_value, new_value);
        unsigned long new_timestamp = current_timestamp; //(old_timestamp + current_timestamp) >> 1;
        unsigned long new_blob      = (new_value << 16) | (old_blob & 0x0000FFFF) ; 
        cartEM[position].address    = new_blob;
        cartEM[position].timestamp  = new_timestamp;
    }
}


unsigned long processingThread::look4opposite(aer* buffer,int initPos, int countTOT){
    unsigned long targetBlob;
    unsigned long blob      = buffer[initPos].address;
    unsigned long timestamp = buffer[initPos].timestamp;
    //printf("\n \n analising %08x in countCount -->",blob, countTOT );
    if (blob % 2 == 0) {
        //printf(" even \n");
        // high event looking for low
        targetBlob = blob + 1;
    }
    else {
        //printf(" odd");
        return timestamp;
        //low event looking for high
        targetBlob = blob - 1;
    }
    bool found = false;
    int i = initPos + 1;
    while ((i< countTOT) && (!found)) {
        if(buffer[i].address == targetBlob) {
            found = true;
        }        
        i++;
    }
    if(found) {
        //printf("found \n");
        return (unsigned long) buffer[i].timestamp;
    }
    else {
        //printf("notFound \n");
        return (unsigned long) buffer[initPos].timestamp;
    }
}


void processingThread::run() {
    count++;
    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM1
    unsigned long maxdiff = 0, mindiff = 0xFFFFFFFF;
    aer* tmp = new aer;

    for(int i = 0; i< countEM1-1 ; i++){
        //printf("analysing EM1 position %d \n", countEM1);
        unsigned long blob = bufferEM1[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM1[i].timestamp;
        timestampFound = look4opposite(bufferEM1,i,countEM1);
        long diff =  timestampFound - timestamp;
        if(diff == 0) {
            continue;
        }
        unsigned long absdiff = std::abs(diff);
        double invertedDiff   = (double) absdiff / 1000000;
        invertedDiff = 1.0 / invertedDiff ;

        //printf("EM1 - 1.buffer.address %08x   ",bufferEM1[i].address,bufferEM1[i].timestamp, timestampFound);
        //printf("2.value %lu %f \n ",absdiff, invertedDiff);
        if (invertedDiff > maxdiff){ maxdiff = invertedDiff; }
        if (invertedDiff < mindiff){ mindiff = invertedDiff; }
        int numbits = 256;        
        double max = 0.1;
        double min = 0.001;
        // grayscale  = 1 / exposure_measure
        int value = floor(
                          ((double)invertedDiff /(max - min)) * numbits
                          );
        if(value > 255) {
            value = 255;
        }
        
        tmp->address = (bufferEM1[i].address & 0x0000FFFF) | (value<<16);
        tmp->timestamp = timestamp;
        //printf("3.buffer.address %08x %d \n",tmp->address,value);
        addBufferEM(tmp);        
    }

    
    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM2    
    for(int i = 0; i< countEM2 - 1 ; i++){
        //printf("analysing EM2 position %d \n", countEM2);
        unsigned long blob = bufferEM2[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM2[i].timestamp;
        timestampFound = look4opposite(bufferEM2,i,countEM2);
        long diff =  timestampFound - timestamp;
        if(diff == 0) {
            continue;
        }
        unsigned long absdiff = std::abs(diff);
        double invertedDiff   = (double) absdiff / 1000000;
        invertedDiff = 1.0 / invertedDiff ;


        //printf("EM2 - 1.buffer.address %08x   ",bufferEM2[i].address);
        //printf("2.value %lu    \n ", absdiff);
        if (invertedDiff > maxdiff){ maxdiff = invertedDiff; }
        if (invertedDiff < mindiff){ mindiff = invertedDiff; }

        int numbits = 256;        
        double max = 0.1;
        double min = 0.001;
        // grayscale  = 1 / exposure_measure
        int value = floor(
                          (invertedDiff /(max - min)) * 256.0
                          );
        if(value > 255) {
            value = 255;
        }
        
        bufferEM2[i].address = (bufferEM2[i].address & 0x0000FFFF) | (value<<16);
        //printf("3.buffer.address %08x %d \n",bufferEM2[i].address, value);
        addBufferEM(&bufferEM2[i]);
    }
    

    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM3
    
    for(int i = 0; i< countEM3 - 1 ; i++){
        //printf("analysing EM3 position %d \n", countEM3);
        unsigned long blob = bufferEM3[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM3[i].timestamp;
        timestampFound = look4opposite(bufferEM3,i,countEM3);
        long diff =  timestampFound - timestamp;
        if(diff == 0) {
            continue;
        }
        unsigned long absdiff = std::abs(diff);
        double invertedDiff   = (double) absdiff / 1000000;
        invertedDiff = 1.0 / invertedDiff ;

        //printf("EM3 - 1.buffer.address %08x   ",bufferEM3[i].address);
        //printf("2.value %lu   \n", absdiff);
        if (invertedDiff > maxdiff){ maxdiff = invertedDiff; }
        if (invertedDiff < mindiff){ mindiff = invertedDiff; }

        int numbits = 256;        
        double max = 0.1;
        double min = 0.001;
        // grayscale  = 1 / exposure_measure
        int value = floor(
                          (invertedDiff /(max - min)) * numbits
                          );
        if(value > 255) {
            value = 255;
        }
        
        bufferEM3[i].address = (bufferEM3[i].address & 0x0000FFF) | (value<<16);
        //printf("3.buffer.address %08x %d \n",bufferEM3[i].address, value);
        addBufferEM(&bufferEM3[i]);
    }    
    //printf(" - - - - - - - - - - - - - - \n");
    

    // searching the couples in EM4
    for(int i = 0; i< countEM4 - 1; i++){
        //printf("analysing EM4 position %d \n", countEM4);
        unsigned long blob = bufferEM4[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM4[i].timestamp;
        timestampFound = look4opposite(bufferEM4,i,countEM4);
        long diff =  timestampFound - timestamp;
        if(diff == 0) {
            continue;
        }

        unsigned long absdiff = std::abs(diff);
        double invertedDiff   = (double) absdiff / 1000000;
        invertedDiff = 1.0 / invertedDiff ;
        


        if (invertedDiff > maxdiff){ maxdiff = invertedDiff; }
        if (invertedDiff < mindiff){ mindiff = invertedDiff; }

        //printf("EM4 - 1.buffer.address %08x   ",bufferEM4[i].address);
        //printf("2.value %lu  \n ", absdiff);
        int numbits = 256;        
        double max = 0.1;
        double min = 0.001;
        // grayscale  = 1 / exposure_measure
        int value = floor( 
                          (invertedDiff /(max - min)) * numbits
                          );
        if(value > 255) {
            value = 255;
        }
        
        bufferEM4[i].address = (bufferEM4[i].address & 0x0000FFFF) | (value<<16);
        //printf("3.buffer.address %08x %d \n",bufferEM4[i].address, value);
        addBufferEM(&bufferEM4[i]);
    } 
    
    
    delete tmp;
    
    //if(maxdiff != 0) {
    //  printf("limits:  maxdiff  %lu \n", maxdiff);
    //}
    //if(mindiff != 0xFFFFFFFF){
    //  printf("limits: mindiff %lu \n", mindiff);
    //}
}




void processingThread::threadRelease() {
  printf("processingThread: releasing \n");  
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
