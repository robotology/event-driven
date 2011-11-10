// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file efExtractorThread.cpp
 * @brief Implementation of the thread (see header efExtractorThread.h)
 */

#include <iCub/efExtractorThread.h>
#include <iCub/cartesianFrameConverter.h>

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define DIM 10
#define THRATE 30
#define SHIFTCONST 100
#define RETINA_SIZE 128
#define FEATUR_SIZE 32
#define ADDRESS 0x40000
#define X_MASK 0x000000FE
#define X_MASK_DEC    254
#define X_SHIFT 1
#define Y_MASK 0x00007F00
#define Y_MASK_DEC  32512    
#define Y_SHIFT 8
#define POLARITY_MASK 0x00000001
#define POLARITY_SHIFT 0
#define CONST_DECREMENT 5

#define CHUNKSIZE 32768

inline int convertChar2Dec(char value) {
    if (value > 60)
        return value - 97 + 10;
    else
        return value - 48;
}

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

efExtractorThread::efExtractorThread() : RateThread(THRATE) {
    resized = false;
    count           = 0;
    countEvent      = 0;
    leftInputImage  = 0;
    rightInputImage = 0;
    shiftValue      = 20;

    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
}

efExtractorThread::~efExtractorThread() {
    free(bufferCopy);
}

bool efExtractorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outLeftPort.open(getName("/edgesLeft:o").c_str());
    outRightPort.open(getName("/edgesRight:o").c_str());
    inLeftPort.open(getName("/left:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    outEventPort.open(getName("/event:o").c_str());

    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->open(getName("/retina:i").c_str());

    int SIZE_OF_EVENT = CHUNKSIZE >> 3; // every event is composed by 4bytes address and 4 bytes timestamp
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);

    bufferFEA = (aer *)  malloc(monBufSize_b);
    if ( bufferFEA == NULL ) {
        printf("bufferFEA malloc failed \n");
    }
    else {
        printf("bufferFEA successfully created \n");
    }

    eventFeaBuffer = new AER_struct[CHUNKSIZE>>3];

    printf("allocating memory for the LUT \n");
    lut = new int[RETINA_SIZE * RETINA_SIZE];
    printf("initialising memory for LUT \n");
    for(int i = 0; i < RETINA_SIZE * RETINA_SIZE; i++) {
        lut[i] = -1;
        //printf("i: %d  value : %d \n",i,lut[i]);
    } 
    printf("successfully initialised memory for LUT \n");
    
    /*opening the file of the mapping and creating the LUT*/
    pFile = fopen (mapURL.c_str(),"rb");    
    fout  = fopen ("lut.txt","w+");
    if (pFile!=NULL) {
        long lSize;
        size_t result;        
        // obtain file size:
        fseek (pFile , 0 , SEEK_END);
        lSize = ftell (pFile);
        printf("dimension of the file %lu \n", lSize);
        rewind(pFile);
        // saving into the buffer
        char * buffer;
        buffer = (char*) malloc (sizeof(char) * lSize);
        //fputs ("fopen example",pFile);
        printf("The file was correctly opened \n");
        result = fread (buffer,1,lSize,pFile);        
        printf(" Saved the data of the file into the buffer lSize:%lu \n", lSize);
        // checking the content of the buffer
        long word = 0;
        long input = -1, output = -1;
        short x, y;
        
        for (int i = 0; i < lSize; i++) {            
            
            int value = convertChar2Dec(*buffer);
            //loking for EOL
            if(*buffer == 10) {
                //sac words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }

                x = word & 0x001F;
                y = word >> 5;
                //printf("sac output: %d --> %d %d \n", word, x, y);
                output = y * 32 + x;
                word = 0;
            }
            //looking for space
            else if(*buffer == 32)  {
                //angel words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }                
                
                x = (word & X_MASK) >> X_SHIFT;
                y = (word & Y_MASK) >> Y_SHIFT;
                //printf("angel input: %d --> %d %d \n", word, x, y);
                input = y * 128 + x;
                word = 0;
            }
            else{
                //printf("%d,%d ", value, word );
                word = word << 4;
                word += value;
            }
            buffer++;
            if((input != -1) && (output!=-1)) {
                printf("lut : %d-->%d \n", input, output);
                fprintf(fout,"%d %d \n", input, output);
                lut[input] = output;
                input  = -1;
                output = -1;
            }            
        }
    }
    
    leftInputImage = new ImageOf<PixelMono>;
    leftInputImage->resize(FEATUR_SIZE, FEATUR_SIZE);
    //initialisation of the memory image
    unsigned char* pLeft = leftInputImage->getRawImage();
    int rowsize = leftInputImage->getRowSize();
    for(int r = 0 ; r < FEATUR_SIZE ; r++){
        for(int c = 0 ; c < FEATUR_SIZE ; c++){
            pLeft[r * rowsize + c] = 127;
        }
    }
    
    printf("initialisation correctly ended \n");
    return true;
}

void efExtractorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string efExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void efExtractorThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    //leftInputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(width, height);
    //rightInputImage = new ImageOf<PixelMono>;
    //rightInputImage->resize(width, height);
}

void efExtractorThread::run() {   
    count++;
    countEvent = 0;
    //printf("counter %d \n", count);
    bool flagCopy;
    if(!idle) {
        
        ImageOf<PixelMono> &left = outLeftPort.prepare();  
        left.resize(FEATUR_SIZE, FEATUR_SIZE);
        unsigned char* pLeft = left.getRawImage(); 
        
        //left.zero();
        
        //for(int r = 0 ; r < FEATUR_SIZE ; r++){
        //    for(int c = 0 ; c < FEATUR_SIZE ; c++){
        //        left(r,c) = 127;
        //    }
        //}
        
        // reads the buffer received
        // saves it into a working buffer        
        //printf("returned 0x%x 0x%x \n", bufferCopy, flagCopy);
        cfConverter->copyChunk(bufferCopy);
        //int unreadDim = selectUnreadBuffer(bufferCopy, flagCopy, resultCopy);

        //printf("Unmasking events:  %d \n", unreadDim);
        // extract a chunk/unmask the chunk       
        unmask_events.unmaskData(bufferCopy,CHUNKSIZE,eventFeaBuffer);
        
        //storing the response in a temp. image
        AER_struct* iterEvent = eventFeaBuffer;
        
        unsigned char* pMemL  = leftInputImage->getRawImage();

        unsigned long ts;
        short pol;
        aer* bufferFEA_copy = bufferFEA;
        // visiting a discrete number of events
        for(int i = 0; i < CHUNKSIZE>>3; i++ ) {
            ts  = iterEvent->ts;
            pol = iterEvent->pol;
            //printf(" %d %d \n",iterEvent->x,iterEvent->y );
            int x = RETINA_SIZE - iterEvent->x;
            int y = RETINA_SIZE - iterEvent->y;
            //printf("pointing in the lut at position %d \n", x + y * RETINA_SIZE);
            int pos = lut[x + y * RETINA_SIZE];            

            if(pos == -1) {
                //printf("not mapped event %d \n",x + y * RETINA_SIZE);
            }
            else {
                //creating an event
                short xevent_tmp   = pos / FEATUR_SIZE;
                short xevent       = FEATUR_SIZE - xevent_tmp;
                short yevent_tmp   = pos - xevent_tmp * FEATUR_SIZE;
                short yevent       = yevent_tmp;
                short polevent     = pol;
                short cameraevent  = 0;
                unsigned long blob = 0;
                //if(ts!=0) {
                //    printf(" %d %d %d %08x %08x \n",pos, yevent, xevent,blob, ts);
                //}
                unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);

                /*
                bufferFEA_copy->address   = (u32) blob;
                bufferFEA_copy->timestamp = (u32) ts;                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                bufferFEA_copy++; // jumping to the next event(u32,u32)
                countEvent++;
                */
                
                
                if (outLeftPort.getOutputCount()){
                        
                    int padding    = left.getPadding();
                    int rowsizeOut = left.getRowSize();
                    float lambda   = 0.6;
                    int deviance   = 40;
                    //creating the debug image
                    if((pol > 0)&&(pLeft[pos] <= 215)){
                        //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
                        pMemL[pos] = (1 - lambda) * pMemL[pos] + lambda * (pMemL[pos] + deviance);
                        pLeft[pos] = pMemL[pos];
                        if(pMemL[pos] > 152) {
                            bufferFEA_copy->address   = (u32) blob;
                            bufferFEA_copy->timestamp = (u32) ts;                
                            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                            bufferFEA_copy++; // jumping to the next event(u32,u32)
                            countEvent++;
                        }
                    }
                    if((pol<0)&&(pLeft[pos] >= 40)){ 
                        //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
                        pMemL[pos] = (1 - lambda) * pMemL[pos] + lambda * (pMemL[pos] - deviance);
                        pLeft[pos] = pMemL[pos];
                        if(pMemL[pos] < 102) {
                            bufferFEA_copy->address   = (u32) blob;
                            bufferFEA_copy->timestamp = (u32) ts;                
                            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                            bufferFEA_copy++; // jumping to the next event(u32,u32)
                            countEvent++; 
                        }
                    }
                } //end of outLeftPort.getOutputCount
                iterEvent++;            
            } //end of if
        } //end of for
        
        
        // leaking section of the algorithm    
        //pLeft = left.getRawImage();
        
        pMemL  = leftInputImage->getRawImage();
        pLeft  = left.getRawImage();
        int padding = leftInputImage->getPadding();
        for (int i = 0; i< FEATUR_SIZE * FEATUR_SIZE; i++) {
            if(*pMemL >= 127 + CONST_DECREMENT) {                
                *pMemL -= CONST_DECREMENT;
            }
            else if(*pMemL <= 127 - CONST_DECREMENT) {
                *pMemL += CONST_DECREMENT;
            }
            
            pMemL++;
            *pLeft = *pMemL;
            pLeft++;
        }
        pMemL += padding;                
        pLeft += padding;
        
        outLeftPort.write();


        //building feature events
        char* buffer = (char*) bufferFEA;
        int sz = countEvent * sizeof(aer);
        // sending events 
        if (outEventPort.getOutputCount()) {            
            eventBuffer data2send(buffer,sz);    
            eventBuffer& tmp = outEventPort.prepare();
            tmp = data2send;
            outEventPort.write();
        }
    } //end of idle
}

void efExtractorThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    inLeftPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
}

void efExtractorThread::threadRelease() {
    /* closing the ports*/
    outLeftPort.close();
    outRightPort.close();
    inLeftPort.close();
    inRightPort.close();
    outEventPort.close();

    delete[] eventFeaBuffer;
    delete bufferFEA;
    delete cfConverter;
    
    /* closing the file */
    delete[] lut;
    fclose (pFile);
}

