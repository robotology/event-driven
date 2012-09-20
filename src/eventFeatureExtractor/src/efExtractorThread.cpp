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

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cstring>
#include <cassert>
#include <iomanip>
#include <string>


using namespace emorph::ecodec;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define DIM 10
#define THRATE 5
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
#define CONST_DECREMENT 1

#define CHUNKSIZE 32768

//#define VERBOSE

inline void printPacket(const Bottle &packets)
{    
    for (int i=0; i<packets.size(); i++)
    {
        cout<<hex<<setw(8)<<setfill('0')
            <<packets.get(i).asInt()<<" ";
    }
    cout<<dec<<endl;
}

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
    firstHalf =  true;
    resized   = false;
    idle      = false;

    count             = 0;
    countEvent        = 0;
    leftInputImage    = 0;
    rightInputImage   = 0;
    shiftValue        = 20;
    lastTimestampLeft = 0;

    bufferCopy  = (char*) malloc(CHUNKSIZE);
    bufferCopy2 = (char*) malloc(CHUNKSIZE);
}

efExtractorThread::~efExtractorThread() {
    free(bufferCopy);
}

bool efExtractorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outLeftPort    .open(getName("/left:o").c_str());
    outRightPort   .open(getName("/right:o").c_str());
    outFeaLeftPort .open(getName("/feaLeft:o").c_str());
    outFeaRightPort.open(getName("/feaRight:o").c_str());
    inLeftPort     .open(getName("/left:i").c_str());
    inRightPort    .open(getName("/right:i").c_str());
    outEventPort   .open(getName("/event:o").c_str());
    outBottlePort  .open(getName("/eventBottle:o").c_str());
    
    ebHandler = new eventBottleHandler();
    ebHandler->setVerbose(VERBOSE);
    ebHandler->useCallback();
    ebHandler->open(getName("/retinaBottle:i").c_str());

    receivedBottle = new Bottle();
    bottleToSend   = new Bottle();

    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->setVERBOSE(VERBOSE);
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
    lut = new int[RETINA_SIZE * RETINA_SIZE * 5];
    printf("initialising memory for LUT \n");

    for(int i = 0; i < RETINA_SIZE * RETINA_SIZE * 5; i++) {
        lut[i] = -1;
        //printf("i: %d  value : %d \n",i,lut[i]);
    }
    printf("successfully initialised memory for LUT \n");
    
    /*opening the file of the mapping and creating the LUT*/
       
    fout  = fopen ("lut.txt","w+");
    fdebug  = fopen ("./eventFeatureExtractor.dumpSet.txt","w+");
    pFile = fopen (mapURL.c_str(),"rb");  
    if (pFile == NULL) {
        printf("file of mapping was not found. The module terminates \n");
        return false;
    }
    else {
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
        countMap = 0;

        //lSize
        for (int i = 0; i < lSize; i++) {            
            
            int value = convertChar2Dec(*buffer);
            //looking for EOL
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
                
                int inputy  = input / 128;
                int inputx  = input - inputy * 128;
                int outputy = output / 32;
                int outputx = output - outputy * 32;
                
                bool continueSaving = true;

                // any input coordinate can point up to 5 output coordinates
                int i = 0;
                while(continueSaving) {
                    if(lut[input + i * RETINA_SIZE * RETINA_SIZE] != -1) {
                        if(lut[input + i * RETINA_SIZE * RETINA_SIZE ]!= output) { 
                            i++;
                            if (i>= 5) {
                                continueSaving = false;
                            }                                                      
                        }
                        else {
                            continueSaving = false;
                        }
                    }
                    else {
                        //saving
                        lut[input + i * RETINA_SIZE * RETINA_SIZE] = output;
                        //printf("lut : %ld-->%ld (%d) \n", input, output, i);
                        fprintf(fout," %ld %ld > %d %d > %d %d   \n",input, output, inputy, inputx,  outputy, outputx);
                        input  = -1;
                        output = -1;
                        continueSaving = false;
                        countMap++;
                    }
                } //end of while
            }            
        }
        printf("counted the number of mapping %d \n", countMap);
        
    }
     
    leftInputImage      = new ImageOf<PixelMono>;
    leftOutputImage     = new ImageOf<PixelMono>;
    rightOutputImage    = new ImageOf<PixelMono>;
    leftFeaOutputImage  = new ImageOf<PixelMono>;
    rightFeaOutputImage = new ImageOf<PixelMono>;

    leftInputImage     ->resize(RETINA_SIZE,RETINA_SIZE);
    leftOutputImage    ->resize(RETINA_SIZE,RETINA_SIZE);
    rightOutputImage   ->resize(RETINA_SIZE,RETINA_SIZE);
    leftFeaOutputImage ->resize(FEATUR_SIZE,FEATUR_SIZE);
    rightFeaOutputImage->resize(FEATUR_SIZE,FEATUR_SIZE);
    
    int padding = leftInputImage->getPadding();
    //initialisation of the memory image
    unsigned char* pLeft     = leftInputImage->getRawImage();
    unsigned char* pLeftOut  = leftOutputImage->getRawImage();
    unsigned char* pRightOut = rightOutputImage->getRawImage();
    
    // assign 127 to all the location in image plane
    int rowsize = leftInputImage->getRowSize();
    for(int r = 0 ; r < RETINA_SIZE ; r++){
        for(int c = 0 ; c < RETINA_SIZE ; c++){
            *pLeft  = 127; pLeft++;
            *pLeftOut = 127; pLeftOut++;
            *pRightOut = 127; pRightOut++;
        }
        pLeft +=  padding;
        pLeftOut += padding;
        pRightOut += padding;
    }
    
    unsigned char* pFeaLeftOut = leftFeaOutputImage->getRawImage();
    unsigned char* pFeaRightOut = rightFeaOutputImage->getRawImage();
    memset(pFeaLeftOut,  127, FEATUR_SIZE * FEATUR_SIZE * sizeof(unsigned char));
    memset(pFeaRightOut, 127, FEATUR_SIZE * FEATUR_SIZE * sizeof(unsigned char));

    txQueue = new eEventQueue();
    rxQueue = new eEventQueue();
    
    printf("initialisation correctly ended \n");
    return true;
}

void efExtractorThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    outFeaLeftPort.interrupt();
    outFeaRightPort.interrupt();
    inLeftPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
    outBottlePort.interrupt();
}

void efExtractorThread::threadRelease() {
    /* closing the ports*/
    printf("efExtractorThread::closing the ports \n");
    outLeftPort.close();
    outRightPort.close();
    outFeaLeftPort.close();
    outFeaRightPort.close();
    inLeftPort.close();
    inRightPort.close();
    outEventPort.close();
    outBottlePort.close();

    printf("efExtractorThread::deleting the buffer of events \n");
    delete[] eventFeaBuffer;
    delete bufferFEA;
    printf("efExtractorThread::deleting event bottle Handler \n");
    if(ebHandler!=0) {
        printf("ebHandler !NULL \n");
        ebHandler->close();
    }
    //delete ebHandler;
    printf("efExtractorThread::deleting cartesianFrame converter \n");
    //cfConverter->close();
    //delete cfConverter;

    
    delete receivedBottle;
    delete bottleToSend;
    //delete leftFeaOutputImage;
    //delete rightFeaOutputImage;

    /*
    printf("efExtractorThread::delete transmit and received queue \n");
    if(txQueue != 0) {
        delete txQueue;
        printf("efExtractorThread::threadRelease correctly  deleted the transmit queue \n");
    }
    
    if(rxQueue != 0) {
        delete rxQueue;
        printf("efExtractorThread::threadRelease correctly deleted the receiver queue \n");
    }
    */
    
    /* closing the file */
    printf("efExtractorThread::deleting LUT \n");
    if(lut!=NULL) {
        delete[] lut;
    }
    
    printf("efExtractorThread::closing the file \n");
    fclose (pFile);

    printf("efExtractorThread::threadReleas : success in releasing \n ");
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

void efExtractorThread::remapEventLeft(int x, int y,short pol,unsigned long ts) {
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = leftFeaOutputImage->getRowSize();
    int deviance      = 1;
    int devianceFea   = 5;   
    aer* bufferFEA_copy = bufferFEA;

    if(true) {
        //printf("entering in efExtractorThread::remapEventLeft \n");

        //countEventLeft++;
        /*if(firstHalf){
            if(ts < 0x807FFFFF) {
                lastTimestampLeft = ts;
            }
            //else {
            //    printf("second half timestamp in first %08x %08x \n",ts,lastTimestampLeft  );
            //}
        }
        else {
            lastTimestampLeft = ts;
        }
        */
        
        
        //printf(" %d %d %08x \n",iterEvent->x,iterEvent->y, ts );
        //int x = RETINA_SIZE - iterEvent->x - 1 ;
        //int y = iterEvent->y  ;

        //if((x >127)||(x<0)||(y>127)||(y<0)) break;
        int posImage     = y * rowSize + x;
        /*
        for (int i = 0; i< 5 ; i++) {                
            int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  y * RETINA_SIZE + x ];     
            
            //printf("        pos ; %d ", pos);
            if(pos == -1) {
                if (x % 2 == 1) {
                    //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                    //countUnmapped++;
                }                
            }
            else {
                //creating an event                            
                int yevent_tmp   = pos / FEATUR_SIZE;
                int yevent       = FEATUR_SIZE - yevent_tmp;
                int xevent_tmp   = pos - yevent_tmp * FEATUR_SIZE;
                int xevent       = xevent_tmp;
                int polevent     = pol < 0 ? 0 : 1;
                int cameraevent  = 1;
                unsigned long blob = 0;
                int posFeaImage     = yevent * rowSizeFea + xevent ;

                unsigned char* pFeaLeft = leftFeaOutputImage->getRawImage();
                if(polevent > 0) {
                    if(pFeaLeft[posFeaImage] <= 255 - devianceFea) {
                        pFeaLeft[posFeaImage] += devianceFea ;
                    }
                    else {
                        pFeaLeft[posFeaImage] = 255;
                    }
                }
                else {
                    if(pFeaLeft[posFeaImage] >= devianceFea) {
                        pFeaLeft[posFeaImage] -= devianceFea ;
                    }
                    else {
                        pFeaLeft[posFeaImage] = 0;
                    }
                }
                
                //if(ts!=0) {
                //    printf(" %d %d %d ---> ", i ,x, y  );
                //    printf(" %d %d %d %d %08x %08x \n",pos, yevent,xevent,posFeaImage,blob, ts);
                //}
                unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
                
                //blob = 0x00001021;
                //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, blob, ts);
                
                // sending the event if we pass thresholds
                unsigned long evPU;
                evPU = 0;
                evPU = evPU | polevent;
                evPU = evPU | (xevent << 1);
                evPU = evPU | (yevent << 8);
                evPU = evPU | (cameraevent << 15);                            
                blob = blob & 0x0000FFFF;                            
                
                if(pFeaLeft[posFeaImage] > 240) {
                    bufferFEA_copy->address   = (u32) blob;
                    bufferFEA_copy->timestamp = (u32) ts;
                    //#ifdef VERBOSE
                    //fprintf(fdebug,"%08X %08X \n",ts,blob);  
                    //#endif
                    
                    //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                    bufferFEA_copy++; // jumping to the next event(u32,u32)
                    // countEventToSend++;
                }
                else if(pFeaLeft[posFeaImage] < 10) {
                    bufferFEA_copy->address   = (u32) blob;
                    bufferFEA_copy->timestamp = (u32) ts;
                    //#ifdef VERBOSE
                    //fprintf(fdebug,"%08X %08X \n",ts,blob);  
                    //#endif
                    
                    //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                    bufferFEA_copy++; // jumping to the next event(u32,u32)
                    //countEventToSend++;
                }                           
                
            } //end else
            
        } //end for i 5

        */
        
        if (outLeftPort.getOutputCount()){
            
            int padding    = leftOutputImage->getPadding();
            int rowsizeOut = leftOutputImage->getRowSize();
            float lambda   = 1.0;
            
            //creating the debug image
            if(pol > 0){
                
                if(pLeft[posImage] <= (255 - deviance)) {
                    //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
                    pLeft[posImage] =  pLeft[posImage] + deviance;
                }
                else {
                    pLeft[posImage] = 255;
                }
                //pMemL[pos] = (int)((1 - lambda) * (double) pMemL[pos] + lambda * (double) (pMemL[pos] + deviance));
                //pLeft[posImage] = pMemL[posImage];
    
            }
            if(pol<0){ 
                //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
                //pMemL[pos] = (int) ((1 - lambda) * (double) pMemL[pos] + lambda * (double)(pMemL[pos] - deviance));
                
                if(pLeft[posImage] >= deviance) {
                    pLeft[posImage] =  pLeft[posImage] - deviance;
                }
                else {
                    pLeft[posImage] = 0;
                }
                //pLeft[posImage] = pMemL[posImage];
                
                //  if(pMemL[pos] < 127 - 70) {
                //  bufferFEA_copy->address   = (u32) blob;
                //  bufferFEA_copy->timestamp = (u32) ts;                
                //  #ifdef VERBOSE
                //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
                //  #endif                                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                //  bufferFEA_copy++; // jumping to the next event(u32,u32)
                // countEvent++; 
                // }
                
            }
            
            
        } //end of if outLeftPort
                
        //printf("pointing in the lut at position %d %d %d \n",x, y, x + y * RETINA_SIZE);
        // extra output positions are pointed by i
        
    } // end if ts > lasttimestamp
}

void efExtractorThread::remapEventRight(int x, int y, short pol, unsigned long ts) {
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = leftFeaOutputImage->getRowSize();
    int deviance      = 20;
    int devianceFea   = 20;   
    aer* bufferFEA_copy = bufferFEA;
    lastTimestampRight = ts;

    // finding the position in the map
    int posImage     = y * rowSize + x;
    
    
    for (int i = 0; i< 5 ; i++) {                
        int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  y * RETINA_SIZE + x ];                      
        
        //printf("        pos ; %d ", pos);
        if(pos == -1) {
            if (x % 2 == 1) {
                //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                //countUnmapped++;
            }                
        }
        else {
            //creating an event
            
            int xevent_tmp   = pos / FEATUR_SIZE;
            int xevent       = FEATUR_SIZE - xevent_tmp;
            int yevent_tmp   = pos - xevent_tmp * FEATUR_SIZE;
            int yevent       = yevent_tmp;
            int polevent     = pol < 0 ? 0 : 1;
            int cameraevent  = 1;
            unsigned long blob = 0;
            int posFeaImage     = y * rowSizeFea + x;
            
            //if(ts!=0) {
            //    printf(" %d %d %d %08x %08x \n",pos, yevent, xevent,blob, ts);
            //}
            unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
            //unsigned long evPU;
            //evPU = 0;
            //evPU = evPU | polevent;
            //evPU = evPU | (xevent << 1);
            //evPU = evPU | (yevent << 8);
            //evPU = evPU | (cameraevent << 15);
            
            //blob = blob & 0x0000FFFF;
            //blob = 0x00001021;
            //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, blob, ts);
            
            
            //unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();
            //pFeaRight[posFeaImage] = 255;
            
        } //end else
        
    } //end for i 5
    if (outRightPort.getOutputCount()){
        
        int padding    = rightOutputImage->getPadding();
        int rowsizeOut = rightOutputImage->getRowSize();
        float lambda   = 1.0;
        //creating the debug image
        if(pol > 0){
            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
            if(pRight[posImage] <= (255 - deviance)) {
                pRight[posImage] =  pRight[posImage] + deviance;
            }
            else {
                pRight[posImage] = 255;
            }
            //pMemL[pos] = (int)((1 - lambda) * (double) pMemL[pos] + lambda * (double) (pMemL[pos] + deviance));
            //pLeft[posImage] = pMemL[posImage];
            
            //  if(pMemL[pos] > 127 + 70) {
            //  bufferFEA_copy->address   = (u32) blob;
            //  bufferFEA_copy->timestamp = (u32) ts;
            //  #ifdef VERBOSE
            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
            //  #endif
            
            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
            //  countEvent++;
            // }
            
        }
        if(pol<0){ 
            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
            //pMemL[pos] = (int) ((1 - lambda) * (double) pMemL[pos] + lambda * (double)(pMemL[pos] - deviance));
            if(pRight[posImage] >= deviance) {
                pRight[posImage] =  pRight[posImage] - deviance;
            }
            else {
                pRight[posImage] = 0;
            }
            //pLeft[posImage] = pMemL[posImage];
            
            //  if(pMemL[pos] < 127 - 70) {
            //  bufferFEA_copy->address   = (u32) blob;
            //  bufferFEA_copy->timestamp = (u32) ts;                
            //  #ifdef VERBOSE
            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
            //  #endif                                
            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
            // countEvent++; 
            // }
            
        }                                        
    } //end of if outLeftPort          
}



void efExtractorThread::remapEventLeft(int x, int y,short pol,unsigned long ts, eEventQueue* txQueue) {
    //printf("efExtractorThread::remapEventLeft %08x size %d \n", txQueue, txQueue->size());
    for (int i=0; i<txQueue->size(); i++)  {
        printf("%d : \n", i);
        cout<<txQueue->at(i)->getContent().toString().c_str()<<endl;
    }

    
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = leftFeaOutputImage->getRowSize();
    int deviance      = 20;
    int devianceFea   = 20;   
    aer* bufferFEA_copy = bufferFEA;

    //eEventQueue txQueue(false);
    int countEventToSend = 0;

    //if(ts > lastTimestampLeft) {
    //countEventLeft++;
    /*if(firstHalf){
      if(ts < 0x807FFFFF) {
      lastTimestampLeft = ts;
      }
      //else {
      //    printf("second half timestamp in first %08x %08x \n",ts,lastTimestampLeft  );
      //}
      }
      else {
      lastTimestampLeft = ts;
      }
    */    
    
    //printf(" %d %d %08x \n",iterEvent->x,iterEvent->y, ts );
    //int x = RETINA_SIZE - iterEvent->x - 1 ;
    //int y = iterEvent->y  ;
    
    //if((x >127)||(x<0)||(y>127)||(y<0)) break;
    int posImage     = y * rowSize + x;
    /*
    for (int i = 0; i< 5 ; i++) {                
        int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  y * RETINA_SIZE + x ];     
        
        //printf("        pos ; %d ", pos);
        if(pos == -1) {
            if (x % 2 == 1) {
                //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                //countUnmapped++;
            }                
        }
        else {
            //creating an event                            
            int yevent_tmp   = pos / FEATUR_SIZE;
            int yevent       = FEATUR_SIZE - yevent_tmp;
            int xevent_tmp   = pos - yevent_tmp * FEATUR_SIZE;
            int xevent       = xevent_tmp;
            //int polevent     = pol < 0 ? 0 : 1;
            int polevent = pol;
            int cameraevent  = 1;
            unsigned long blob = 0;
            int posFeaImage     = yevent * rowSizeFea + xevent ;
            
            //if(ts!=0) {
            //    printf(" %d %d %d ---> ", i ,x, y  );
            //    printf(" %d %d %d %d %08x %08x \n",pos, yevent,xevent,posFeaImage,blob, ts);
            //}
            unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
            
            //blob = 0x00001021;
            //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, (unsigned int) blob, (unsigned int)ts);
            
            // representing the feature map for debug
            unsigned char* pFeaLeft = leftFeaOutputImage->getRawImage();
            if(polevent > 0) {
                if(pFeaLeft[posFeaImage] <= 255 - devianceFea) {
                    pFeaLeft[posFeaImage] += devianceFea ;
                }
                else {
                    pFeaLeft[posFeaImage] = 255;
                }
            }
            else {
                if(pFeaLeft[posFeaImage] >= devianceFea) {
                    pFeaLeft[posFeaImage] -= devianceFea ;
                }
                else {
                    pFeaLeft[posFeaImage] = 0;
                }
            }
            
            
            
            // sending the event if we pass thresholds
             
            //printf("checking for thresholds \n");
            if(pFeaLeft[posFeaImage] > 240) {
                
                
                TimeStamp timestamp;
                timestamp.setStamp(ts);
                txQueue->push_back(&timestamp);
                countEventToSend++;
                
                AddressEvent ae;
                ae.setChannel(1);
                ae.setPolarity(1);
                ae.setX(xevent);
                ae.setY(yevent);
                txQueue->push_back(&ae);
                countEventToSend++;
                

                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                //bufferFEA_copy++; // jumping to the next event(u32,u32)

            }
            else if(pFeaLeft[posFeaImage] < 10) {
                
                TimeStamp timestamp;
                timestamp.setStamp(ts);
                txQueue->push_back(&timestamp);
                countEventToSend++;
                
                AddressEvent ae;
                ae.setChannel(1);
                ae.setPolarity(0);
                ae.setX(xevent);
                ae.setY(yevent);
                txQueue->push_back(&ae);
                countEventToSend++;
                

                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                //bufferFEA_copy++; // jumping to the next event(u32,u32)
                
            }                           
            
        } //end else
        
    } //end for i 5

    */
    
    // passing all the events to the bottle
    //printf("trying to passing %d events left \n", txQueue.size());
    //for(size_t j = 0 ; j < txQueue.size(); j++) {
    //    bottle->append(txQueue[j]->encode());
    //}
    


    for (int i=0; i<txQueue->size(); i++)  {
        printf("%d :   ", i);
        cout<<txQueue->at(i)->getContent().toString().c_str()<<endl;
    }
    
    /*
    if (outLeftPort.getOutputCount()){
        
        int padding    = leftOutputImage->getPadding();
        int rowsizeOut = leftOutputImage->getRowSize();
        float lambda   = 1.0;
        
        //creating the debug image
        if(pol){
            if(pLeft[posImage] <= (255 - deviance)) {
                //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
                pLeft[posImage] =  pLeft[posImage] + deviance;
            }
            else {
                pLeft[posImage] = 255;
            }
            //pMemL[pos] = (int)((1 - lambda) * (double) pMemL[pos] + lambda * (double) (pMemL[pos] + deviance));
            //pLeft[posImage] = pMemL[posImage];                         
        }
        else { 
            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
            //pMemL[pos] = (int) ((1 - lambda) * (double) pMemL[pos] + lambda * (double)(pMemL[pos] - deviance));
            if(pLeft[posImage] >= deviance) {
                pLeft[posImage] =  pLeft[posImage] - deviance;
            }
            else {
                pLeft[posImage] = 0;
            }
            //pLeft[posImage] = pMemL[posImage];
            
            //  if(pMemL[pos] < 127 - 70) {
            //  bufferFEA_copy->address   = (u32) blob;
            //  bufferFEA_copy->timestamp = (u32) ts;                
            //  #ifdef VERBOSE
            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
            //  #endif                                
            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
            // countEvent++; 
            // }
            
        }
        
        
    } //end of if outLeftPort
    
    */
    
    //printf("pointing in the lut at position %d %d %d \n",x, y, x + y * RETINA_SIZE);
    // extra output positions are pointed by i
    
    //} // end if ts > lasttimestamp
}




void efExtractorThread::remapEventRight(int x, int y, short pol, unsigned long ts, eEventQueue* txQueue) {
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = rightFeaOutputImage->getRowSize();
   
    int deviance          = 20;
    int devianceFea       = 20;   
    aer* bufferFEA_copy   = bufferFEA;
    lastTimestampRight    = ts;

    // finding the position in the map
    int posImage     = y * rowSize + x;
     
    //eEventQueue txQueue(false);
    int countEventToSend = 0;
    
    //***********************************************************************************************
    // looking at the 5 columns of the LUT to remap event 
    for (int i = 0; i< 5 ; i++) {                
        int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  y * RETINA_SIZE + x ];                      
        
        //printf("        pos ; %d ", pos);
        if(pos == -1) {
            if (x % 2 == 1) {
                //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                //countUnmapped++;
            }                
        }
        else {
            //creating an event            
            int yevent_tmp   = pos / FEATUR_SIZE;
            int yevent       = FEATUR_SIZE - yevent_tmp;
            int xevent_tmp   = pos - yevent_tmp * FEATUR_SIZE;
            int xevent       = xevent_tmp;
            //int polevent     = pol < 0 ? 0 : 1;
            int polevent = pol;
            int cameraevent  = 1;
            unsigned long blob = 0;
            int posFeaImage     = yevent * rowSizeFea + xevent;
            
            //if(ts!=0) {
            //    printf(" %d %d %d %08x %08x \n",pos, yevent, xevent,blob, ts);
            //}
            unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
            //unsigned long evPU;
            //evPU = 0;
            //evPU = evPU | polevent;
            //evPU = evPU | (xevent << 1);
            //evPU = evPU | (yevent << 8);
            //evPU = evPU | (cameraevent << 15);
            
            //blob = blob & 0x0000FFFF;
            //blob = 0x00001021;
            //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, blob, ts);
            
            
            // representing the feature map for debug
            unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();

            
            
            if(polevent > 0) {
                if(pFeaRight[posFeaImage] <= 255 - devianceFea) {
                    pFeaRight[posFeaImage] += devianceFea ;
                }
                else {
                    pFeaRight[posFeaImage] = 255;
                }
            }
            else {
                if(pFeaRight[posFeaImage] >= devianceFea) {
                    pFeaRight[posFeaImage] -= devianceFea ;
                }
                else {
                    pFeaRight[posFeaImage] = 0;
                }
            }
            
            
            // sending the event if we pass thresholds
            //unsigned long evPU;
            //evPU = 0;
            //evPU = evPU | polevent;
            //evPU = evPU | (xevent << 1);
            //evPU = evPU | (yevent << 8);
            //evPU = evPU | (cameraevent << 15);                            
            //blob = blob & 0x0000FFFF;                            
            
            
            if(pFeaRight[posFeaImage] > 240) {
                
                TimeStamp timestamp;
                timestamp.setStamp(ts);
                txQueue->push_back(&timestamp);
                countEventToSend++;
                
                AddressEvent ae;
                ae.setChannel(0);
                ae.setPolarity(1);
                ae.setX(xevent);
                ae.setY(yevent);
                txQueue->push_back(&ae);
                countEventToSend++;
                
                //  bufferFEA_copy->address   = (u32) blob;
                //  bufferFEA_copy->timestamp = (u32) ts;
                
                
                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                //bufferFEA_copy++; // jumping to the next event(u32,u32)
                // countEventToSend++;
            }
            else if(pFeaRight[posFeaImage] < 10) {
                
                TimeStamp timestamp;
                timestamp.setStamp(ts);
                txQueue->push_back(&timestamp);
                countEventToSend++;
                
                AddressEvent ae;
                ae.setChannel(0);
                ae.setPolarity(0);
                ae.setX(xevent);
                ae.setY(yevent);
                txQueue->push_back(&ae);
                countEventToSend++;
                
                
                // bufferFEA_copy->address   = (u32) blob;
                //  bufferFEA_copy->timestamp = (u32) ts;
                
                
                //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                //bufferFEA_copy++; // jumping to the next event(u32,u32)
                //countEventToSend++;
            }                           
            
            
            //unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();
            //pFeaRight[posFeaImage] = 255;
            
        } //end else
        
    } //end for i 5
    //******************************************************************************************************

    // passing all the events to the bottle
    //printf("trying to pass %d right \n", countEventToSend);
    //for(int j = 0 ; j < countEventToSend; j++) {
    //    bottle->append(txQueue[j]->encode());
    //}
    
    if (outRightPort.getOutputCount()){
        // representing remapping into an image
        int padding    = rightOutputImage->getPadding();
        int rowsizeOut = rightOutputImage->getRowSize();
        float lambda   = 1.0;
        //creating the debug image
        if(pol){
            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
            if(pRight[posImage] <= (255 - deviance)) {
                pRight[posImage] =  pRight[posImage] + deviance;
            }
            else {
                pRight[posImage] = 255;
            }
            //pMemL[pos] = (int)((1 - lambda) * (double) pMemL[pos] + lambda * (double) (pMemL[pos] + deviance));
            //pLeft[posImage] = pMemL[posImage];
            
            //  if(pMemL[pos] > 127 + 70) {
            //  bufferFEA_copy->address   = (u32) blob;
            //  bufferFEA_copy->timestamp = (u32) ts;
            //  #ifdef VERBOSE
            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
            //  #endif
            
            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
            //  countEvent++;
            // }
            
        }
        else { 
            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
            //pMemL[pos] = (int) ((1 - lambda) * (double) pMemL[pos] + lambda * (double)(pMemL[pos] - deviance));
            if(pRight[posImage] >= deviance) {
                pRight[posImage] =  pRight[posImage] - deviance;
            }
            else {
                pRight[posImage] = 0;
            }
            //pLeft[posImage] = pMemL[posImage];
            
            //  if(pMemL[pos] < 127 - 70) {
            //  bufferFEA_copy->address   = (u32) blob;
            //  bufferFEA_copy->timestamp = (u32) ts;                
            //  #ifdef VERBOSE
            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
            //  #endif                                
            //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
            // countEvent++; 
            // }
            
        }                                        
    } //end of if outLeftPort          
}

void efExtractorThread::generateMemory(int countEvent, int& countEventToSend) {
    int countEventLeft  = 0;
    int countEventRight = 0;
    printf("GenerateMemory without bottles %d  \n", countEvent);
    //storing the response in a temp. image
    AER_struct* iterEvent = eventFeaBuffer;  //<------------ using the pointer to the unmasked events!
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = leftFeaOutputImage->getRowSize();
    unsigned long ts;
    short pol, cam;
    aer* bufferFEA_copy = bufferFEA;
    // visiting a discrete number of events
    int countUnmapped = 0;
    int deviance      = 50;
    int devianceFea   = 50;      
    //#############################################################################        
    for(int i = 0; i < countEvent; i++ ) {
        ts  = iterEvent->ts;
        int x = iterEvent->x;
        int y = RETINA_SIZE - iterEvent->y;
        pol = iterEvent->pol;
        cam = iterEvent->cam;
        printf("x %d  y %d  pol %d cam %d \n", cam);
        // -------------------------- CAMERA 1 ----------------------------------
        if(cam == 0) {
            remapEventLeft(x,y,pol,ts);               
        } //end of if cam!=0
        // -------------------------- CAMERA 0 ----------------------------------
        else if(cam == 1) {
            countEventRight++;                               
            remapEventRight(x,y,pol,ts);
        }
        else {
            printf("error in the camera value %d \n", cam);
        }
        
        //printf("\n");
        iterEvent++;
    } //end for i
    //############################################################################
}


void efExtractorThread::generateMemory(eEventQueue *q, Bottle* packets, int& countEventToSend) {
    int countEventLeft  = 0;
    int countEventRight = 0;
    
    //storing the response in a temp. image
    AER_struct* iterEvent = eventFeaBuffer;  //<------------ using the pointer to the unmasked events!
    unsigned char* pLeft  = leftOutputImage->getRawImage(); 
    unsigned char* pRight = rightOutputImage->getRawImage();
    unsigned char* pMemL  = leftInputImage->getRawImage();
    int          rowSize  = leftOutputImage->getRowSize();
    int       rowSizeFea  = leftFeaOutputImage->getRowSize();
    //eEventQueue txQueue;
    
    
    unsigned long ts;
    short pol, cam;
    aer* bufferFEA_copy = bufferFEA;
    // visiting a discrete number of events
    int countUnmapped = 0;
    int deviance      = 20;
    int devianceFea   = 20;      
    //#############################################################################        
    int dequeSize = q->size();
    int readPosition = 0;
    
    for (int evt = 0; evt < dequeSize; evt++) {
        
        eEvent* eventInQueue = q->operator[](evt);
        if( (*q)[evt]!= 0) {                    
            //********** extracting the event information **********************
            // to identify the type of the packet
            // user can rely on the getType() method
            
            // -------------------------- AE  ----------------------------------
            string strcmpAE("AE");
            string strcmpTS("TS");
            string typeRef = q->operator[](evt)->getType();

            if(typeRef == "null!"){
                printf("efExtractorThread::generateMemory %s \n", typeRef.c_str());
                continue;
            }
            
 
            if (typeRef == strcmpAE) {
                // identified an  address event
                AddressEvent* ptr=dynamic_cast<AddressEvent*>(eventInQueue);
                if(ptr->isValid()) { 
                    //ts  = iterEvent->ts;
                    //pol = iterEvent->pol;
                    //cam = iterEvent->cam;
                    int cartY     = ptr->getX();
                    int cartX     = ptr->getY();
                    int camera    = ptr->getChannel();
                    int polarity  = ptr->getPolarity();
                    
                    if(VERBOSE & false) {
                        fprintf(fdebug, " %d %d %d %d \n",cartX, cartY, camera, polarity );
                    }
                    
                    //printf(" %d %d %d %d ",cartX, cartY, camera, polarity );
                    if((cartX == 0) && (cartY == 0) && (camera == 0) && (polarity == 0)) {
                        //printf("\n");
                    }
                    else {
                        
                        if(camera==0) {
                            //printf("remapping event left camera:%d \n", camera);
                            //printf("before remapping %08x \n", &txQueue);
                            //remapEventLeft(cartX,cartY,polarity,ts,&txQueue);  
                            int posImage     = cartY * rowSize + cartX;
                            
                            for (int i = 0; i< 5 ; i++) {                
                                int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  cartY * RETINA_SIZE + cartX ];     
                                
                                //printf("        pos ; %d ", pos);
                                if(pos == -1) {
                                    if (cartX % 2 == 1) {
                                        //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                                        //countUnmapped++;
                                    }                
                                }
                                else {
                                    //creating an event                            
                                    int yevent_tmp      = pos / FEATUR_SIZE;
                                    int yevent          = FEATUR_SIZE - yevent_tmp;
                                    int xevent_tmp      = pos - yevent_tmp * FEATUR_SIZE;
                                    int xevent          = xevent_tmp;
                                    //int polevent      = pol < 0 ? 0 : 1;
                                    int polevent        = polarity;
                                    int cameraevent     = 1;
                                    unsigned long blob  = 0;
                                    int posFeaImage     = yevent * rowSizeFea + xevent ;
                                    
                                    //if(ts!=0) {
                                    //    printf(" %d %d %d ---> ", i ,x, y  );
                                    //    printf(" %d %d %d %d %08x %08x \n",pos, yevent,xevent,posFeaImage,blob, ts);
                                    //}
                                    unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
                                    
                                    //blob = 0x00001021;
                                    //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, (unsigned int) blob, (unsigned int)ts);
                                    
                                    // representing the feature map for debug
                                    unsigned char* pFeaLeft = leftFeaOutputImage->getRawImage();
                                    if(polevent > 0) {
                                        if(pFeaLeft[posFeaImage] <= 255 - devianceFea) {
                                            pFeaLeft[posFeaImage] += devianceFea ;
                                        }
                                        else {
                                            pFeaLeft[posFeaImage] = 255;
                                        }
                                    }
                                    else {
                                        if(pFeaLeft[posFeaImage] >= devianceFea) {
                                            pFeaLeft[posFeaImage] -= devianceFea ;
                                        }
                                        else {
                                            pFeaLeft[posFeaImage] = 0;
                                        }
                                    }
                                    
                                    
                                    
                                    // sending the event if we pass thresholds
                                    
                                    //printf("checking for thresholds \n");
                                    if(pFeaLeft[posFeaImage] > 240) {
                                        
                                        TimeStamp* timestamp = new TimeStamp();
                                        timestamp->setStamp(ts);
                                        packets->append(timestamp->encode());
                                        //txQueue.push_back(timestamp);                                         
                                        delete timestamp;
                                        countEventToSend++;
                                        
                                        AddressEvent* ae = new AddressEvent();
                                        ae->setChannel(1);
                                        ae->setPolarity(1);
                                        ae->setX(xevent);
                                        ae->setY(yevent);
                                        packets->append(ae->encode());
                                        //txQueue.push_back(ae);
                                        delete ae;
                                        countEventToSend++;
                                        
                                        //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                                        //bufferFEA_copy++; // jumping to the next event(u32,u32)
                                        
                                    }
                                    else if(pFeaLeft[posFeaImage] < 10) {
                                        
                                        TimeStamp* timestamp = new TimeStamp();
                                        timestamp->setStamp(ts);
                                        //txQueue.push_back(timestamp);
                                        packets->append(timestamp->encode());
                                        delete timestamp;
                                        countEventToSend++;
                                        
                                        AddressEvent* ae = new AddressEvent();
                                        ae->setChannel(1);
                                        ae->setPolarity(0);
                                        ae->setX(xevent);
                                        ae->setY(yevent);
                                        packets->append(ae->encode());
                                        //txQueue.push_back(ae);
                                        delete ae;
                                        countEventToSend++;                                                                                  
                                        
                                        //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                                        //bufferFEA_copy++; // jumping to the next event(u32,u32)
                                        
                                    }                           
                                    
                                } //end else
                                
                            } //end for i 5 
                            
                        }
                        else { //------------------------------------------------------------------------------------------------------
                            //printf("remapping event right camera:%d \n", camera);
                            //remapEventRight(cartX,cartY,polarity,ts,&txQueue);
                            for (int i = 0; i< 5 ; i++) {                
                                int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  cartY * RETINA_SIZE + cartX ];                      
                                
                                //printf("        pos ; %d ", pos);
                                if(pos == -1) {
                                    if (cartX % 2 == 1) {
                                        //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                                        //countUnmapped++;
                                    }                
                                }
                                else {
                                    //creating an event            
                                    int yevent_tmp      = pos / FEATUR_SIZE;
                                    int yevent          = FEATUR_SIZE - yevent_tmp;
                                    int xevent_tmp      = pos - yevent_tmp * FEATUR_SIZE;
                                    int xevent          = xevent_tmp;
                                    //int polevent      = pol < 0 ? 0 : 1;
                                    int polevent        = polarity;
                                    int cameraevent     = 1;
                                    unsigned long blob  = 0;
                                    int posFeaImage     = yevent * rowSizeFea + xevent;
                                    
                                    //if(ts!=0) {
                                    //printf(" %d %d %d %d %d %08x %08x \n",FEATUR_SIZE,pos, yevent, xevent,posFeaImage,blob, ts);
                                    //}
                                    unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
                                    //unsigned long evPU;
                                    //evPU = 0;
                                    //evPU = evPU | polevent;
                                    //evPU = evPU | (xevent << 1);
                                    //evPU = evPU | (yevent << 8);
                                    //evPU = evPU | (cameraevent << 15);
                                    
                                    //blob = blob & 0x0000FFFF;
                                    //blob = 0x00001021;
                                    //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, blob, ts);
                                    
                                    
                                    // representing the feature map for debug
                                    unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();
                                    
                                    
                                    
                                    if(polevent > 0) {
                                        if(pFeaRight[posFeaImage] <= 255 - devianceFea) {
                                            pFeaRight[posFeaImage] += devianceFea ;
                                        }
                                        else {
                                            pFeaRight[posFeaImage] = 255;
                                        }
                                    }
                                    else {
                                        if(pFeaRight[posFeaImage] >= devianceFea) {
                                            pFeaRight[posFeaImage] -= devianceFea ;
                                        }
                                        else {
                                            pFeaRight[posFeaImage] = 0;
                                        }
                                    }
                                    
                                    if(pFeaRight[posFeaImage] > 240) {
                                        
                                        TimeStamp* timestamp = new TimeStamp();
                                        timestamp->setStamp(ts);
                                        //txQueue.push_back(timestamp);
                                        packets->append(timestamp->encode());
                                        delete timestamp;
                                        countEventToSend++;
                                        
                                        AddressEvent* ae = new AddressEvent(); 
                                        ae->setChannel(0);
                                        ae->setPolarity(1);
                                        ae->setX(xevent);
                                        ae->setY(yevent);
                                        packets->append(ae->encode());
                                        //txQueue.push_back(ae);
                                        delete ae;
                                        countEventToSend++;
                                        
                                        //  bufferFEA_copy->address   = (u32) blob;
                                        //  bufferFEA_copy->timestamp = (u32) ts;
                                        
                                        
                                        
                                        //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                                        //bufferFEA_copy++; // jumping to the next event(u32,u32)
                                        // countEventToSend++;
                                    }
                                    else if(pFeaRight[posFeaImage] < 10) {
                                        
                                        TimeStamp* timestamp = new TimeStamp();
                                        timestamp->setStamp(ts);
                                        //txQueue.push_back(timestamp);
                                        packets->append(timestamp->encode());
                                        delete timestamp;
                                        countEventToSend++;
                                        
                                        AddressEvent* ae = new AddressEvent();
                                        ae->setChannel(0);
                                        ae->setPolarity(0);
                                        ae->setX(xevent);
                                        ae->setY(yevent);
                                        packets->append(ae->encode());
                                        //txQueue.push_back(ae);
                                        delete ae;
                                        countEventToSend++;
                                        
                                        
                                        // bufferFEA_copy->address   = (u32) blob;
                                        //  bufferFEA_copy->timestamp = (u32) ts;
                                        
                                        
                                        //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                                        //bufferFEA_copy++; // jumping to the next event(u32,u32)
                                        //countEventToSend++;
                                    }                           
                                    
                                    
                                    //unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();
                                    //pFeaRight[posFeaImage] = 255;
                                    
                                } //end else
                                
                            } //end for i 5
                        }
                        //                      
                        countEventToSend++;
                    }
                }
            }
            // -------------------------- TS ----------------------------------
            else if(typeRef == strcmpTS) {
                //printf("timestamp \n");
                TimeStamp* ptr=dynamic_cast<TimeStamp*>(eventInQueue);
                
                //identified an time stamp event
                ts = (unsigned int) ptr->getStamp();
                //printf("timestamp \n")
                if(VERBOSE & false) {
                    fprintf(fdebug, " %08x  \n", (unsigned int) ts);
                }
                
                countEventToSend++;

            }
            // -------------------------- NULL ----------------------------------
            else {
                printf("not recognized \n");
                return;
            } 
        }    
        //printf("\n");
        iterEvent++;
        //printf("end of the generate function \n");
        
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //deleting the event to avoid memory leak
		if (eventInQueue)
        {
			delete eventInQueue;
			eventInQueue=0;
		}
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
    } //end for i
    
	//all the elements processed, clearing the queue
	q->clear();
   


    //############################################################################
    //copying the packets to the bottle
    //printf("copying the queue %08x of %d elements to the bottle \n",packets,packets->size() );
    //printf("value %s \n", packets->toString().c_str());
    
    
    /* _old
    for (size_t i = 0; i<txQueue.size(); i++) {
        if(txQueue[i]->getType()=="AE") {
            printf("found")
            AddressEvent* ptr=dynamic_cast<AddressEvent*>(txQueue[i]);
            Bottle b = ptr->encode();
            packets.append(b);
        }
        else if (txQueue[i]->getType()=="TS") {
            TimeStamp* ptr=dynamic_cast<TimeStamp*>(txQueue[i]);
            Bottle b = ptr->encode();
            packets.append(b);
        }
    }
    */

    
    
}

void efExtractorThread::run() {   

    count++;
    countEvent = 0;

    bool flagCopy;
    if(!idle) {
        
        //ImageOf<PixelMono> &left = outLeftPort.prepare();  
        //left.resize(1228, 128);
        //left.zero();
        unsigned char* pLeft  = leftOutputImage->getRawImage(); 
        unsigned char* pRight = rightOutputImage->getRawImage();
        unsigned char* pMemL  = leftInputImage->getRawImage();
        //left.zero();
        
        // reads the buffer received and saves it into a working buffer        
        //printf("returned 0x%x 0x%x with bottleHandler %d \n", bufferCopy, flagCopy, bottleHandler);
        if(!bottleHandler) {
            cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
        }
        else {
            //printf("extracting bottle \n");
            receivedBottle->clear();
            ebHandler->extractBottle(receivedBottle);  
			if (receivedBottle == 0)
			{
				//do something
			}
        }
        
        // ---------------------------------------------------------------------------------------------------

        int num_events = CHUNKSIZE >> 3 ;
        u32* buf2 = (u32*)bufferCopy;
        //plotting out
        int countEvent = 0;
        u32* bufCopy2 = (u32*)bufferCopy2;
        firstHalf = true;
        if(buf2[0] < 0x8000F000) {
            firstHalf = true;
        }
        else {
            firstHalf = false;
        }

        // ----------------------------------------------------------------------------------------------------
        
        // extract a chunk/unmask the chunk               
        //printf("trying to unmask \n");        
        if(!bottleHandler) {
            int dim = unmask_events.unmaskData(bufferCopy2,countEvent * sizeof(u32),eventFeaBuffer);        
        }
        else {
            
            if(receivedBottle->size() != 0) {
                //printf("deleting the rxQueue %08X \n",rxQueue );
                //delete rxQueue;   // freeing memory for the new queue of events
                //rxQueue = new eEventQueue(); // preparing the new queue                
                //printf("created the new rxQueue %08X \n",rxQueue );
                //printf("unmasking received bottle and creating the queue of event to send \n");
                unmask_events.unmaskData(receivedBottle, rxQueue); // saving the queue
            }  
        }
        
        
        //printf("event converted %d  \n",countEvent);          
        //printf("         countLeft %d \n" , countEventLeft);
        //printf("         countRight %d \n", countEventRight);

        // --------------------------------------------------------------------------
        
        //generating the memory
        int countEventToSend = 0;
        if(!bottleHandler) {
            //printf("Generating memory in not-bottleHandler \n");
            generateMemory(countEvent, countEventToSend);
        }
        else {
            //eEventQueue tx(false);
            //printf("Generating memory in bottleHandler %d \n",rxQueue->size() );
            //tx = *txQueue;
            if((rxQueue != NULL) && (rxQueue->size() != 0)) {
                //printf("Counted a total of %d %d \n", countEventToSend, rxQueue->size());
                bottleToSend->clear();
                //printf("generating memory of the events \n");
                generateMemory(rxQueue, bottleToSend, countEventToSend);
                //printf("Counted a total of %d %d \n \n", countEventToSend, rxQueue->size());
            }
        }
        
        // -----------------------------------------------------------------------  
        // leaking section of the algorithm (retina space) 
        //printf("leaking section of the algorithm (retina space) \n");
        pMemL       = leftInputImage->getRawImage();
        pLeft       = leftOutputImage->getRawImage();
        pRight      = rightOutputImage->getRawImage();
        int padding = leftOutputImage->getPadding();
        
        int CONST_DECREMENT_DOUBLE = 1 ;        
        
        for(int row  = 0; row < RETINA_SIZE; row++) {
            for (int col = 0; col< RETINA_SIZE; col++) {
                if(*pLeft >= 127 + CONST_DECREMENT) { 
                    *pLeft-= CONST_DECREMENT;                    
                }
                else if(*pLeft<= 127 - CONST_DECREMENT) {                    
                    *pLeft += CONST_DECREMENT;
                }
                else{
                    *pLeft = 127;
                }
                
                if(*pRight >= 127 + CONST_DECREMENT) {                
                    *pRight -= CONST_DECREMENT;
                }
                else if(*pRight <= 127 - CONST_DECREMENT) {
                    *pRight += CONST_DECREMENT;
                }
                else{
                    *pRight = 127;
                }
                
                pLeft++;
                pRight++;
            }
            //pMemL += padding;                
            pLeft += padding;
            pRight+= padding;
        }
        
        
        //---------------------------------------------------------------------------        
        //printf("leaking section of the algorithm feature space \n");
        // leaking section of the algorithm ( feature map)
        unsigned char* pFeaLeft  = leftFeaOutputImage->getRawImage();
        unsigned char* pFeaRight = rightFeaOutputImage->getRawImage();
        padding  = leftFeaOutputImage->getPadding();
        for(int row =0; row < FEATUR_SIZE; row++) {
            for (int col = 0; col< FEATUR_SIZE; col++) {
                if(*pFeaLeft >= 127 + CONST_DECREMENT) {                
                    *pFeaLeft -= CONST_DECREMENT;
                }
                else if(*pFeaLeft <= 127 - CONST_DECREMENT) {
                    *pFeaLeft += CONST_DECREMENT;
                }
                else{
                    *pFeaLeft = 127;
                }
                
                if(*pFeaRight >= 127 + CONST_DECREMENT) {                
                    *pFeaRight -= CONST_DECREMENT;
                }
                else if(*pFeaRight <= 127 - CONST_DECREMENT) {
                    *pFeaRight += CONST_DECREMENT;
                }
                else{
                    *pFeaRight = 127;
                }                
                
                pFeaLeft++;
                pFeaRight++;
                
            }
                            
            pFeaLeft  += padding;
            pFeaRight += padding;
        } 
        
        
       
        //printf("after leaking section of the algorithm \n");
        // ---------------------------------------------------------------------------
        
        
        /******** deprecated *************
        //building feature events
        char* buffer = (char*) bufferFEA;
        aer* bFea_copy = bufferFEA;
        //int sz = countEvent * sizeof(aer);
        int sz = countEventToSend * sizeof(aer);
        // sending events 
        if ((outEventPort.getOutputCount()) && (countEventToSend > 0)) {     
            if (VERBOSE) {
                for (int i = 0; i < countEventToSend; i++) {
                    u32 blob      = bFea_copy[i].address;
                    u32 timestamp = bFea_copy[i].timestamp;
                    fprintf(fout,">%08x %08x \n",timestamp, blob);
                }
                fprintf(fout, ">>>>>>>>>>>>>>>>>>>>>>>>>>> \n");
            }
            //printf("Sending \n");
            eventBuffer data2send(buffer,sz);    
            eventBuffer& tmp = outEventPort.prepare();
            tmp = data2send;
            outEventPort.write();
        } 
        */
        
        
        //************************ OUTPUT OF THE MODULE ****************************
        // writing the mapped events
        
        if(outBottlePort.getOutputCount()) {
            //Bottle packets;          
            //cout<<"encoding events within packets "<<bottleToSend->size() <<endl;
            if(bottleToSend->size() > 0) {
                //for (size_t i=0; i<txQueue->size(); i++) {
                //    printf("encoding the %d event \n", i);
                //    cout<<((*txQueue)[i])->getContent().toString()<<endl;
                //    packets.append(((*txQueue)[i])->encode());
                //}
                //printf("after encoding events in the packets %d \n", bottleToSend->size() );
                //Bottle b;
                //b.copy(*bottleToSend);

                if(VERBOSE) {
                    for(int i = 0; i< bottleToSend->size(); i++) {
                        fprintf(fdebug,"# %08x \n", bottleToSend->get(i).asInt());
                    }
                }
                
                eventBottle data2send(bottleToSend);         
                eventBottle& tmp = outBottlePort.prepare();
                tmp = data2send;
                outBottlePort.write(); 
                //printf("writing the port \n");
            }
        }
        
        
        
        // writing images out on ports
        //if(outLeftPort.getOutputCount()) {
        //     outLeftPort.prepare()     = *leftOutputImage;
        //     outLeftPort.write();
        //}
        
        
        // if(outRightPort.getOutputCount()) {
        //     outRightPort.prepare()    = *rightOutputImage;
        //     outRightPort.write();
        // }
        
        //printf("sending images \n");
        if(outFeaLeftPort.getOutputCount() && !(count % 10)) {
            outFeaLeftPort.prepare()  = *leftFeaOutputImage;
            outFeaLeftPort.write();
        }
        if(outFeaRightPort.getOutputCount() && !(count % 10)) {
            outFeaRightPort.prepare() = *rightFeaOutputImage;
            outFeaRightPort.write();
        } 
        

        //printf("after the images are sent to the ports \n");
    } //end of idle
}




