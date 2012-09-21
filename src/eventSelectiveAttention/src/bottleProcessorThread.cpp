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
 * @file bottleProcessorThread.cpp
 * @brief Implementation of the thread (see header bottleProcessorThread.h)
 */

#include <iCub/bottleProcessorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <time.h>

using namespace emorph::ecodec;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define INTERVFACTOR 1
#define COUNTERRATIO 1 //1.25       //1.25 is the ratio 0.160/0.128
#define MAXVALUE     0xFFFFFF //4294967295
#define THRATE       5
#define CHUNKSIZE    32768 //65536 //8192
#define dim_window   10
#define synch_time   1

//#define VERBOSE
#define TEMPDIST      1000000
#define INCR_RESPONSE 0.1
#define DECR_RESPONSE 0.9

bottleProcessorThread::bottleProcessorThread() : RateThread(THRATE) {
    responseGradient = 127;
    retinalSize      = 128;  //default value before setting 
    saliencySize     = 128;  //default dimension of the saliency map
    lasttimestamp    = 0;
    maxLeft          = -1.0;
    minLeft          =  1.0;
    maxRight         = -1.0;
    minRight         =  1.0;
    featureMapLeft   = 0;
    timestampMapLeft = 0;
    count            = 0;
    minCount         = 0; //initialisation of the timestamp limits of the first frame

    bottleHandler   = true;
    synchronised    = false;
    greaterHalf     = false;
    firstRun        = true;
    idle            = false;
    timestampUpdate = false;
    verb            = false;

    bufferCopy = (char*) malloc(CHUNKSIZE);
    countStop = 0;
    
    string i_fileName("bottleProcessorThread.eventBottle.txt");
    raw = fopen(i_fileName.c_str(), "wb");
    lc            = 0;	
    rc            = 0;
    minCount      = 0;
    minCountRight = 0;
}

bottleProcessorThread::~bottleProcessorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool bottleProcessorThread::threadInit() {
    printf(" \nstarting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    fout = fopen("./eventSelectiveAttention.dump.txt","w+");

    resize(retinalSize, retinalSize);

    printf("starting the first collector!!!.... \n");    
    cfConverter = new eventCartesianCollector();
    cfConverter->useCallback();
    cfConverter->setRetinalSize(retinalSize);
    cfConverter->open(getName("/retina:i").c_str());
    
    printf("\n opening retina\n");
    printf("starting the plotter \n");
    
    //pThread = new plotterThread();
    //pThread->setName(getName("").c_str());
    //pThread->setStereo(stereo);
    //pThread->setRetinalSize(retinalSize);
    //pThread->start();

    ebHandler = new eventBottleHandler();
    ebHandler->useCallback();
    ebHandler->setRetinalSize(128);
    ebHandler->open(getName("/retinaBottle:i").c_str());

    //map1Handler = new eventBottleHandler();
    //map1Handler->useCallback();
    //map1Handler->setRetinalSize(32);
    //map1Handler->open(getName("/map1Bottle:i").c_str());

    receivedBottle = new Bottle();
    unmask_events = new unmask(32);
    //unmask_events->setRetinalSize(32);
    
    //unmask_events->setResponseGradient(responseGradient);
    //unmask_events->setASVMode(asvFlag);
    //unmask_events->setDVSMode(dvsFlag);
    //unmask_events->start();

    //minCount = cfConverter->getEldestTimeStamp();
    startTimer = Time::now();
    //clock(); //startTime ;
    //T1 = times(&start_time);
    //microseconds = 0;
    //microsecondsPrev = 0;
    gettimeofday(&tvend, NULL);

    count = 0;
    microsecondsPrev = 0;
    minCount = 0;
    minCountRight= 0;
    
    //saliencyMapLeft  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    //memset(saliencyMapLeft,0, retinalSize * retinalSize * sizeof(double));
    //saliencyMapRight = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    //memset(saliencyMapRight,0, retinalSize * retinalSize * sizeof(double));
    
    
    printf("allocating memory for maps %d %d \n", saliencySize, retinalSize);
    featureMapLeft  = (double*) malloc(saliencySize * saliencySize * sizeof(double));
    featureMapRight = (double*) malloc(saliencySize * saliencySize * sizeof(double));
    memset(featureMapLeft ,0, saliencySize * saliencySize * sizeof(double));
    memset(featureMapRight,0, saliencySize * saliencySize * sizeof(double));
    for (int k = 0; k < saliencySize * saliencySize; k++) {
        featureMapLeft[k] =0;
        featureMapRight[k] = 0;
    }
    
    //timestampMap = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    //memset(timestampMap,0, retinalSize * retinalSize * sizeof(unsigned long));
    timestampMapLeft  = (unsigned long*) malloc(saliencySize * saliencySize * sizeof(unsigned long) );
    timestampMapRight = (unsigned long*) malloc(saliencySize * saliencySize * sizeof(unsigned long) );
    memset(timestampMapLeft ,0, saliencySize * saliencySize * sizeof(unsigned long));
    memset(timestampMapRight,0, saliencySize * saliencySize * sizeof(unsigned long));

    unmaskedEvents = (AER_struct*) malloc (CHUNKSIZE * sizeof(int));
    memset(unmaskedEvents, 0, CHUNKSIZE * sizeof(int));

    rxQueue = new eEventQueue();

    printf("Initialisation in selector thread correctly ended \n");
    return true;
}

void bottleProcessorThread::interrupt() {
    outPort.interrupt();
    outPortRight.interrupt();
}

void bottleProcessorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string bottleProcessorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void bottleProcessorThread::resize(int widthp, int heightp) {
    imageLeft = new ImageOf<PixelMono>;
    imageLeft->resize(retinalSize,retinalSize);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(retinalSize,retinalSize);
    //imageRight->resize(widthp,heightp);
}



void bottleProcessorThread::forgettingMemory() {
    
    double* pLeft  = featureMapLeft;
    double* pRight = featureMapRight;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {            
            if(*pLeft > DECR_RESPONSE) {
                *pLeft -= DECR_RESPONSE;               
                //*pLeft = 0;
            }
            else if (*pLeft < -DECR_RESPONSE){
                *pLeft += DECR_RESPONSE;
                //*pLeft = 0;
            }
            pLeft++;
            if(*pRight > 0) {
                
                *pRight -= DECR_RESPONSE;
            }
            else {
                *pRight += DECR_RESPONSE;
            }
            pRight++;
        }
    }
    
}


void bottleProcessorThread::spatialSelection(eEventQueue *q) {
    int dequeSize = q->size();
    //printf("bottleProcessorThread::spatialSelection: %d events inQueue \n", dequeSize);
    unsigned long ts;
    int countLeftPos  = 0 , countLeftNeg  = 0;
    int countRightPos = 0 , countRightNeg = 0;
    //printf("saliencySize %d retinalSize %d \n", saliencySize, retinalSize);
    int scaleFactor   = saliencySize / retinalSize; 
    //printf("scaleFactor %d \n", scaleFactor);

    mutexMinMaxLeft.wait();
    maxLeft = 0.0;
    minLeft = 1.0;
    mutexMinMaxLeft.post();  
  

    for (int evt = 0; evt < dequeSize; evt++) {
        //printf("analyzing event %d \n", evt);
        eEvent* eventInQueue = (*q) [evt];
        if(eventInQueue != 0) {                    
            //********** extracting the event information **********************
            // to identify the type of the packet
            // user can rely on the getType() method
            // -------------------------- AE  ----------------------------------
            if (eventInQueue->getType()=="AE") {
                // identified an  address event
                AddressEvent* ptr=dynamic_cast<AddressEvent*>(eventInQueue);
                if(ptr->isValid()) { 
                    //ts  = iterEvent->ts;
                    //pol = iterEvent->pol;
                    //cam = iterEvent->cam;
                    int cartX     = ptr->getX();
                    int cartY     = ptr->getY();
                    int camera    = ptr->getChannel();
                    int polarity  = ptr->getPolarity();

                    if(scaleFactor == 4){
                        if(cartX > 31) {
                            //printf("ERROR in UNMASKING cartX %d \n", cartX);
                            //cartX = 31;
                        }
                        if(cartY > 31){
                            //printf("ERROR in UNMASKING cartY %d \n", cartY);
                            //cartY = 31;
                        }
                    }
                    

                    // TODO: check the difference between representation of feature map and original flow
                    int xpos,ypos;
                    if(scaleFactor == 4) {
                        xpos      = cartX * scaleFactor;
                        ypos      = (cartY - 1) * scaleFactor;
                        
                        /*
                        if(camera == 0)
                            camera = 1;
                        else
                            camera = 0;
                        */
                        
                    }
                    else {
                        xpos      = cartY;
                        ypos      = saliencySize - cartX;
                        
                    }
                    
                    //printf("cartX %d cartY %d xpos %d ypos %d \n", cartX, cartY, xpos, ypos);


#ifdef VERBOSE
                    //printf(" address event %s \n",ptr->getContent().toString().c_str());
                    //printf("scale factor %d \n", scaleFactor);
                    fprintf(fout,"ae : %08X \n",ptr->encode().get(0).asInt());
                    //fprintf(fout,"%s \n",ptr->getContent().toString().c_str());
                    fprintf(fout,"cartx %d  carty %d  camera %d \n", cartX, cartY, camera);
#endif

                    //if(camera == 0) {                        
                    if(camera == 0) { 
                       //printf("saliencyMapLeft in %d %d changed (pol:%d)  saliencySize %d scaleFactor %d \n", cartX, cartY, polarity, saliencySize, scaleFactor);
                        if(polarity == 0) {
                           
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    //saliencyMapLeft [(xpos * scaleFactor + xi) * 3   + (ypos + yi) * saliencySize * 3] +=  INCR_RESPONSE;
                                    
                                    mutexFeaLeft.wait();
                                    double* pFea = &featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)];
                                    //*pFea = 0;
                                    *pFea += INCR_RESPONSE;
                                    //featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] += INCR_RESPONSE;
                                    mutexMinMaxLeft.wait();
                                    if (abs(*pFea) > maxLeft) {
                                        maxLeft = abs(*pFea);                                        
                                    }
                                    if (abs(*pFea) < minLeft) {
                                        minLeft = abs(*pFea);
                                    }
                                    mutexMinMaxLeft.post();
                                    mutexFeaLeft.post();
                                    countLeftPos++;
                                    
                                }
                            }
                        }
                        else {
                            //printf("saliencyMapRight in %d %d changed (pol:%d) saliencySize %d scaleFactor %d \n", cartX, cartY, polarity, saliencySize, scaleFactor);
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    //saliencyMapLeft [(xpos * scaleFactor + xi) * 3   + (ypos + yi) * saliencySize * 3 ] -=  INCR_RESPONSE;
                                    
                                    mutexFeaLeft.wait();
                                    double* pFea = &featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)];
                                    //*pFea = 0;
                                    *pFea -= INCR_RESPONSE;
                                    //featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] -= INCR_RESPONSE;
                                    mutexMinMaxLeft.wait();
                                    if (abs(*pFea) > maxLeft) {                                        
                                        maxLeft = abs(*pFea);
                                    }
                                    if (abs(*pFea) < minLeft) {
                                        minLeft = abs(*pFea);
                                    }
                                    mutexMinMaxLeft.post();
                                    mutexFeaLeft.post();
                                    countLeftNeg++;
                                    
                                }
                            }
                        }
                        //----------------------------------------------------------
                        for ( int xi = 0; xi < scaleFactor; xi++) {
                            for (int yi = 0; yi < scaleFactor; yi++) {
                                
                                mutexTimeLeft.wait();
                                //printf("updating ts left %08x  \n", ts);
                                timestampMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] = ts;
                                mutexTimeLeft.post();
                                
                            }
                        }
                        
                    }
                    else {
                        //printf("saliencyMapRight in %d %d changed (pol:%d) \n", cartX, cartY, polarity);                       
                        if(polarity == 0) {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    mutexFeaRight.wait();
                                    double* pFea = &featureMapRight[(ypos + yi) * saliencySize + (xpos + xi)];
                                    *pFea = 0; //INCR_RESPONSE;
                                    //featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] += INCR_RESPONSE;
                                    mutexMinMaxRight.wait();
                                    if (abs(*pFea) > maxRight) {
                                        maxRight = abs(*pFea);
                                    }
                                    if (abs(*pFea) < minRight) {
                                        minRight = abs(*pFea);
                                    }
                                    mutexMinMaxRight.post();
                                    mutexFeaRight.post();
                                    countRightPos++;
                                }
                            }
                        }
                        else {
                            for ( int xi = 0; xi < scaleFactor; xi++) {
                                for (int yi = 0; yi < scaleFactor; yi++) {
                                    mutexFeaRight.wait();
                                    double* pFea = &featureMapRight[(ypos + yi) * saliencySize + (xpos + xi)];
                                    *pFea = 0; //INCR_RESPONSE;
                                    //featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] -= INCR_RESPONSE;
                                    mutexMinMaxRight.wait();
                                    if (abs(*pFea) > maxRight) {                                     
                                        maxRight = abs(*pFea);                                       
                                    }
                                    if (abs(*pFea) < minRight) {              
                                        minRight = abs(*pFea);                                        
                                    }
                                    mutexMinMaxRight.post();
                                    mutexFeaRight.post();
                                    countRightNeg++;
                                }
                            }
                        }
                        //----------
                        for ( int xi = 0; xi < scaleFactor; xi++) {
                            for (int yi = 0; yi < scaleFactor; yi++) {
                                mutexTimeRight.wait();
                                //printf("updating ts right \n");
                                timestampMapRight[(ypos + yi) * saliencySize + (xpos + xi)] = ts;
                                mutexTimeRight.post();
                            }
                        }
                    }                                     
                } //end if (ptr->isValid()) 
            } //end if ((*q)[evt]->getType()=="AE")
            // -------------------------- TS ----------------------------------
            else if(eventInQueue->getType()=="TS") {
                
                TimeStamp* ptr=dynamic_cast<TimeStamp*>(eventInQueue);

#ifdef VERBOSE
                fprintf(fout,"ts : %08X  \n",ptr->encode().get(0).asInt());
#endif
                
                //identified an time stamp event
                ts = (unsigned int) ptr->getStamp();
                //printf("timestamp %08X %08X \n", ts, lasttimestamp);
                if(timestampUpdate) {
                    mutexLastTimestamp.wait();
                    //printf("timestamp %08X %08X %d \n", ts, *lasttimestamp, scaleFactor);
                    *lasttimestamp = ts;
                    mutexLastTimestamp.post();
                }
                    
                forgettingMemory();

            }
            // -------------------------- NULL ----------------------------------
            else {
                printf("not recognized \n");
                return;
            } 
        } //end if((*q)[evt] != 0)
        
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //deleting the event to avoid memory leak
        if(eventInQueue) {
            delete eventInQueue;
            eventInQueue = 0;
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
    } //end for
    
    //all the elements processed, clearing the queue
	q->clear();


#ifdef VERBOSE
                fprintf(fout,"-------------------------------- \n");
#endif
                //printf("number for left %d %d \n", countLeftPos, countLeftNeg);
                //printf("number for right %d %d \n", countRightPos, countRightNeg);
}

void bottleProcessorThread::spatialSelection(AER_struct* buffer,int numberOfEvents, double w, unsigned long minCount, unsigned long maxCount) {
    // in the list of unmasked event updates the saliency map and the timestamp map 
    AER_struct* iter = buffer; // generated the iterator of the buffer
    int inputRetinaSize = 32;
    
    for (int i = 0; i < numberOfEvents; i++) {        
        if((iter->ts > minCount) && (iter->ts < maxCount)) {
            if((iter->x != 128) && (iter->y != 1)){
                //printf("%02d %02d  ", iter->x, iter->y);    
                
                int pos = (inputRetinaSize - 1 - iter->x) + (inputRetinaSize - 1 - iter->y) * inputRetinaSize;
                if(pos >= 32*32) {
                    printf("Error %d %d \n", iter->x, iter->y);
                }
                if(iter->pol > 0) {
                    featureMapLeft[pos]   = featureMapLeft[pos] + 127;
                    if(featureMapLeft[pos] > 127) {
                        featureMapLeft[pos] = 127;
                    }
                }
                else {
                    featureMapLeft[pos]   = featureMapLeft[pos] - 127;
                    if(featureMapLeft[pos] < -127) {
                        featureMapLeft[pos] = -127;
                    }
                }
                
                //printf (" %08X \n", iter->ts);
                timestampMap[pos] = iter->ts;            
            } 
        }
        iter++;
    }    
}

void bottleProcessorThread::run() {
  count++;
  bottleHandler = true;
  if(!idle) {
    interTimer = Time::now();
    double interval2 = (interTimer - startTimer) * 1000000;
    // reads the buffer received
    //bufferRead = cfConverter->getBuffer();    
    // saves it into a working buffer
    //printf("checking the bottleHandler \n");
    //printf("============================================================= \n");

    
    if(!bottleHandler) {
        cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
    }
    else {
        //printf("bottleProcessorThread::run : extracting Bottle! \n");
        
        receivedBottle->clear();
        ebHandler->extractBottle(receivedBottle); 
    }

 
    // =============================== Unmasking ====================================
    // extract a chunk/unmask the chunk
    // printf("verb %d \n",verb);
    if(!bottleHandler) {
        //printf("unmasking without bottleHandler \n");
        unmask_events->unmaskData(bufferCopy,CHUNKSIZE, unmaskedEvents);
    }
    else {
        //printf("unmasking with bottleHandler \n");
        if(receivedBottle->size() != 0) {     
            //printf("unmasking with bottleHandler : size()== %d \n",receivedBottle->size() );
            // rxQueue->clear(); <----- no need for clear here. Look in the ....
            //printf("unmasking received bottle and creating the queue of event to send \n");            
            unmask_events->unmaskData(receivedBottle, rxQueue); // saving the queue
            //printf("bottle of events to send \n");
        }
    }
    //printf("end of the unmasking \n");g
    
    
    // =======================  spatial processing===========================
    double w1 = 0, w2 = 0, w3 = 0, w4 = 0;
    //spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1);    
    
    // spatial processing
    if(!bottleHandler) {
        //printf("spatial selection no bottle Handler \n");
        spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1, minCount, maxCount);
    }
    else {        
        if((rxQueue != 0)&&(receivedBottle->size() != 0)) {
            //printf("spatial selection  bottle Handler\n");
            spatialSelection(rxQueue);
            //printf("after spatial selection bottleHandler \n");
        }
   }
  }
}

void bottleProcessorThread::copyFeatureMapLeft(double *pointer) {
    mutexFeaLeft.wait();
    if(0 == featureMapLeft) {
         mutexFeaLeft.post();
        return;
    }
    double* pFea = featureMapLeft;
    for (int i = 0; i < saliencySize * saliencySize; i++) {
        
        //*pFea = 0;
        *pointer = *pFea;
        pointer++;  pFea++;
    }    
    mutexFeaLeft.post();
}

void bottleProcessorThread::copyTimestampMapLeft(unsigned long *pointer) {
    mutexTimeLeft.wait();
    if(0 == timestampMapLeft) {
        mutexTimeLeft.post();
        return;
    }
    unsigned long* pTime = timestampMapLeft;
    double *       pFea  = featureMapLeft;

    for (int i = 0; i < saliencySize * saliencySize; i++) { 
        // control for old events to avoid events after one cycle timestamp
        long int distance;
        if(lasttimestamp!=0) {
            distance = std::abs((long int) (*pTime - *lasttimestamp));
        }
        //printf("testing the distance from the last time stamp %d \n", distance);
        if(distance > TEMPDIST) {
            *pointer = *pTime;
            *pTime   = 0;
            *pFea    = 0;
        }
        else {
            *pointer = *pTime;   
        }
        pointer++; 
        pTime++;
        pFea++;
    }    
    mutexTimeLeft.post();
}

void bottleProcessorThread::threadRelease() {
    printf("bottleProcessorThread::threadRelease()  \n");
    idle = false;
    //fclose(fout);
    printf("bottleProcessorThread release:freeing bufferCopy \n");
    free(featureMapLeft);
    free(featureMapRight);
    printf("bottleProcessorThread release:freed featuremaps \n");
    free(timestampMapLeft); 
    free(timestampMapRight);    
    printf("bottleProcessorThread release:freed timestampMap \n");
    free(unmaskedEvents); 

    printf("bottleProcessorThread release:closing ports \n");
    outPort.close();
    outPortRight.close();
    
    printf("deleting memory \n");
    delete receivedBottle;
    ebHandler->close();
    
    printf("bottleProcessorThread release :        deleting converter \n");
    delete cfConverter;
    printf("bottleProcessorThread release : correctly freed memory from the cfCollector \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

