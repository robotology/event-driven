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
 * @file eventSelectorThread.cpp
 * @brief Implementation of the thread (see header eventSelectorThread.h)
 */

#include <iCub/eventSelectorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define INTERVFACTOR 6.24
#define COUNTERRATIO 1 //1.25       //1.25 is the ratio 0.160/0.128
#define MAXVALUE 4294967295
#define THRATE 5
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
//#define retinalSize 128
#define CHUNKSIZE 32768 //65536 //8192
#define dim_window 1
#define synch_time 1

//#define VERBOSE

eventSelectorThread::eventSelectorThread() : RateThread(THRATE) {
    responseGradient = 127;
    retinalSize  = 128;  //default value before setting 
  
    synchronised = false;
    greaterHalf  = false;
    firstRun     = true;
    count        = 0;
    minCount     = 0; //initialisation of the timestamp limits of the first frame
    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
    countStop = 0;
    verb = false;
    string i_fileName("events.log");
    raw = fopen(i_fileName.c_str(), "wb");
    lc = rc = 0;	
    minCount      = 0;
    minCountRight = 0;
}

eventSelectorThread::~eventSelectorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool eventSelectorThread::threadInit() {
    printf(" \nstarting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    fout = fopen("./dump.txt","w+");

    resize(retinalSize, retinalSize);
    printf("starting the converter!!!.... \n");
    
    cfConverter = new eventCartesianCollector();
    cfConverter->useCallback();
    cfConverter->setRetinalSize(retinalSize);
    cfConverter->open(getName("/retina:i").c_str());
    
    printf("\n opening retina\n");
    printf("starting the plotter \n");
    
    pThread = new plotterThread();
    pThread->setName(getName("").c_str());
    pThread->setStereo(stereo);
    pThread->setRetinalSize(retinalSize);
    pThread->start();
    
    unmask_events = new unmask();
    //unmask_events->setRetinalSize(retinalSize);
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

    printf("Initialisation in collector thread correctly ended \n");
    return true;
}

void eventSelectorThread::interrupt() {
    outPort.interrupt();
    outPortRight.interrupt();
}

void eventSelectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string eventSelectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void eventSelectorThread::resize(int widthp, int heightp) {
    imageLeft = new ImageOf<PixelMono>;
    imageLeft->resize(widthp,heightp);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(widthp,heightp);
}


void eventSelectorThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera){
    assert(image!=0);
    //printf("retinalSize in getMonoImage %d \n", retinalSize);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    
    /*
    unsigned long int lasttimestamp = getLastTimeStamp();
    if (lasttimestamp == previousTimeStamp) {   //condition where there were not event between this call and the previous
        for(int r = 0 ; r < retinalSize ; r++){
            for(int c = 0 ; c < retinalSize ; c++) {
                *pImage++ = (unsigned char) 127;
            }
            pImage+=imagePadding;
        }
        return;
    }
    previousTimeStamp = lasttimestamp;
    */

    // determining whether the camera is left or right
    int* pBuffer = unmask_events->getEventBuffer(camera);
    unsigned long* pTime   = unmask_events->getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            
            unsigned long timestampactual = *pTime;
            //bool tristateView = false;
            
            if(tristate) {
                // decreasing the spatial response without removing it
                if(count % 10 == 0) {
                    if(*pBuffer > 20){
                        *pBuffer = *pBuffer - 1;                                       
                    }
                    else if(*pBuffer < -20){
                        *pBuffer = *pBuffer + 1;                                       
                    }
                }
                

                //if(minCount>0 && maxCount > 0 && timestampactual>0)
                //printf("actualTS%ld val%ld max%ld min%ld  are\n",timestampactual,timestampactual * COUNTERRATIO,minCount,maxCount);
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
                    *pImage = (unsigned char) (127 + value);
                    //if(value>0)printf("event%d val%d buf%d\n",*pImage,value,*pBuffer);
                    pImage++;
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) (127 + value);
                    //pImage--;
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                else {
                    *pImage = (unsigned char) 127 ;
                    //printf("NOT event%d \n",*pImage);
                    pImage++;
                    
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) 127;
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage--;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                pBuffer++;
                pTime++;
            }
            // branch !tristateView
            else {
                // decreasing the spatial response without removing 
                if((*pBuffer > 20) && (count % 10 == 0)){
                    *pBuffer = *pBuffer - 1;                                       
                }
                //if(minCount>0 && maxCount > 0 && timestampactual>0)
                //printf("actualTS%ld val%ld max%ld min%ld  are\n",timestampactual,timestampactual * COUNTERRATIO,minCount,maxCount);
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
                    *pImage = (unsigned char) abs(value * 2);
                    //if(value>0)printf("event%d val%d buf%d\n",*pImage,value,*pBuffer);
                    pImage++;
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) (127 + value);
                    //pImage--;
                    //  *pImage = (unsigned char) (127 + value);
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                else {
                    *pImage = (unsigned char) 0 ;
                    //printf("NOT event%d \n",*pImage);
                    pImage++;
                    
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) 127;
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage--;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                pBuffer++;
                pTime++;
            }            
        }
        pImage+=imagePadding;
    }
    //printf("end of the function get in mono \n");
    //unmask_events->setLastTimestamp(0);
}



void eventSelectorThread::run() {
  count++;
	
  if(!idle) {
    interTimer = Time::now();
    double interval2 = (interTimer - startTimer)* 1000000;
    // reads the buffer received
    //bufferRead = cfConverter->getBuffer();    
    // saves it into a working buffer
    cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
    
    // saving the buffer into the file
    int num_events = CHUNKSIZE / 8 ;
    uint32_t* buf2 = (uint32_t*)bufferCopy;

#ifdef VERBOSE
    fprintf(fout,"##############");
    for (int evt = 0; evt < num_events; evt++) {
        unsigned long blob      = buf2[2 * evt];
        unsigned long t         = buf2[2 * evt + 1];
        fprintf(fout,"0x%08x 0x%08x \n",blob, t);
    }
#endif
    

    //getting the time
    endTimer = Time::now();
    double interval  = (endTimer - startTimer) * 1000000; //interval in us
    double procInter = interval -interval2;
    //printf("procInter %f \n", procInter);
    startTimer = Time::now();

    
    //check for wrapping of the left and right timestamp
    if(maxCount >= 4294967268  ) {
      verb = true;
      unmask_events->resetTimestampLeft();
      unmask_events->resetTimestampRight();
      printf("wrapping left %lu %lu \n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      printf("wrapping left %lu %lu\n",lc,unmask_events->getLastTimestamp());
      minCount = 0;
      maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
   
    }
    if(maxCountRight >= 4294967268) {
      verb = true;
      unmask_events->resetTimestampRight();
      unmask_events->resetTimestampLeft();
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());
      printf("wrapping right %lu %lu\n",rc,unmask_events->getLastTimestampRight());

      minCountRight = 0;
      maxCountRight = minCountRight + interval * INTERVFACTOR* (dim_window);
    }
    

    // extract a chunk/unmask the chunk
    // printf("verb %d \n",verb);
    unmask_events->unmaskData(bufferCopy,CHUNKSIZE);
    if(verb) {
      verb = false;
      countStop = 0;
    }
    
    //gettin the time between two threads
    gettimeofday(&tvend, NULL);
    //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    Tnow = ((tvend.tv_sec * 1000000 + tvend.tv_usec)
	    - (tvstart.tv_sec * 1000000 + tvstart.tv_usec));
    //printf("timeofday>%ld\n",Tnow );
    gettimeofday(&tvstart, NULL);       
    
    
    
    /*if(true){
      //printf("Saving in file \n");
      fprintf(raw,"%08X \n",lc); 
      //fwrite(&sz, sizeof(int), 1, raw);
      //fwrite(buffer, 1, sz, raw);
    }*/
    
    //synchronising the threads at the connection time
    if ((cfConverter->isValid())&&(!synchronised)) {
      printf("Sychronising ");
      unsigned long int lastleft  = unmask_events->getLastTimestamp();
      lc = lastleft  * COUNTERRATIO; 
      unsigned long int lastright = unmask_events->getLastTimestampRight();
      rc = lastright * COUNTERRATIO;

      //TODO : Check for negative values of minCount not allowed!!!!!!

      minCount = lc - interval * INTERVFACTOR* dim_window; 
      //cfConverter->getEldestTimeStamp();                                                                   
      minCountRight = rc - interval * INTERVFACTOR* dim_window;
      //printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
      startTimer = Time::now();
      synchronised = true;
      //minCount = unmask_events->getLastTimestamp();
      //printf("minCount %d \n", minCount);
      //minCountRight = unmask_events->getLastTimestamp();
      count = synch_time - 200;
    }
    else if ((count % synch_time == 0) && (minCount < 4294500000)) {
      unsigned long lastleft = unmask_events->getLastTimestamp();
      lc = lastleft * COUNTERRATIO; 
      unsigned long lastright = unmask_events->getLastTimestampRight();
      rc = lastright * COUNTERRATIO;

      //if (count == synch_time) {
      //printf("Sychronised Sychronised Sychronised Sychronised ");
      if( lc > interval* INTERVFACTOR * dim_window)
          minCount      = lc - interval* INTERVFACTOR * dim_window; //cfConverter->getEldestTimeStamp();        
      else
          minCount = 0;
      
      if( rc > interval* INTERVFACTOR * dim_window)
          minCountRight = rc - interval* INTERVFACTOR * dim_window;
      else
          minCountRight = 0;
      //maxCount = lc; 
      //maxCountRight = rc;
		 
      //printf("synchronised %1f! %llu,%llu,%llu||%llu,%llu,%llu \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
      startTimer = Time::now();
      synchronised = true; 
    }
    else {
      // this value is simply the ration between the timestamp reported by the aexGrabber (6.25Mhz) 
      //and the correct timestamp counter clock of FPGA (50 Mhz)
      microsecondsPrev = interval;
      interval = Tnow;
      minCount      = minCount + interval * INTERVFACTOR; // * (50.0 MHz FPGA Counter Clock / 6.25 Mhz ;
      minCountRight = minCount + interval * INTERVFACTOR;  // minCountRight + interval;
    } 
    //printf("minCount %d interval %f \n", minCount, interval);
    maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
    maxCountRight =  minCountRight + interval * INTERVFACTOR* (dim_window);
    
    
    //---- preventer for fixed  addresses ----//
    if(count % 100 == 0) { 
        unsigned long lastleft = unmask_events->getLastTimestamp();
        lc = lastleft * COUNTERRATIO; 
        unsigned long lastright = unmask_events->getLastTimestampRight();
        rc = lastright * COUNTERRATIO;
        //printf("countStop %d lcprev %d lc %d \n",countStop, lcprev,lc);
        if (stereo) {
            if ((lcprev == lc)||(rcprev == rc)) {
                //if (lcprev == lc) {
                countStop++;
                printf("countStop %d %lu %lu %lu %lu \n", countStop, lc, lcprev, rc, rcprev);
            }            
            else {
                countStop--;
                //printf("countStop %d \n", countStop);
                if(countStop<= 0) {
                    countStop = 0;
                }
            }
        }
        else {
            if (lcprev == lc) {
                //if (lcprev == lc) {
                countStop++;
                printf("countStop %d %lu %lu %lu %lu \n", countStop, lc, lcprev, rc, rcprev);
            }            
            else {
                countStop--;
                //printf("countStop %d \n", countStop);
                if(countStop<= 0) {
                    countStop = 0;
                }
            }
        }
        lcprev = lc;
        rcprev = rc;
    }
    
    
      
    //resetting time stamps at overflow
    if (countStop == 10) {
      //printf("resetting time stamps!!!!!!!!!!!!! %d %d   \n ", minCount, minCountRight);
      unmask_events->resetTimestampLeft(); 
      unmask_events->resetTimestampRight();
      cfConverter->reset();
      unsigned long lastleft = unmask_events->getLastTimestamp();
      lc = lastleft * COUNTERRATIO; 
      unsigned long lastright = unmask_events->getLastTimestampRight();
      rc = lastright * COUNTERRATIO;
      minCount      = 0;
      minCountRight = 0;
      maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
      maxCountRight =  minCountRight + interval * INTERVFACTOR* (dim_window);
      
      //maxCount      = 4294967268;
      //maxCountRight = 4294967268;
      countStop = 0;
      verb = true;
      printf("countStop resetting %llu %llu %llu \n",unmask_events->getLastTimestamp(), lc, rc );
      count = synch_time - 200;
      
    }
    
    // ----------- preventer end    --------------------- //
    
    getMonoImage(imageLeft,minCount,maxCount,1);
    if(imageLeft != 0) {
      pThread->copyLeft(imageLeft);
    }
    
    if(stereo) {
        getMonoImage(imageRight,minCountRight,maxCountRight,0);
        if(imageRight != 0) {
            pThread->copyRight(imageRight);
        }
    }
  }
}

void eventSelectorThread::threadRelease() {
    idle = false;
    fclose(fout);
    printf("eventSelectorThread release:freeing bufferCopy \n");
    //free(bufferCopy);
    printf("eventSelectorThread release:closing ports \n");
    outPort.close();
    outPortRight.close();
    //delete imageLeft;
    //delete imageRight;
    printf("cFCollectorThread release         stopping plotterThread \n");
    pThread->stop();
    printf("eventSelectorThread release         deleting converter \n");
    delete cfConverter;
    printf("correctly freed memory from the cfCollector \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

