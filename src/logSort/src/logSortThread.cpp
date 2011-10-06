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
 * @file logSortThread.cpp
 * @brief Implementation of the thread (see header logSortThread.h)
 */

#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

#include <iCub/logSortThread.h>
#include <iCub/config.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define COUNTERRATIO  1             // deprecated 1.25 is the ratio 0.160/0.128
#define MAXVALUE      4294967295    // max value for wrapping
#define THRATE        5             // ratethread period
#define STAMPINFRAME                // 10 ms of period times the us in 1 millisecond + time for computing
#define retinalSize   128           // deprecated
//#define CHUNKSIZE     32768//8192          //16384 //1024  // dimension of the received packet
#define dim_window    5             // deprecated 
#define synch_time    1000          // deprecated
//#define SIZE_PACKET   8192          // deprecated
#define RESET_COUNTER 200           // counter for the resetting of the EM buffer


//#define VERBOSE

logSortThread::logSortThread() : RateThread(THRATE) {
    synchronised = false;
    greaterHalf = false;
    firstRun = true;
    count=0;
    minCount = 0; //initialisation of the timestamp limits of the first frame
    idle = false;
    bufferCopy   = (char*) malloc(CHUNKSIZE);
    flagCopy     = (char*) malloc(CHUNKSIZE);
    resultCopy   = (char*) malloc(CHUNKSIZE);
    memset(bufferCopy, 0, CHUNKSIZE);
    memset(resultCopy, 0, CHUNKSIZE);
    memset(flagCopy,   0, CHUNKSIZE);
    //bufferRead = (char*) malloc(8192);
    countStop = 0;
    verb = false;    
    fout = fopen("sentEvents.txt", "w+");
}

logSortThread::~logSortThread() {
  printf("freeing memory in collector");
  delete bufferCopy;
  delete flagCopy;
  delete resultCopy;
  fclose(fout);
}

bool logSortThread::threadInit() {
  //printf(" \nstarting the threads.... \n");
  portCD.open(getName("/CD:o").c_str());
  portEM.open(getName("/EM:o").c_str());
  portIF.open(getName("/IF:o").c_str());
  
  //outPortRight.open(getName("/right:o").c_str());
  //resize(retinalSize, retinalSize);
  printf("starting the converter!!!.... \n");
  lfConverter = new logFrameConverter();
  lfConverter->useCallback();
  lfConverter->open(getName("/retina:i").c_str());
  printf("\n opening retina\n");
  printf("starting the plotter \n");
  pThread = new plotterThread();
  pThread->setName(getName("").c_str());
  pThread->start();
  
  
  //minCount = lfConverter->getEldestTimeStamp();
  startTimer = Time::now();
  //clock(); //startTime ;
  //T1 = times(&start_time);
  //microseconds = 0;
  //microsecondsPrev = 0;
  gettimeofday(&tvend, NULL);
  unmask_events.start();
  count = 0;
  microsecondsPrev = 0;
  printf("Initialisation ended \n");
  return true;
}

void logSortThread::interrupt() {
    outPort.interrupt();
}

void logSortThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string logSortThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void logSortThread::resize(int widthp, int heightp) {
    imageLeft = new ImageOf<PixelMono>;
    imageLeft->resize(widthp,heightp);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(widthp,heightp);
}


void logSortThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera){
    assert(image!=0);
    //image->resize(retinalSize,retinalSize);
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
    int* pBuffer           = unmask_events.getEventBuffer(camera);
    unsigned long* pTime   = unmask_events.getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            unsigned long timestampactual = *pTime;
            if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
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

int logSortThread::selectUnreadBuffer(char* bufferCopy, char* flagCopy, char* resultCopy){    
    //char* bufferCopy_copy = bufferCopy;
    //char* flagCopy_copy   = flagCopy;
    //char* resultCopy_copy = resultCopy;
    int sum = 0; // counter of the number of unread
    //printf("in selectUnreadBuffer 0x%x 0x%x \n",bufferCopy, &bufferCopy);

#ifdef VERBOSE
    fprintf(fout,"bbbbbbbbbbbbbbbbbbbbbbbb \n");
    int num_events = CHUNKSIZE >> 3 ;
    uint32_t* buf2 = (uint32_t*)bufferCopy;
    uint32_t* bufflag = (uint32_t*) flagCopy;

    //plotting out
    for (int evt = 0; evt < num_events; evt++) {
        unsigned long blob      = buf2[2 * evt];
        unsigned long t         = buf2[2 * evt + 1];
        unsigned long flag1     = bufflag[2 * evt];
        unsigned long flag2     = bufflag[2 * evt + 1];
        if(blob!=0) {
            fprintf(fout,"%08X %08X \n",blob,t);
            fprintf(fout,"%08X %08X \n",flag1,flag2);
        }
    }
#endif    

    //printf("in selectUnreadBuffer2 0x%x 0x%x \n",bufferCopy, &bufferCopy);

    //searching for flag == 1
    //flagCopy_copy = flagCopy;
    for(int i = 0; i < CHUNKSIZE; i++) {
        int value = *flagCopy;
        if(*flagCopy==1) {
            sum++;            
            //printf("char:%d %d ----------->", *flagCopy,*bufferCopy );               
            *resultCopy = *bufferCopy;            
            resultCopy++;
            //*flagCopy = 0;  // it is not necessary because already reset in the copychunk
            //printf("flagCopy %d \n", *flagCopy);
        }
        bufferCopy++;
        flagCopy++;
    }

    resultCopy -= sum;
    flagCopy   -= CHUNKSIZE;
    bufferCopy -= CHUNKSIZE;
    
    
#ifdef VERBOSE
        fprintf(fout, "rrrrrrrrrrrrrrrrrrrrrrrr \n" );
    num_events = CHUNKSIZE >> 3 ;
    buf2 = (uint32_t*)resultCopy;
    bufflag = (uint32_t*) flagCopy;
    //plotting out
    for (int evt = 0; evt < num_events; evt++) {
        unsigned long blob      = buf2[2 * evt];
        unsigned long t         = buf2[2 * evt + 1];
        unsigned long flag1     = bufflag[2 * evt];
        unsigned long flag2     = bufflag[2 * evt + 1];
        if(blob!=0){
            fprintf(fout,"%08X %08X \n",blob,t); 
            fprintf(fout,"%08X %08X \n",flag1,flag2);
        }
    }
    fprintf(fout, "-------------------------------- \n" );
#endif

    
    //resetting flags
    //memset(flagCopy, 0, CHUNKSIZE);


    return sum;
}

void logSortThread::run() {
    count++;
    //printf("initialised the timer \n");
    double tinit = Time::now();
    double tend;
    double difftime1, difftime2;
    if(!idle) { 
        // reads the buffer received
        // saves it into a working buffer
        //printf("returned 0x%x 0x%x \n", bufferCopy, flagCopy);
        lfConverter->copyChunk(bufferCopy, flagCopy);
        //printf("after copy chunk 0x%x 0x%x \n", bufferCopy, flagCopy);
        int unreadDim = selectUnreadBuffer(bufferCopy, flagCopy, resultCopy);

        if(unreadDim!=0) {
            //printf("Unmasking events:  %d \n", unreadDim);
           // extract a chunk/unmask the chunk       
           unmask_events.logUnmaskData(resultCopy,unreadDim,verb);
        }

        // ---  time measure --- //
        tend = Time::now();
        difftime1 = tend - tinit;
        // --------------------- //

        //if(verb) {
        //    verb = false;
        //    countStop = 0;
        //}
        //printf("countBuffer %d \n", lfConverter->getCountBuffer());


        /*
        //gettin the time between two threads
        gettimeofday(&tvend, NULL);
        //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
        Tnow = ((tvend.tv_sec * 1000000 + tvend.tv_usec)
                - (tvstart.tv_sec * 1000000 + tvstart.tv_usec));
        //printf("timeofday>%ld\n",Tnow );
        gettimeofday(&tvstart, NULL);       
        endTimer = Time::now();
        double interval = (endTimer - startTimer) * 1000000; //interval in us
        startTimer = Time::now();
        */
        
        //unsigned long int lastleft = unmask_events.getLastTimestamp();
        //printf("lastTimestamp %08x \n", lastleft);

        /*
          unsigned long int lastleft = unmask_events.getLastTimestamp();
        lc = lastleft * COUNTERRATIO; 
        unsigned long int lastright = unmask_events.getLastTimestampRight();
        rc = lastright * COUNTERRATIO;
        
        if((lc >= 4294967295) || (rc >= 4294967295)) {
        verb = true;
        printf("wrapping %d, %d \n",lc,rc );
        }

 
        
        //synchronising the threads at the connection time
        if ((lfConverter->isValid())&&(!synchronised)) {
	    printf("Sychronised Sychronised Sychronised Sychronised ");
            //firstRun = false;
            minCount = lc - interval * dim_window; 
            //lfConverter->getEldestTimeStamp();                                                                   
            minCountRight = rc - interval * dim_window;
            printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
            startTimer = Time::now();
            synchronised = true;
            //minCount = unmask_events.getLastTimestamp();
            //printf("minCount %d \n", minCount);
            //minCountRight = unmask_events.getLastTimestamp();
            count = synch_time - 200;
        }

        //synchronising the thread every time interval 1000*period of the thread
        //if (count % synch_time == 0) {
        if (count == synch_time) {
            minCount = lc - interval * dim_window; //lfConverter->getEldestTimeStamp();        
            minCountRight = rc - interval * dim_window; 
            printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
            startTimer = Time::now();
            synchronised = true; 
        }
        else {
            // this value is simply the ration between the timestamp reported by the aexGrabber (62.5Mhz) 
            //and the correct timestamp counter clock of FPGA (50 Mhz)
            microsecondsPrev = interval;
            interval = Tnow;
            minCount = minCount + interval ; // * (50.0 / 62.5) * 1.10;
            minCountRight = minCountRight + interval;
        }   
             
        maxCount =  minCount + interval * dim_window;
        maxCountRight =  minCountRight + interval * dim_window;
        
        if(count % 100 == 0) { 
        //printf("countStop %d lcprev %d lc %d \n",countStop, lcprev,lc);
            if (lcprev == lc) { 
                countStop++;
		printf("countStop %d \n", countStop);
	    }            
            //else if (rcprev == rc) { 
            //    countStop++;
            //    printf("countStop %d \n", countStop);
            //}
	    else {
	      countStop--;
	      printf("countStop %d \n", countStop);
	      if(countStop<= 0)
		countStop = 0;
	    }
            lcprev = lc;
            rcprev = rc;
        }        
	
        //resetting time stamps at overflow
        if (countStop == 10) {
            //printf("resetting time stamps!!!!!!!!!!!!! %d %d   \n ", minCount, minCountRight);
            //lfConverter->resetTimestamps(); 
            verb = true;
            printf("countStop %d %d \n",countStop, verb );
	  
	    count = synch_time - 200;
        }

	    */
	
        int dimCD, dimEM, dimIF;
        aer *pCD, *pEM, *pIF;

        //TODO : code MUTEXes in these lines! Strictly Necessary!
        unmask_events.getCD(&pCD, &dimCD);
        if (dimCD > 0) {
            //printf("dimCD :  %d \n", dimCD);
            sendBuffer(&portCD, pCD, dimCD);
            unmask_events.resetCD();
        }
        
        // ----------------------------------------------- 
        
        unmask_events.getEM(&pEM, &dimEM);
        if (dimEM > 0) {
            //printf("dimEM :             %d \n", dimEM);
        }

        for (int i = 0; i < dimEM; i++) {
            u32 blob      = pEM[i].address;
            u32 timestamp = pEM[i].timestamp;
            fprintf(fout,"%08x %08x \n",blob,timestamp);
        }                                       

        //printf("dimEM :  %d \n", dimEM);
        sendBuffer(&portEM, pEM, dimEM);
        if(count % 100 == 0){
            //printf("_____________________________________ \n");            
            unmask_events.resetEM1();
            unmask_events.resetEM2();
            unmask_events.resetEM3();
            unmask_events.resetEM4();
            unmask_events.resetTOTEM();
        }
        
        // --------------------------------------------
        
        
        unmask_events.getIF(&pIF, &dimIF);
        if (dimIF > 0) {
            printf("dimIF :                                 %d \n", dimIF);
            sendBuffer(&portIF, pIF, dimIF);
            unmask_events.resetIF();
        }

        
   
    
        // measuring execution time of the module
        tend = Time::now();
        difftime2 = tend - tinit;
        //if(count % 100 == 0) {
        //    printf("time us: %f %f %d %d %d \n", difftime1 * 1000000, difftime2 * 1000000, dimCD, dimEM, dimIF);
        //}
    }
}

/**
 * @param size : number of events to be sent
 */
void logSortThread::sendBuffer(BufferedPort<eventBuffer>* port, aer* buffer, int sz) {
    int szSent = sz * 8; // dimension of the event times the bytes per event
    //aer* copyEvent = buffer;
    
    // unsigned char* copyBuffer = (unsigned char*) buffer;
    //for (int i = 0; i < szSent; i++) {
    //    if (*copyBuffer < 0) {
    //        *copyBuffer = 256 - *copyBuffer;
    //    }
    //    fprintf(fout," %d %X \n",*copyBuffer, *copyBuffer);
    //    copyBuffer++;
    //}
    
    
    if (port->getOutputCount()) {           
        char* pBuffer = (char*) buffer;
       
        /*for (int i = 0; i < sz; i++) {
            u32 blob      = buffer[i].address;
            u32 timestamp = buffer[i].timestamp;
            fprintf(fout,"%08x %08x \n",blob,timestamp);
            }*/
               
        //printf("sending : %d 0x%x \n", szSent, buffer);
        eventBuffer data2send(pBuffer, szSent);    
        eventBuffer& tmp = port->prepare();
        tmp = data2send;   
        port->write();    
    }     
}



/*
void logSortThread::run() {
    count++;
    if(count == 100000) {
        count = 0;
    }
    if(!synchronised) {
        countDivider = count; 
    }
    
    //T2 = times(&stop_time);
    unsigned long int lastleft = lfConverter->getLastTimeStamp();
    lc = lastleft * 1.25; //1.25 is the ratio 0.160/0.128
    unsigned long int lastright = lfConverter->getLastTimeStampRight();
    rc = lastright * 1.25;

    //gettimeofday(&tvend, NULL);
    //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    
    endTimer = Time::now();
    double interval = (endTimer - startTimer) * 1000000; //interval in us
    startTimer = Time::now();

    //clock_gettime(CLOCK_REALTIME, &stop_time );
    //double diffTime = (endTime - startTime);
    //printf("timeofday>%ld\n", ((tvend.tv_sec * 1000000 + tvend.tv_usec)
	//	  - (tvstart.tv_sec * 1000000 + tvstart.tv_usec)));
   
    //double time = (double)stop_time.tms_utime - start_time.tms_utime;
    //microseconds = stop_time.tv_nsec / 1000 ; 
    
    //gettimeofday(&tvstart, NULL);
    //startTime = clock();
    //T1 = times(&start_time);
    //clock();
    //clock_gettime( CLOCK_REALTIME, &start_time );
    
    
    if ((lfConverter->getInputCount()) && (!synchronised)) { 
        minCount = lc - interval * 2; //lfConverter->getEldestTimeStamp();        
        minCountRight = rc - interval * 2;
        printf("synchronised! %d,%d,%d||%d,%d,%d \n", minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true;    
    }
    else if (count % 1000 == 0) {
        minCount = lc - interval * 2; //lfConverter->getEldestTimeStamp();        
        minCountRight = rc - interval * 2; 
        printf("synchronised! %d,%d,%d||%d,%d,%d \n", minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true;  
    }
    else {
        minCount = minCount + interval ; // * (50.0 / 62.5) * 1.10;
        minCountRight = minCountRight + interval;
    }

    if ((lc > 10000000)||(rc>10000000)) {
        greaterHalf = true;
    }
    else if(((lc < 10000000)||(rc < 10000000))&&(greaterHalf)) {
        greaterHalf = false;
        lfConverter->resetTimestamps();
        printf("resetting time stamps!!!!!!!!!!!!!");
    }

             
    // this value is simply the ration between the timestamp reported by the aexGrabber (62.5Mhz) 
    //and the correct timestamp counter clock of FPGA (50 Mhz)
    maxCount =  minCount + interval * 5;
    maxCountRight =  minCountRight + interval * 5;
    if( count % 100 == 0) {
        printf("greterHalf:%d! %d,%d,%d||%d,%d,%d \n",greaterHalf, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
    }
    precl = lc;

    
    microsecondsPrev = microseconds;
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
        if(&outputImage!=0) {
            lfConverter->getMonoImage(&outputImage, minCount, maxCount,1);
            outPort.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }

    if(outPortRight.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImageRight=outPortRight.prepare();
        if(&outputImageRight!=0) {
            lfConverter->getMonoImage(&outputImageRight, minCountRight, maxCountRight, 0);
            outPortRight.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }
    //minCount = lfConverter->getLastTimeStamp(); //get the last before going to sleep
    
}
*/

void logSortThread::threadRelease() {
    idle = false;
    printf("Threadrelease:freeing bufferCopy \n");
    //free(bufferCopy);
    printf("Threadrelease:closing ports \n");
    outPort.close();
    portCD.close();
    portEM.close();
    portIF.close();
    outPortRight.close();
    //delete imageLeft;
    //delete imageRight;
    printf("Threadrelease         stopping plotterThread \n");
    pThread->stop();
    printf("Thread releas         stopping unmaskThread \n");
    printf("Threadrelease         deleting converter \n");
    delete lfConverter;
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

