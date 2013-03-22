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
#include <cmath>
#include <time.h>

using namespace emorph::ecodec;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define FIRETHRESHOLD 200            // value of the fire threshold in the saliency map [0,255]
#define INTERVFACTOR  1
#define COUNTERRATIO  1              //1.25 is the ratio 0.160/0.128
#define MAXVALUE      0xFFFFFF       //4294967295
#define THRATE        1
#define CHUNKSIZE     32768          //65536 //8192
#define dim_window    10
#define synch_time    1
#define COMMCOUNTDEF  20

//#define VERBOSE
#define STOREWTA
#define STOREINI
#define INCR_RESPONSE 0
#define DECR_RESPONSE 0

eventSelectorThread::eventSelectorThread() : RateThread(THRATE) {
    responseGradient = 127;
    retinalSize      = 128;  //default value before setting 
    lasttimestamp    = 0;
    count            = 0;
    minCount         = 0;    //initialisation of the timestamp limits of the first frame
    countStop        = 0;
    countCommands    = 30;    
    
    lc               = 0;	
    rc               = 0; 
    minCount         = 0;
    minCountRight    = 0;

    maxDistance      = 255;

    forgettingFactor = 0.00; //in the range [0.0,1.0]
        
    bottleHandler = true;
    synchronised = false;
    greaterHalf  = false;
    firstRun     = true;
    
    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);

    verb = false;
    plotLatency = true;

    string i_fileName("eventSelectorThread.events.log");
    string w_fileName("eventSelectorThread.wta.log");
    string n_fileName("eventSelectorThread.ini.log");
    string l_fileName("eventSelectorThread.latency.txt");
    raw         = fopen(i_fileName.c_str(), "wb");
    fstore      = fopen(w_fileName.c_str(), "wb"); 
    istore      = fopen(n_fileName.c_str(), "wb"); 
    latencyFile = fopen(l_fileName.c_str(), "wb");

}

eventSelectorThread::~eventSelectorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool eventSelectorThread::threadInit() {
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
    
    pThread = new plotterThread();
    pThread->setName(getName("").c_str());
    pThread->setStereo(stereo);
    pThread->setRetinalSize(retinalSize);
    pThread->start();

    /*
    ebHandler = new eventBottleHandler();
    ebHandler->useCallback();
    ebHandler->setRetinalSize(128);
    ebHandler->open(getName("/retinaBottle:i").c_str());
    map1Handler = new eventBottleHandler();
    map1Handler->useCallback();
    map1Handler->setRetinalSize(32);
    map1Handler->open(getName("/map1Bottle:i").c_str());
    */


    startTimer = Time::now();
    //gettimeofday(&tvend, NULL);

    count = 0;
    microsecondsPrev = 0;
    minCount = 0;
    minCountRight= 0;
    
    saliencyMapLeft    = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    saliencyMapRight   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap41Left   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap41Right  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap42Left   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap42Right  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap43Left   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMap43Right  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMapA1Left   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMapA1Right  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMapA2Left   = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    featureMapA2Right  = (double*) malloc(retinalSize * retinalSize * sizeof(double));
    
    memset(saliencyMapLeft  ,0, retinalSize * retinalSize * sizeof(double));
    memset(saliencyMapRight ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap41Left ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap41Right,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap42Left ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap42Right,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap43Left ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMap43Right,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMapA1Left ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMapA1Right,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMapA2Left ,0, retinalSize * retinalSize * sizeof(double));
    memset(featureMapA2Right,0, retinalSize * retinalSize * sizeof(double));
    
    
    featureMap = (int*) malloc(retinalSize * retinalSize * sizeof(int));
    memset(featureMap,0, retinalSize * retinalSize * sizeof(int));
    
    timestampMapLeft    = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );  
    timestampMap41Left  = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMap41Right = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMap42Left  = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMap42Right = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMap43Left  = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMap43Right = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMapA1Left  = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMapA1Right = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMapA2Left  = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    timestampMapA2Right = (unsigned long*) malloc(retinalSize * retinalSize * sizeof(unsigned long) );
    
    memset(timestampMapLeft    ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap41Left  ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap41Right ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap42Left  ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap42Right ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap43Left  ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMap43Right ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMapA1Left  ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMapA1Right ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMapA2Left  ,0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(timestampMapA2Right ,0, retinalSize * retinalSize * sizeof(unsigned long));

    unmaskedEvents = (AER_struct*) malloc (CHUNKSIZE * sizeof(int));
    memset(unmaskedEvents, 0, CHUNKSIZE * sizeof(int));

    // common resource necessary to define the temporal horizon
    lasttimestamp  = new unsigned long; 
    *lasttimestamp = 0;

    bptA1 = 0;
    bptA2 = 0;
    bpt41 = 0;
    bpt42 = 0;
    bpt43 = 0;
    
    // istantiating the processors of events
    bptA1 = new bottleProcessorThread();
    bptA1->setLastTimestamp(lasttimestamp);
    bptA1->setSaliencySize(retinalSize);  
    bptA1->setRetinalSize(retinalSize);   
    bptA1->setName(getName("/btpA1"));
    bptA1->start();    
    
    bptA2 = new bottleProcessorThread();
    bptA2->setLastTimestamp(lasttimestamp);
    bptA2->setSaliencySize(retinalSize);  
    bptA2->setRetinalSize(retinalSize);   
    bptA2->setName(getName("/btpA2"));
    bptA2->start();

    bpt41 = new bottleProcessorThread();
    bpt41->setSaliencySize(retinalSize);  
    bpt41->setRetinalSize(32);
    bpt41->setName(getName("/btp41"));    
    bpt41->start();

    bpt42 = new bottleProcessorThread();
    bpt42->setSaliencySize(retinalSize);  
    bpt42->setRetinalSize(32);
    bpt42->setName(getName("/btp42"));    
    bpt42->start();

    bpt43 = new bottleProcessorThread();
    bpt43->setSaliencySize(retinalSize);  
    bpt43->setRetinalSize(32);
    bpt43->setName(getName("/btp43"));    
    bpt43->start();
    
    outputCmdPort.open(getName("/cmd:o").c_str());

    receivedBottle = new Bottle();
    unmask_events  = new unmask(32);

    timeStart = Time::now();

    printf("Initialisation in selector thread correctly ended \n");
    return true;
}

void eventSelectorThread::interrupt() {
    outPort.interrupt();
    outPortRight.interrupt();
    outputCmdPort.interrupt();
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
    imageLeft = new ImageOf<PixelRgb>;
    imageLeft->resize(retinalSize,retinalSize);
    imageRight = new ImageOf<PixelRgb>;
    imageRight->resize(retinalSize,retinalSize);
    imageLeftBW = new ImageOf<PixelMono>;
    imageLeftBW->resize(retinalSize,retinalSize);
    imageRightBW = new ImageOf<PixelMono>;
    imageRightBW->resize(retinalSize,retinalSize);
}


void eventSelectorThread::getMonoImage(ImageOf<yarp::sig::PixelRgb>* image, unsigned long minCount,unsigned long maxCount, bool camera){
    assert(image!=0);
    //printf("retinalSize in getMonoImage %d \n", retinalSize);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding      = image->getPadding();
    int imageRowSize      = image->getRowSize();
    
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
    //int* pBuffer = unmask_events->getEventBuffer(camera);
    //unsigned long* pTime   = unmask_events->getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    double maxLeft = -100, maxRight = -100;
    double minLeft =  100, minRight =  100;
    double maxResponseLeft = 0, maxResponseRight = 0;
    int maxLeftR, maxLeftC, maxRightR, maxRightC;
    unsigned int value;
   
    // copying the feature map for any bottleProcessor connected to the input flow of events    
    if(bpt41!= 0){
        bpt41->copyFeatureMapLeft(featureMap41Left);
    }
    if(bpt42!=0) {
        bpt42->copyFeatureMapLeft(featureMap42Left);
    }
    if(bpt43!=0) {
        bpt43->copyFeatureMapLeft(featureMap43Left);
    }
    if(bptA1!=0) {
        bptA1->copyFeatureMapLeft(featureMapA1Left);
    }
    //bptA2->copyFeatureMapLeft(featureMapA2Left);


    // copying the map of timestamps
    if(bpt41!= 0){
        bpt41->copyTimestampMapLeft(timestampMap41Left);
    }
    if(bpt42!= 0){
        bpt42->copyTimestampMapLeft(timestampMap42Left);
    }
    if(bpt43!= 0){
        bpt43->copyTimestampMapLeft(timestampMap43Left);
    }
    if(bptA1!= 0){
        bptA1->copyTimestampMapLeft(timestampMapA1Left);
    }
    //bptA2->copyTimestampMapLeft(timestampMapA2Left);
    
    
    unsigned long last41 = 0;
    if(bpt41!= 0){
        last41=  bpt41->getLastTimestamp();
    }
    unsigned long last42 = 0;
    if(bpt42!= 0){
        last42=  bpt42->getLastTimestamp();
    }
    unsigned long last43 = 0;
    if(bpt43!= 0){
        last43=  bpt43->getLastTimestamp();
    }
    unsigned long lastA1 = 0;
    if(bptA1!= 0){
        lastA1=  bptA1->getLastTimestamp();
    }

    unsigned long last_ts;
    if(last41 > last42)
        last_ts = last41;
    else if( last42 > lastA1)
        last_ts = last42;
    else
        last_ts = lastA1;

    //#ifdef STOREWTA
    //fprintf(fstore," %lu %lu %lu > %lu \n",last41,last42,lastA1, last_ts);
    //#endif
    
    
    unsigned long  timestampactual;                      // timestamp of the current selected location
    unsigned long  maxtimestamp;                         // timestamp associated to the max value
    unsigned long* pTime41Left  = timestampMap41Left;
    unsigned long* pTime42Left  = timestampMap42Left;
    unsigned long* pTime43Left  = timestampMap43Left;
    unsigned long* pTimeA1Left  = timestampMapA1Left;
    unsigned long* pTimeA2Left  = timestampMapA2Left;
    unsigned long* pTimeLeft    = timestampMapLeft;

    double* pMap41Left          = featureMap41Left;
    double* pMap42Left          = featureMap42Left;
    double* pMap43Left          = featureMap43Left;
    double* pMapA1Left          = featureMapA1Left;
    double* pMapA2Left          = featureMapA2Left;
    double* pBufferLeft         = saliencyMapLeft;

    /*
    for(int j = 0; j < retinalSize * retinalSize; j++){
        //if(*pTimeA1Left!=0)
        //    printf("read j %d  %08x \n" , j,*pTimeA1Left);
        *pTimeA1Left = minCount+1;
        pTimeA1Left++;
    }
    pTimeA1Left = timestampMapA1Left;
    */
    
    maxDistance = 100; // resetting the max distance to the max value
    

    /* BEWARE! REMEMBER: the feature map received are double value in the range [-1.0, 1.0]
       However in the next analysis it does not matter whether the contribution is positive or negative.
       The more distant from 0 the more salient. It suffices to extract the abs(value) as a measure of saliency
       The pixel value is immediately forced in the range [0.0, 1.0] using abs operator
     */

    unsigned int maxValue = 0;   /*  MAX VALUE REGISTER of saliency map [0, 255] initialised to 0 */

    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {            
            // combining the feature map and normalisation            
            double contrib41Left = abs(*pMap41Left); 
            double contrib42Left = abs(*pMap42Left);
            double contrib43Left = abs(*pMap43Left);
            double contribA1Left = abs(*pMapA1Left);
            double contribA2Left = abs(*pMapA2Left); 

            // forgetting factor decrement
            if(contrib41Left > forgettingFactor)  {
                contrib41Left -= forgettingFactor;
            }
            if(contrib42Left > forgettingFactor)  {
                contrib42Left -= forgettingFactor;
            }
            if(contrib43Left > forgettingFactor)  {
                contrib43Left -= forgettingFactor;
            }
            if(contribA1Left > forgettingFactor)  {
                contribA1Left -= forgettingFactor;
            }
            if(contribA2Left > forgettingFactor)  {
                contribA2Left -= forgettingFactor;
            }

            //double contrib41Left = (abs(*pMap41Left) - bpt41->getMinLeft()) / (bpt41->getMaxLeft() - bpt41->getMinLeft());
            //double contribA1Left = (abs(*pMapA1Left) - bptA1->getMinLeft()) / (bptA1->getMaxLeft() - bptA1->getMinLeft());     

            //printf("%f %f %f \n",contribA1Left, bptA1->getMaxLeft(), bptA1->getMinLeft() );
            
            // if(*pMapA1Left == 0) 
            //    contribA1Left = 0;
            
            //TODO : check that every feature maps remains into the limits [0, 1.0]
            //if(*pMap41Left >1.0) { 
            //    printf("pMap41Left %f \n", *pMap41Left);
            //}

            
            double wa1 = 0.0;
            double wa2 = 0.0;
            double w41 = 1.0;
            double w42 = 0.0;
            double w43 = 0.0;
            //*pBufferLeft =  contrib41Left ;
            *pBufferLeft =  wa1 * contribA1Left + wa2 * contribA2Left + 
                w41 * contrib41Left + w42 * contrib42Left + w43 * contrib43Left;        
            //printf("wa1 %f contribA1Left %f w41 %f contrib41Left %f w42 %f contrib42Left %f \n ",wa1, contribA1Left, w41,contrib41Left, w42, contrib42Left );
            if(*pBufferLeft > 1.0) {
                *pBufferLeft = 1.0;
            }


            //double left_double  = saliencyMapLeft [r * retinalSize + c];
            //double right_double = saliencyMapRight[r * retinalSize + c];
            //left_double = featureMap41Left[r * retinalSize + c];            
            //if(value > 255) {
            //    printf("overflow %f %f %f \n", contrib41Left, contribA1Left,*pBuffer);
            //}

            double right_double = 0;
            double left_double  = 0;

            // DEPREC -------------- max and min of left and right image  -------------------
            left_double  = *pBufferLeft;
            //if(left_double  < minLeft)   minLeft = left_double;
            //if(left_double  > maxLeft)   maxLeft = left_double;
            //if(right_double > maxRight)  maxRight = right_double;
            //if(right_double < minRight)  minRight = right_double;
            //------------------------------------------------------------------------------


            //--------------------------------------------------------------------------
            //if(maxResponseLeft < abs(minLeft)) {
            //    maxResponseLeft = abs(minLeft);
            //    maxLeftR = r; maxLeftC = c;
            //}

            // --------- weight biases distance from the center of the saliency map -----------
            //double a, b;
            //a = b = (double) (retinalSize >> 1); 
            //double max_distance = sqrt (a * a + b * b) + 50;
            //a -= r;
            //b -= c;           
            //double distance = sqrt (a * a + b * b);
            //double weight = 1.0 + (1 - (distance / max_distance)) / 4 ;
            // weight in the range between 0.35 and 0.9
            //---------------------------------------------------------------------------------
            
            
            /* deprec
            if(maxResponseLeft  < (abs(left_double))) {
                //printf("%f %f \n", distance, maxDistance);
                //if (distance < maxDistance) {
                    maxResponseLeft  = abs(left_double);
                    maxDistance      = distance;
                    maxLeftR = r; maxLeftC = c;
                    //}
            } 
            //if(maxResponseRight < abs(minRight)) {
            //    maxResponseRight = abs(minRight);
            //    maxRightR = r; maxRightC = c;
            //}
            if(maxResponseRight  <= abs(maxRight)) {
                maxResponseRight = abs(maxRight);
                maxRightR = r; maxRightC = c;
            } 
            */

            //--------------- temporal information ------------------------------------
            unsigned long ta1l = *pTimeA1Left;
            unsigned long t41l = *pTime41Left;
            unsigned long t42l = *pTime42Left;
            unsigned long t43l = *pTime43Left;
            
            timestampactual = 0;
            
            if(t41l > timestampactual){
                timestampactual = t41l;
            }
            //else if(t42l > timestampactual){
            //    timestampactual = t42l;
            //}
            //else if(t43l > timestampactual){
            //    timestampactual = t43l;
            //}
            else if( ta1l > timestampactual){
                timestampactual = ta1l;
            }
               

            //timestampactual   = (*pTimeA1Left > *pTime41Left)?*pTimeA1Left:*pTime41Left;
            //timestampactual   = *pTimeA1Left;
            timestampactual   = *pTime41Left;

            
            //--------------------------------------------------------------------------


            //--------------- conversion  --------------------------------------------
            /** BEWARE! REMEMBER : In the previous section the saliency map is double register in the range [0.0, 1.0];
                Towards 1.0 : high saliency
                Towards 0.0 : low  saliency

                From now on, the register value is mapped in to unsigned int in the range [0, 255];
                This allows for the creation of an grey-scale image representing the saliency map.
             */            
            
            value = left_double * 255;         // CAUTION : casting of a double into an unsigned int!!!!!!!!!!!!!!!
            // movieng from pBufferleft (saliencyImageLeft) to pImage(Image)
            //------------------------------------------------------------------------

            

            //----------------------------------------------------------------------------------------
            if(tristate) {
                
                // -----(NOT ENABLED)--- FORGETTING FACTOR -----------------------------------------
                // decreasing the spatial response without removing it
                // without forgetting factor the event can be present for the timestamp but it is always presenting the max value
                /*
                if(count % 1 == 0) {
                // old
                //    if(*pBufferLeft > 0.1){
                //        left_double = left_double - 0.1;
                //        
                //   }
                //  else if(*pBufferLeft < -0.1){
                //     printf("less than -100 \n");
                //    left_double = left_double + 0.1;                                             
                //  }
                // old

                    if(left_double > 0.99){
                        left_double = left_double - 0.99;
                                                          
                    }
                    else if(left_double < -0.8){
                        printf("less than -100 \n");
                        left_double = left_double + 0.99;
                                                      
                    }
                }
               */
               // ----(NOT ENABLED)------------------------------------------------------------------


                //if(minCount>0 && maxCount > 0 && timestampactual>0)
                //printf("actualTS %08X val %08X max %08X min %08X  are\n",timestampactual,timestampactual * COUNTERRATIO,minCount,maxCount);

                if ( 
                    ((timestampactual * COUNTERRATIO) > minCount) && ((timestampactual * COUNTERRATIO) < maxCount)
                    ) {   //(timestampactual != lasttimestamp)

                    *pImage = (unsigned char) (value);
                    pImage++;
                    *pImage = (unsigned char) (value);
                    pImage++;
                    *pImage = (unsigned char) (value);
                    pImage++;
                    
                    
                    if(value > maxValue){
                        maxValue = value;
                        maxLeftR = r; 
                        maxLeftC = c;
                        maxtimestamp = timestampactual;
                    }

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
                    pImage++;
                    *pImage = (unsigned char) 0 ;
                    pImage++;
                    *pImage = (unsigned char) 0 ;
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
                pBufferLeft++;
                pMap41Left++;
                pMap42Left++;
                pMap43Left++;
                pMapA1Left++;
                //---
                pTime41Left++;
                pTime42Left++;
                pTime43Left++;
                pTimeA1Left++;
            }
            else { 

                // ------------------------- branch !tristateView ------------------------

                // -----(NOT ENABLED)--- FORGETTING FACTOR -----------------------------------------
                // decreasing the spatial response without removing 
                //if((*pBufferLeft > 20) && (count % 10 == 0)){
                //    *pBufferLeft = *pBufferLeft - 1;                                       
                //}
                // -----(NOT ENABLED)-----------------------------------------------------------------

                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   
                    *pImage =  value ;
                    //if(value>0)printf("event%d val%d buf%d\n",*pImage,value,*pBuffer);
                    pImage++;
                    
                    //TODO:  the output image is RGB image, still address other color pixels
                    

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
                    *pImage = (unsigned char) 10 ;
                    //printf("NOT event%d \n",*pImage);
                    pImage++;

                    //TODO:  the output image is RGB image, still address other color pixels

                    
                    //if ((stereo) && (r < 7) && (r >= 16) && (c < 7) && (c >= 16)) {
                    //  *pImage = (unsigned char) 127;
                    //  pImage += imageRowSize;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage--;
                    //  *pImage = (unsigned char) 127 + value;
                    //  pImage -= (imageRowSize + 1);
                    //}
                    
                }
                pBufferLeft++;
                pMap41Left++;
                pMap42Left++;
                pMap43Left++;
                pMapA1Left++;
                //--
                pTime41Left++;
                pTime42Left++;
                pTime43Left++;
                pTimeA1Left++;
                
            } // end !tristate            
        } // end inner loop
        pImage += imagePadding;
    }//end outer loop

    //printf("A1 > %f %f \n", bptA1->getMinLeft(),  bptA1->getMaxLeft());
    //printf("41 > %f %f \n", bpt41->getMinLeft(),  bpt41->getMaxLeft());
    
    //printf("end of the function get in mono \n");
    //unmask_events->setLastTimestamp(0);
    //printf("maxLR %d  maxLC%d \n",maxLeftR, maxLeftC );
    
    // cycle after normalisation - WTA representation
    //printf("cycle after the normalisation LEFT:(%f, %f)  RIGHT:(%f,%f) \n", minLeft, maxLeft, minRight, maxRight);
    double* pSalLeft        = saliencyMapLeft;
    double* pSalRight       = saliencyMapRight;
    //unsigned long* pTimeLeft     = timestampMapLeft;
    //unsigned long* pTimeRight    = timestampMapRight;
    double rangeLeft        = abs(maxLeft  - minLeft);
    double rangeRight       = abs(maxRight - minRight);
    unsigned char* pLeft    = imageLeft    ->getRawImage();
    unsigned char* pRight   = imageRight   ->getRawImage();
    unsigned char* pLeftBW  = imageLeftBW  ->getRawImage();
    unsigned char* pRightBW = imageRightBW ->getRawImage();
    int padding             = imageLeft    ->getPadding();
    int rowSize             = imageLeft    ->getRowSize();
    int rowSizeBW           = imageLeftBW  ->getRowSize();

    imageLeftBW->zero();
    imageRightBW->zero();


    if(plotLatency) {
        timeStop = Time::now();
        double latency = timeStop - timeStart;
        fprintf(latencyFile, "%f \n", latency);
        
        timeStart = Time::now();
    }

               
   
    // if(maxResponseLeft > FIRETHRESHOLD) {         
    if(maxValue >= FIRETHRESHOLD) {  

#ifdef STOREWTA
        /** 
         *saving in a file the list of WTA position and their timestamp (X,Y) for the left camera
         */
        fprintf(fstore, "%d %d %lu \n", maxLeftC,maxLeftR, maxtimestamp);
#endif
  
        //printf("maxResponseLeft %f position %d %d \n",maxResponseLeft,maxLeftC,maxLeftR  );
        // sending command for saccade; to focus redeployment corresponds fixation point reallocation 
        if(outputCmdPort.getOutputCount()){

            // the countCommands prevents from sending frequent sequences of commands
            if(countCommands >= COMMCOUNTDEF) {
                Bottle& commandBottle=outputCmdPort.prepare();
                commandBottle.clear();
                commandBottle.addString("SAC_MONO");
                commandBottle.addInt(maxLeftC);
                commandBottle.addInt(maxLeftR);
                commandBottle.addDouble(0.5);
                commandBottle.addDouble(1.0);
                //commandBottle.addDouble(timestampactual);
                outputCmdPort.write();
            }
                
            countCommands--;
            if(countCommands <= 0) {
                countCommands = COMMCOUNTDEF;
            }
            
        }

        // representing the WTA on a BW saliency map image
        pLeftBW += (maxLeftR - 1)  * rowSizeBW  + (maxLeftC - 1);
        for (int r = 0 ; r < 3; r++) {
            for (int c = 0 ; c < 3 ; c++) {
                *pLeftBW = 255;
                pLeftBW++;
            }
            pLeftBW += rowSizeBW - 3; 
        }


        //pLeft +=  (maxLeftR - 1) * rowsize + (maxLeftC - 1) * 3 ;

        
        // (TODO : make it more efficient with jump to correct location) representing the WTA as red dot 
        for(int r = 0 ; r < retinalSize ; r++){
            for(int c = 0 ; c < retinalSize ; c++) {
                timestampactual = *pTimeLeft; 
                if (
                    ((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)
                    ) {
                    
                    if ((r == maxLeftR - 1 ) && (c == maxLeftC - 1)) {
                        for (int j = 0; j < 3; j ++) {
                            // maximum response in saliency map;
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            pLeft += rowSize - 3 * 3;
                        }
                    }                    
                    else {
                        //double d = ((abs(*pSalLeft  - minLeft) )  / rangeLeft ) * 80;
                        //printf(" d=%f %08X < %08X < %08X\n", d, minCount, timestampactual, maxCount);                        
                        pLeft   += 3;
                    }
                    
                }
                else {
                    if ((r == maxLeftR - 1) && (c == maxLeftC - 1)) {
                        for (int j = 0; j < 3; j ++) {
                            // maximum response
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 255; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            *pLeft = 0  ; pLeft++;
                            pLeft += rowSize - 3 * 3;
                        }
                    }
                    else {                    
                        pLeft++;
                        pLeft++;
                        pLeft++;
                    }
                    
                }
                // pLeft++;
                pSalLeft++;
                pTimeLeft++;

                /*
                // -----------------------  RIGHT WTA  ------------------------------
                timestampactual = *pTimeRight;
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) { 
                    if (r == maxRightR) {
                        *pRight = 255; pRight++;
                        *pRight = 0;   pRight++;
                        *pRight = 0;   pRight++;
                    }
                    else if(c == maxRightC) {
                        *pRight = 255; pRight++;
                        *pRight = 0;   pRight++;
                        *pRight = 0;   pRight++;
                    }
                    else {
                        double d = ((abs(*pSalRight - minRight) )/ rangeRight) * 80;
                        *pRight = (unsigned char) d; pRight++;
                        *pRight = (unsigned char) d; pRight++;
                        *pRight = (unsigned char) d; pRight++;
                    }
                }
                else {
                    if(r == maxRightR)  {
                        // maximum response
                        *pRight = 255; pRight++;
                        *pRight = 0;   pRight++;
                        *pRight = 0;   pRight++;
                    }
                    else if (c == maxRightC) {
                        *pRight = 255; pRight++;
                        *pRight = 0;   pRight++;
                        *pRight = 0;   pRight++;
                    }
                    else {
                        *pRight = 0; pRight++;
                        *pRight = 0; pRight++;
                        *pRight = 0; pRight++;
                    }
                }
                

                //pRight++;
                pSalRight++;
                pTimeRight++;
                */
                
            } //end inner for
            
        pLeft  += padding;
        pRight += padding;
        
        }//end outer for
    } // end if    
}

void eventSelectorThread::forgettingMemory() {
    double* pLeft = saliencyMapLeft;
    double* pRight = saliencyMapRight;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {            
            if(*pLeft > 0) {
                *pLeft -= DECR_RESPONSE;               
            }
            else {
                *pLeft += DECR_RESPONSE;
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


void eventSelectorThread::spatialSelection(eEventQueue *q) {
    int dequeSize = q->size();
    printf("eventSelectorThread::spatialSelection: %d events inQueue \n", dequeSize);
    unsigned long ts;
    int countLeftPos = 0, countLeftNeg = 0;
    int countRightPos = 0 , countRightNeg =0;
    for (int evt = 0; evt < dequeSize; evt++) {
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
                    int cartX     = retinalSize - ptr->getX();
                    int cartY     = ptr->getY();
                    int camera    = ptr->getChannel();
                    int polarity  = ptr->getPolarity();
                    
                    if(camera == 0) {
                        // memoriseLeft(cartX, cartY, polarity, ts); //sending event and last timestamp
                        //printf("saliencyMapLeft in %d %d changed (pol:%d)  \n", cartX, cartY, polarity);
                        if(polarity == 0) {
                            saliencyMapLeft [cartX * retinalSize + cartY] +=  INCR_RESPONSE;
                            countLeftPos++;
                        }
                        else {
                            saliencyMapLeft [cartX * retinalSize + cartY] -=  INCR_RESPONSE;
                            countLeftNeg++;
                        }
                        timestampMap41Left[cartX * retinalSize + cartY] = ts;
                    }
                    else {
                        //printf("saliencyMapRight in %d %d changed (pol:%d) \n", cartX, cartY, polarity);
                        //memoriseRight(cartX, cartY, polarity, ts); //sending event and last timestamp
                        
                        if(polarity == 0) {
                            saliencyMapRight [cartX * retinalSize + cartY] += INCR_RESPONSE; 
                            countRightPos++;
                        }
                        else {
                            saliencyMapRight [cartX * retinalSize + cartY] -= INCR_RESPONSE; 
                            countRightNeg++;
                        }
                        timestampMap41Right[cartX * retinalSize + cartY] = ts;
                        
                    }                                     
                } //end if (ptr->isValid()) 
            } //end if ((*q)[evt]->getType()=="AE")
            // -------------------------- TS ----------------------------------
            else if(eventInQueue->getType()=="TS") {
                
                TimeStamp* ptr=dynamic_cast<TimeStamp*>(eventInQueue);
                
                //identified an time stamp event
                ts = (unsigned int) ptr->getStamp();
                *lasttimestamp = ts;
                //printf("timestamp %lu \n", ts);

                forgettingMemory();
                
                //if(VERBOSE) {
                //    fprintf(fdebug, " %08x  \n", (unsigned int) ts);
                //}                
                //countEventToSend++;
            }
            // -------------------------- NULL ----------------------------------
            else {
                printf("not recognized \n");
                return;
            } 
        } //end if((*q)[evt] != 0)

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //deleting the event to avoid memory leak
        delete eventInQueue;
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
    } //end for

    //printf("number for left %d %d \n", countLeftPos, countLeftNeg);
    //printf("number for right %d %d \n", countRightPos, countRightNeg);
}

void eventSelectorThread::spatialSelection(AER_struct* buffer,int numberOfEvents, double w, unsigned long minCount, unsigned long maxCount) {
    printf("spatial selection using the AER_struct \n");
    
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
                    featureMap[pos]   = featureMap[pos] + 127;
                    if(featureMap[pos] > 127) {
                        featureMap[pos] = 127;
                    }
                }
                else {
                    featureMap[pos]   = featureMap[pos] - 127;
                    if(featureMap[pos] < -127) {
                        featureMap[pos] = -127;
                    }
                }
                
                //printf (" %08X \n", iter->ts);
                timestampMapLeft[pos] = iter->ts;            
            } 
        }
        iter++;
    }    
}

void eventSelectorThread::run() {
  count++;
  bottleHandler = true;
  if(!idle) {
    interTimer = Time::now();
    double interval2 = (interTimer - startTimer) * 1000000;
    // reads the buffer received
    //bufferRead = cfConverter->getBuffer();    
    // saves it into a working buffer


    /******MOVED TO THE SINGLE PROCESSOR
    //printf("checking the bottleHandler \n");
    //printf("============================================================= \n");
    if(!bottleHandler) {
        cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
    }
    else {
        //printf("eventSelectorThread::run : extracting Bottle! \n");
        //receivedBottle->clear();
        ebHandler->extractBottle(receivedBottle);  
        //printf("received bottle \n");
        //printf("%s \n", receivedBottle->toString().c_str());
    }
    *************************************/
    
    
    //======================== temporal synchronization pre-unmasking  =================================
    //printf("after extracting the bottle \n");
    // saving the buffer into the file
    int num_events = CHUNKSIZE / 8 ;
    u32* buf2 = (u32*)bufferCopy;

    // getting the time statistics
    endTimer = Time::now();
    double interval  = (endTimer - startTimer) * 1000000; //interval in us
    //double procInter = interval -interval2;
#ifdef STOREINI
    fprintf(istore,"\n %f ", interval);
#endif
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
    //printf("end of pre-unmasking synchronization \n ");
    

    
    /******MOVED TO THE SINGLE PROCESSOR
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
            //delete rxQueue;   // freeing memory for the new queue of events
            //printf("TODO : check for leaking \n");
            //rxQueue = new eEventQueue(); // preparing the new queue
            //rxQueue->clear();
            //printf("unmasking received bottle and creating the queue of event to send \n");
            unmask_events->unmaskData(receivedBottle, rxQueue); // saving the queue
            //printf("bottle of events to send : \n");
        }
    }
    ******************************/
        

    //==================== temporal synchronization post unmasking =======================   
#ifdef VERBOSE
    int num_events2 = CHUNKSIZE>>3;
    AER_struct* iter = unmaskedEvents;
    for (int evt = 0; evt < num_events2; evt++) {
        //unsigned long blob      = buf2[2 * evt];
        //unsigned long t         = buf2[2 * evt + 1];
        if(iter->ts!=0) {
            fprintf(fout," %08x %d %d \n", iter->ts, iter->x, iter->y);
        }
        //if((iter->x >=32) || (iter->y >= 32)) {
        //    printf("Error after unmasking %d %d \n", iter->x, iter->y);
        //}
        iter++;
    }
#endif

    //gettin the time between two threads
    //gettimeofday(&tvend, NULL);
    //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    //Tnow = ((tvend.tv_sec * 1000000 + tvend.tv_usec)
	//    - (tvstart.tv_sec * 1000000 + tvstart.tv_usec));
    //printf("timeofday>%ld\n",Tnow );
    //gettimeofday(&tvstart, NULL);       
            

    
    //synchronising the threads at the connection time
    unsigned long int lastleft, lastright;
    if ((cfConverter->isValid())&&(!synchronised)) {
        printf("Sychronising ");
        if(!bottleHandler) {
            lastleft  = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = *lasttimestamp;
        }
        lc = lastleft  * COUNTERRATIO; 
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
        if(!bottleHandler) {
            lastleft  = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = *lasttimestamp;  
        }
        lc = lastleft  * COUNTERRATIO; 
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
        if(!bottleHandler) {
            lastleft  = unmask_events->getLastTimestamp();
            lastright = unmask_events->getLastTimestampRight();
        }
        else {
            lastleft = lastright = *lasttimestamp;
        }
        
        lc = lastleft * COUNTERRATIO; 
        //unsigned long lastright = unmask_events->getLastTimestampRight();
        rc = lastright * COUNTERRATIO;
        printf("countStop %d lcprev %d lc %d \n",countStop, lcprev,lc);
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
    

    /******MOVED TO THE SINGLE PROCESSOR
    // =======================  spatial processing===========================
    
    double w1 = 0, w2 = 0, w3 = 0, w4 = 0;
    //spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1);       
    // spatial processing
    if(!bottleHandler) {
        //printf("spatial selection no bottle Handler \n");
        spatialSelection(unmaskedEvents,CHUNKSIZE>>3,w1, minCount, maxCount);
    }
    else {
        //printf("pre-spatial selection \n");
        if((rxQueue != 0) && (receivedBottle->size() != 0)) {
            //printf("spatial selection  bottle Handler\n");
            // spatialSelection(rxQueue);
        }
    }
    *********************************/
    
    
    // the getMonoImage gets as default input image the saliency map    
    if(imageLeft != 0) {
        
        getMonoImage(imageLeft,minCount,maxCount,1);
        if(imageLeftBW != 0) {
            pThread->copyLeftBW(imageLeftBW);
        }
        //printf("copying the right \n");
        pThread->copyLeft(imageLeft);
    }   
    
    if(stereo) {
        if(imageRight != 0) {
            //printf("getting the right image \n");
            getMonoImage(imageRight,minCountRight,maxCountRight,0);
            //printf("copying the right image \n");
            pThread->copyRight(imageRight);
        }
        if(imageRightBW != 0) {
            pThread->copyRightBW(imageRightBW);
        }
    }

    // debug check point! Checking the time interval required for processing
    //endTimer = Time::now();
    //double intervalProc  = (endTimer - interTimer) * 1000000; //interval in us
    //printf("processing time %f \n", intervalProc);

  }
}

void eventSelectorThread::threadRelease() {
    idle = false;
    fclose(fout);
    fclose(fstore);
    fclose(istore);
    fclose(latencyFile);
    printf("eventSelectorThread release:freeing bufferCopy \n");

    //delete imageLeft;
    //delete imageRight;    
    //delete imageLeftBW;
    //delete imageRightBW;
    
    free(saliencyMapLeft);
    free(saliencyMapRight);
    free(featureMap41Left);    
    free(featureMap41Right);
    free(featureMap42Left);    
    free(featureMap42Right);
    free(featureMap43Left);    
    free(featureMap43Right);
    free(featureMapA1Left);    
    free(featureMapA1Right);
    free(featureMapA2Left);    
    free(featureMapA2Right);
    free(featureMap);
    
    free(timestampMapLeft);
    free(timestampMap41Left); 
    free(timestampMap41Right); 
    free(timestampMap42Left); 
    free(timestampMap42Right);
    free(timestampMap43Left); 
    free(timestampMap43Right);
    free(timestampMapA1Left); 
    free(timestampMapA1Right);
    free(timestampMapA2Left); 
    free(timestampMapA2Right);
    free(unmaskedEvents); 
    
    printf("eventSelectorThread release:closing ports \n");
    outPort.close();
    outPortRight.close();
    outputCmdPort.close();

    printf("eventSelectorThread release         stopping Threads \n");
    pThread->stop();
    if(bptA1!=0)
        bptA1->stop();
    //if(bptA2!=0)
    //bptA2->stop();

    if(bpt41!=0)
        bpt41->stop();
    if(bpt42!=0)
        bpt42->stop();
    if(bpt43!=0)
        bpt43->stop();

    printf("eventSelectorThread release         deleting converter \n");
    //delete cfConverter;
    printf("correctly freed memory from the cfCollector \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

