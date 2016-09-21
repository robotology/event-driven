/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#include "vSaliencyMap.h"
#include <time.h>
#include <cassert>
#include <cmath>

#define THRATE        1
#define COUNTERRATIO  1              //1.25 is the ratio 0.160/0.128
#define FIRETHRESHOLD 200            // value of the fire threshold in the saliency map [0,255]
#define COMMCOUNTDEF  500
#define dim_window    10
#define INTERVFACTOR  1
#define synch_time    1

#define INCR_RESPONSE 0.09  //0.007    //0.07
#define DECR_RESPONSE 0.001 //0.01  //0.002



/* ---------------------------------------------------------------- */
/* ------------ vFeatureMap Processor ----------------------------- */
/* it is a buffered port that receives the vBottle from vFeatureMap */
/* ---------------------------------------------------------------- */

vFeatureMapProcessor::vFeatureMapProcessor(){
    
    strictness       = false;
    retinalSize      = 128;  //default value before setting
    saliencySize     = 128;  //default dimension of the saliency map
    
    maxLeft          = -1.0;
    minLeft          =  1.0;
    maxRight         = -1.0;
    minRight         =  1.0;
    
//    featureMapLeft   = 0;
//    timestampMapLeft = 0;
    
}

vFeatureMapProcessor::~vFeatureMapProcessor() {
    delete featureMapLeft;
    delete featureMapRight;
    delete timestampMapLeft;
    delete timestampMapRight;
    
}


bool vFeatureMapProcessor::open(const std::string moduleName, bool strictness){
    
    // set strictness of the communication
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    
    this->useCallback();
    
    // open input buffered port
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    return check1;
}

void vFeatureMapProcessor::close(){
    
    yarp::os::BufferedPort<emorph::vBottle>::close();
    
}

void vFeatureMapProcessor::interrupt(){
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    
}


//this is the entry point to your main functionality
void vFeatureMapProcessor::onRead(emorph::vBottle &bot){
    
    /* get the event queue in the vBottle bot */
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);
    
    // spatial processing
    spatialSelection(&q);
    
}

void vFeatureMapProcessor::spatialSelection(emorph::vQueue *q){
    
    int countLeftPos  = 0 , countLeftNeg  = 0;
    int countRightPos = 0 , countRightNeg = 0;

    int scaleFactor   = saliencySize / retinalSize;
  
    //    mutexMinMaxLeft.wait();
    maxLeft = 0.0;
    minLeft = 1.0;
    maxRight = 0.0;
    minRight = 1.0;
    //    mutexMinMaxLeft.post();
    
    // iterate the queue of events
    for(emorph::vQueue::iterator qi = q->begin(); qi != q->end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>(); // shall I do a getAs TimeStamp??? or the AE has the TS inside?
        if(!aep) continue;
        
        int cartX     = aep->getX();
        int cartY     = aep->getY();
        int camera    = aep->getChannel();
        int polarity  = aep->getPolarity();
        int ts        = aep->getStamp();
        
        if(scaleFactor == 4){
            if(cartX > 31 || cartY > 31) {
                std::cout << "ERROR in UNMASKING: cartX " << cartX << "cartY " << cartY << std::endl;
            }
        }
        
        // TODO: check the difference between representation of feature map and original flow
        int xpos,ypos;
        if(scaleFactor == 4) { // why? can't we do this for any scaleFactor?????
            xpos      = cartX * scaleFactor;
            ypos      = cartY  * scaleFactor;
        }
        else {
            xpos      = cartY;
            ypos      = saliencySize - cartX;
        }

        if(camera == 0) {
            
            /* ----------------------  LEFT CAMERA --------------------------------*/
            
            if(polarity == 0) {
                // CONTRIBUTION OF A POSITIVE EVENT increments saliency
                for ( int xi = 0; xi < scaleFactor; xi++) {
                    for (int yi = 0; yi < scaleFactor; yi++) {
                        
                        //mutexFeaLeft.wait();
                        double* pFea = &featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)];
                        if (*pFea < 1.0 - INCR_RESPONSE) {
                            *pFea += INCR_RESPONSE;
                        }
                        
                        //mutexMinMaxLeft.wait();
                        
                        if (*pFea > maxLeft) {
                            maxLeft = *pFea;
                        }
                        if (*pFea < minLeft) {
                            minLeft = *pFea;
                        }
                        //mutexMinMaxLeft.post();
                        //mutexFeaLeft.post();
                        countLeftPos++;
                        
                    }
                }
            }
            else {
                // CONTRIBUTION OF A NEGATIVE EVENT increments saliency
                for ( int xi = 0; xi < scaleFactor; xi++) {
                    for (int yi = 0; yi < scaleFactor; yi++) {
                        //mutexFeaLeft.wait();
                        double* pFea = &featureMapLeft[(ypos + yi) * saliencySize + (xpos + xi)];
                        
                        if (*pFea < 1.0 - INCR_RESPONSE) {
                            *pFea += INCR_RESPONSE;
                        }
                        
                        //mutexMinMaxLeft.wait();
                        if (*pFea > maxLeft) {
                            maxLeft = *pFea;
                        }
                        if (*pFea < minLeft) {
                            minLeft = *pFea;
                        }
                        //mutexMinMaxLeft.post();
                        //mutexFeaLeft.post();
                        countLeftNeg++;
                        
                    }
                }
            }
            //---------- timestamp for the left camera ----------------
            for ( int xi = 0; xi < scaleFactor; xi++) {
                for (int yi = 0; yi < scaleFactor; yi++) {
                    
                    //mutexTimeLeft.wait();
                    timestampMapLeft[(ypos + yi) * saliencySize + (xpos + xi)] = ts;
                    //mutexTimeLeft.post();
                    
                }
            }
            
        } // end left camera
        else {
            
            /* ----------------------  RIGHT CAMERA --------------------------------*/
            
            if(polarity == 0) {
                for ( int xi = 0; xi < scaleFactor; xi++) {
                    for (int yi = 0; yi < scaleFactor; yi++) {
                        //mutexFeaRight.wait();
                        double* pFea = &featureMapRight[(ypos + yi) * saliencySize + (xpos + xi)];
                        if(*pFea < 1.0 - INCR_RESPONSE) {
                            *pFea += INCR_RESPONSE;
                        }
                        //mutexMinMaxRight.wait();
                        if (*pFea > maxRight) {
                            maxRight = *pFea;
                        }
                        if (*pFea < minRight) {
                            minRight = *pFea;
                        }
                        //mutexMinMaxRight.post();
                        //mutexFeaRight.post();
                        countRightPos++;
                    }
                }
            }
            else {
                for ( int xi = 0; xi < scaleFactor; xi++) {
                    for (int yi = 0; yi < scaleFactor; yi++) {
                        //mutexFeaRight.wait();
                        double* pFea = &featureMapRight[(ypos + yi) * saliencySize + (xpos + xi)];
                        if(*pFea > -1.0 + INCR_RESPONSE) {
                            *pFea -= INCR_RESPONSE;
                        }
                        //mutexMinMaxRight.wait();
                        if (*pFea > maxRight) {
                            maxRight = *pFea;
                        }
                        if (*pFea < minRight) {
                            minRight = *pFea;
                        }
                        //mutexMinMaxRight.post();
                        //mutexFeaRight.post();
                        countRightNeg++;
                    }
                }
            }
            //---------- timestamp for the right camera ----------------
            for ( int xi = 0; xi < scaleFactor; xi++) {
                for (int yi = 0; yi < scaleFactor; yi++) {
                    //mutexTimeRight.wait();
                    timestampMapRight[(ypos + yi) * saliencySize + (xpos + xi)] = ts;
                    //mutexTimeRight.post();
                }
            }
        } // end right camera
    } // end queue iteration
    forgettingMemory();
    
}

void vFeatureMapProcessor::forgettingMemory() {
    
    double* pLeft  = featureMapLeft;
    double* pRight = featureMapRight;
    
    for(int r = 0 ; r < saliencySize ; r++){
        for(int c = 0 ; c < saliencySize ; c++) {
            
            if(*pLeft > DECR_RESPONSE) {
                *pLeft -= DECR_RESPONSE;
            }
            else if (*pLeft < -DECR_RESPONSE){
                *pLeft += DECR_RESPONSE;
            }
            else{
                *pLeft = 0;
            }
            pLeft++;
            
            if(*pRight > 0) {
                *pRight -= DECR_RESPONSE;
            }
            else if(*pRight < -DECR_RESPONSE) {
                *pRight += DECR_RESPONSE;
            }
            else {
                *pRight = 0;
            }
            pRight++;
        }
    }
    
}

void vFeatureMapProcessor::copyFeatureMapLeft(double *pointer) {
//    mutexFeaLeft.wait();
    if(0 == featureMapLeft) {
//        mutexFeaLeft.post();
        return;
    }
    double* pFea = featureMapLeft;
    for (int i = 0; i < saliencySize * saliencySize; i++) {

        // *pFea = 0;
        *pointer = *pFea;
        pointer++;  pFea++;
    }
//    mutexFeaLeft.post();
}

void vFeatureMapProcessor::copyTimestampMapLeft(unsigned long *pointer) {
    
//    mutexTimeLeft.wait();
    if(0 == timestampMapLeft) {
//        mutexTimeLeft.post();
        return;
    }
    unsigned long* pTime = timestampMapLeft;
    double *       pFea  = featureMapLeft;
    
    for (int i = 0; i < saliencySize * saliencySize; i++) {
     
        *pointer = *pTime;
     
        pointer++;
        pTime++;
        pFea++;
    }
    
//    mutexTimeLeft.post();
}




/* ---------------------------------------------------------------- */
/* ------------ vSaliencyMap Module ------------------------------- */
/* ------------ RF module ----------------------------------------- */
/* ---------------------------------------------------------------- */

/**********************************************************/
bool vSaliencyMapModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vSaliencyMap")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    //bool strict = rf.check("strict") &&
    //        rf.check("strict", yarp::os::Value(true)).asBool();
    
    /* attach a port of the same name as the module (prefixed with a /) to the module 
     so that messages received from the port are redirected to the respond method */
    
    std::string handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }
    
    attach(handlerPort);                  // attach to port

    /* set parameters */
    //int retSize = rf.check("retSize", yarp::os::Value(128)).asInt();
    
    /* create the thread and pass pointers to the module parameters */
    smManager = new vSaliencyMapManager();
    smManager->setName(getName().c_str());
    return smManager->start();

}

/**********************************************************/
bool vSaliencyMapModule::interruptModule()
{
    smManager->interrupt();
    handlerPort.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vSaliencyMapModule::close()
{
    handlerPort.close();
    smManager->stop();
    delete smManager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vSaliencyMapModule::updateModule()
{
    return true;
}

/**********************************************************/
double vSaliencyMapModule::getPeriod()
{
    return 1;
}
/**********************************************************/
bool vSaliencyMapModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) {
    std::string helpMessage =  std::string(getName().c_str()) +
    " commands are: \n" +
    "help \n" +
    "quit \n";
    
    reply.clear();
    
    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }
    return true;
}

/* ---------------------------------------------------------------- */
/* ------------ vSaliency Map Manager ----------------------------- */
/* it is a rate thread that combines the results from all of the    */
/* vFeatureMap Processors ----------------------------------------- */
/* ---------------------------------------------------------------- */

vSaliencyMapManager::vSaliencyMapManager() : RateThread(THRATE)
{
    strictness = false;

    responseGradient = 127;
    retinalSize      = 128;  //default value before setting
    lasttimestamp    = 0;
    count            = 0;
    iCount           = 0;
    minCount         = 0;    //initialisation of the timestamp limits of the first frame
    countStop        = 0;
    countCommands    = 30;
    
    lc               = 0;
    rc               = 0;
    minCount         = 0;
    minCountRight    = 0;
    
    maxDistance      = 255;
    
    forgettingFactor = 0.00; //in the range [0.0,1.0]
    
    synchronised = false;

    //plotLatency = false;
    tristate = true;
    
    std::string i_fileName("salMapManager.events.log");
    std::string w_fileName("salMapManager.wta.log");
    std::string n_fileName("salMapManager.ini.log");
    std::string l_fileName("salMapManager.latency.txt");
    raw         = fopen(i_fileName.c_str(), "wb");
    fstore      = fopen(w_fileName.c_str(), "wb");
    istore      = fopen(n_fileName.c_str(), "wb");
    latencyFile = fopen(l_fileName.c_str(), "wb");
    
}
/**********************************************************/

bool vSaliencyMapManager::threadInit() {
    
    // resize all of the images
    resizeAll(retinalSize, retinalSize);
    
//    printf("starting the plotter \n");
    
//    pThread = new plotterThread();
//    pThread->setName(getName("").c_str());
//    pThread->setStereo(stereo);
//    pThread->setRetinalSize(retinalSize);
//    pThread->start();
    
    //startTimer = Time::now();
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
    
    //unmaskedEvents = (AER_struct*) malloc (CHUNKSIZE * sizeof(int));
    //memset(unmaskedEvents, 0, CHUNKSIZE * sizeof(int));
    
    // common resource necessary to define the temporal horizon
    lasttimestamp  = new unsigned long;
    *lasttimestamp = 0;
    
    bptA1 = 0;
    bptA2 = 0;
    bpt41 = 0;
    bpt42 = 0;
    bpt43 = 0;
    
    // here we open a number of buffered ports, one for each feature map
    // deciding which feat map to use should be done from a config file TO DO
    std::string moduleName;
    // istantiating the processors of events
    bptA1 = new vFeatureMapProcessor();
    moduleName = "/" + name + "/bptA1";
    bptA1->open(moduleName, strictness);
    
    bptA2 = new vFeatureMapProcessor();
    moduleName = "/" + name + "/bptA2";
    bptA2->open(moduleName, strictness);
    
    bpt41 = new vFeatureMapProcessor();
    moduleName = "/" + name + "/bpt41";
    bpt41->open(moduleName, strictness);
    
    bpt42 = new vFeatureMapProcessor();
    moduleName = "/" + name + "/bpt42";
    bpt42->open(moduleName, strictness);
    
    bpt43 = new vFeatureMapProcessor();
    moduleName = "/" + name + "/bpt43";
    bpt43->open(moduleName, strictness);

    // open ports for output
    moduleName = "/" + name + "/cmd:o";
    outputCmdPort.open(moduleName);
    moduleName = "/" + name + "/vBottle:o";
    smEventsPort.open(moduleName);
    //receivedBottle = new Bottle();
    //unmask_events  = new unmask(32);
    
    //timeStart = Time::now();
    
    printf("Initialisation in saliency map manager rate thread correctly ended \n");
    return true;
}

void vSaliencyMapManager::interrupt() {
    // interrupt ports
    outputCmdPort.interrupt();
    smEventsPort.interrupt();
}

void vSaliencyMapManager::getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* image, unsigned long minCount,unsigned long maxCount, bool camera)
{
    assert(image!=0);
    
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding      = image->getPadding();
    //int imageRowSize      = image->getRowSize();
    
    
    // determining whether the camera is left or right
    //int* pBuffer = unmask_events->getEventBuffer(camera);
    //unsigned long* pTime   = unmask_events->getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;

//    double maxLeft = -100, maxRight = -100;
//    double minLeft =  100, minRight =  100;
//    double maxResponseLeft = 0, maxResponseRight = 0;
    int maxLeftR, maxLeftC;
    //int maxRightR, maxRightC;
    
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
    if(bptA2!=0) {
        bptA2->copyFeatureMapLeft(featureMapA2Left);
    }
    
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
    if(bptA2!=0) {
        bptA2->copyTimestampMapLeft(timestampMapA2Left);
    }
    
    /*
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
    unsigned long lastA2 = 0;
    if(bptA2!= 0){
        lastA2=  bptA2->getLastTimestamp();
    }
    */
    
    /*
    unsigned long last_ts;
    if(last41 > last42)
        last_ts = last41;
    else if(last42 > lastA1)
        last_ts = last42;
    else
        last_ts = lastA1;
    
    //#ifdef STOREWTA
    //fprintf(fstore," %lu %lu %lu > %lu \n",last41,last42,lastA1, last_ts);
    //#endif
    */
    
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
            double contrib41Left = std::abs(*pMap41Left);
            double contrib42Left = std::abs(*pMap42Left);
            double contrib43Left = std::abs(*pMap43Left);
            double contribA1Left = std::abs(*pMapA1Left);
            double contribA2Left = std::abs(*pMapA2Left);
            
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
            
            // weights of the feature maps -- they should be in a config file
            double wa1 = 0.0;
            double wa2 = 0.0;
            double w41 = 1.0;
            double w42 = 0.0;
            double w43 = 0.0;

            // the saliency map is updated as the weighted sum of the absolute values of the feature maps
            *pBufferLeft =  wa1 * contribA1Left + wa2 * contribA2Left + wa2 * contribA2Left +
            w41 * contrib41Left + w42 * contrib42Left + w43 * contrib43Left;

            if(*pBufferLeft > 1.0) {
                *pBufferLeft = 1.0;
            }
            

            //double right_double = 0;
            double left_double  = 0;
            
            // DEPREC -------------- max and min of left and right image  -------------------
            left_double  = *pBufferLeft;
            
            //--------------- temporal information ------------------------------------
            unsigned long ta1l = *pTimeA1Left;
            unsigned long ta2l = *pTimeA2Left;
            unsigned long t41l = *pTime41Left;
            unsigned long t42l = *pTime42Left;
            unsigned long t43l = *pTime43Left;
            
            // takes the maximum time stamp
            timestampactual = 0;
            
            if(t41l > timestampactual){
                timestampactual = t41l;
            }
            else if(t42l > timestampactual){
                timestampactual = t42l;
            }
            else if(t43l > timestampactual){
                timestampactual = t43l;
            }
            else if( ta1l > timestampactual){
                timestampactual = ta1l;
            }
            else if( ta2l > timestampactual){
                timestampactual = ta2l;
            }
            // takes the timestamp from feat map 41
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
                
                if (((timestampactual * COUNTERRATIO) > minCount) && ((timestampactual * COUNTERRATIO) < maxCount))
                {   //(timestampactual != lasttimestamp)
                    
                    // update grey scale value of the saliency map (from [0 1.0] to [0 255])
                    *pImage = (unsigned char) (value);
                    pImage++;
                    *pImage = (unsigned char) (value);
                    pImage++;
                    *pImage = (unsigned char) (value);
                    pImage++;
                    
                    if(value > maxValue){ // looking for the max in the saliency map, then storing the value, the timestamp and the coordinates
                        maxValue = value;
                        maxLeftR = r;
                        maxLeftC = c;
                        maxtimestamp = timestampactual;
                    }
                }
                else { // if the timestamp is not in the valid range, then the Saliency map has minimum saliency in the location
                    *pImage = (unsigned char) 0 ;
                    pImage++;
                    *pImage = (unsigned char) 0 ;
                    pImage++;
                    *pImage = (unsigned char) 0 ;
                    pImage++;
                }
               
            }
            else {
                
                // ------------------------- branch !tristateView ------------------------
                if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {
                    *pImage =  value ;
                    pImage++;
                    //TODO:  the output image is RGB image, still address other color pixels
                    
                }
                else {
                    *pImage = (unsigned char) 10 ;
                    pImage++;
                    //TODO:  the output image is RGB image, still address other color pixels
                }
                
            } // end !tristate
            
            pBufferLeft++;
            pMap41Left++;
            pMap42Left++;
            pMap43Left++;
            pMapA1Left++;
            pMapA2Left++;
            //---
            pTime41Left++;
            pTime42Left++;
            pTime43Left++;
            pTimeA1Left++;
            pTimeA2Left++;
        } // end inner loop
        pImage += imagePadding;
    }//end outer loop
    
    // cycle after normalisation - WTA representation
    //printf("cycle after the normalisation LEFT:(%f, %f)  RIGHT:(%f,%f) \n", minLeft, maxLeft, minRight, maxRight);
    double* pSalLeft        = saliencyMapLeft;
    //double* pSalRight       = saliencyMapRight;
    //double rangeLeft        = std::abs(maxLeft  - minLeft);
    //double rangeRight       = std::abs(maxRight - minRight);
    unsigned char* pLeft    = imageLeft    ->getRawImage();
    unsigned char* pRight   = imageRight   ->getRawImage();
    unsigned char* pLeftBW  = imageLeftBW  ->getRawImage();
    //unsigned char* pRightBW = imageRightBW ->getRawImage();
    int padding             = imageLeft    ->getPadding();
    int rowSize             = imageLeft    ->getRowSize();
    int rowSizeBW           = imageLeftBW  ->getRowSize();
    
    imageLeftBW->zero();
    imageRightBW->zero();
    
    if(plotLatency) {
        //timeStop = Time::now();
        double latency = timeStop - timeStart;
        fprintf(latencyFile, "%f \n", latency);
        
        //timeStart = Time::now();
    }
    
    if(maxValue >= FIRETHRESHOLD) {
        
        // sending command for saccade; to focus redeployment corresponds fixation point reallocation
        if(outputCmdPort.getOutputCount()){
            
            iCount++;
            
            // the countCommands prevents from sending frequent sequences of commands
            if(countCommands >= COMMCOUNTDEF)  {
                yarp::os::Bottle& commandBottle=outputCmdPort.prepare();
                commandBottle.clear();
                commandBottle.addString("left");
                commandBottle.addInt(maxLeftC);
                commandBottle.addInt(maxLeftR);
                commandBottle.addDouble(0.5);
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
                pSalLeft++;
                pTimeLeft++;
            } //end inner for
            
            pLeft  += padding;
            pRight += padding;
            
        }//end outer for
    } // end if
}

void vSaliencyMapManager::run() {
    
    /*prepare output vBottle with AEs */
    emorph::vBottle &outBottle = smEventsPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    // ????? this works when the thread is a buffered port, what is the method for threads?
    //this->getEnvelope(st);
    //outBottle.setEnvelope(st);
    
    count++;
        //interTimer = Time::now();
    //double interval2 = (interTimer - startTimer) * 1000000;
    double interval = (interTimer - startTimer) * 1000000;
    
        //======================== temporal synchronization pre-unmasking  =================================
        //printf("after extracting the bottle \n");
        // saving the buffer into the file
    
        // getting the time statistics
        //endTimer = Time::now();
        //double interval  = (endTimer - startTimer) * 1000000; //interval in us

#ifdef STOREINI
        fprintf(istore,"\n %f ", interval);
#endif
        //startTimer = Time::now();
        
        //check for wrapping of the left and right timestamp
    /* -- chiara -- to compile --
        if(maxCount >= 4294967268  ) {
            //verb = true;
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
            //verb = true;
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
      -- end chiara -- to compile -- */
        
        //==================== temporal synchronization post unmasking =======================
        
        //synchronising the threads at the connection time
        unsigned long int lastleft, lastright;
        if (!synchronised) {
            std::cout << "Sychronising "<< std::endl;
            
            lastleft = lastright = *lasttimestamp;
            lc = lastleft  * COUNTERRATIO;
            rc = lastright * COUNTERRATIO;
            
            //TODO : Check for negative values of minCount not allowed!!!!!!
            
            minCount = lc - interval * INTERVFACTOR* dim_window;
            minCountRight = rc - interval * INTERVFACTOR* dim_window;
            
            //startTimer = Time::now();
            synchronised = true;
            count = synch_time - 200;
        }
        else if ((count % synch_time == 0) && (minCount < 4294500000)) {
            lastleft = lastright = *lasttimestamp;
            lc = lastleft  * COUNTERRATIO;
            rc = lastright * COUNTERRATIO;
            
            if( lc > interval* INTERVFACTOR * dim_window)
                minCount      = lc - interval* INTERVFACTOR * dim_window; //cfConverter->getEldestTimeStamp();
            else
                minCount = 0;
            
            if( rc > interval* INTERVFACTOR * dim_window)
                minCountRight = rc - interval* INTERVFACTOR * dim_window;
            else
                minCountRight = 0;
            
            //startTimer = Time::now();
            synchronised = true;
        }
        else {
            // this value is simply the ration between the timestamp reported by the aexGrabber (6.25Mhz)
            //and the correct timestamp counter clock of FPGA (50 Mhz)
            microsecondsPrev = interval;
            //interval = Tnow;
            minCount      = minCount + interval * INTERVFACTOR; // * (50.0 MHz FPGA Counter Clock / 6.25 Mhz ;
            minCountRight = minCount + interval * INTERVFACTOR;  // minCountRight + interval;
        }
        //printf("minCount %d interval %f \n", minCount, interval);
        maxCount      =  minCount      + interval * INTERVFACTOR* (dim_window);
        maxCountRight =  minCountRight + interval * INTERVFACTOR* (dim_window);
        
        
        //---- preventer for fixed  addresses ----//
        if(count % 100 == 0) {
                lastleft = lastright = *lasttimestamp;
            
            lc = lastleft * COUNTERRATIO;
            
            rc = lastright * COUNTERRATIO;
            
                if ((lcprev == lc)||(rcprev == rc)) {
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
            
            lcprev = lc;
            rcprev = rc;
        }
        
        
        //resetting time stamps at overflow
        if (countStop == 10) {
            //printf("resetting time stamps!!!!!!!!!!!!! %d %d   \n ", minCount, minCountRight);
            /*
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
            
            countStop = 0;
            //printf("countStop resetting %llu %llu %llu \n",unmask_events->getLastTimestamp(), lc, rc );
            count = synch_time - 200;
            */
        }
        
        
        // the getMonoImage gets as default input image the saliency map
        if(imageLeft != 0) {
            
            getMonoImage(imageLeft,minCount,maxCount,1);
            if(imageLeftBW != 0) {
                //pThread->copyLeftBW(imageLeftBW);
            }
            //printf("copying the right \n");
            //pThread->copyLeft(imageLeft);
        }
    
            if(imageRight != 0) {
                //printf("getting the right image \n");
                getMonoImage(imageRight,minCountRight,maxCountRight,0);
                //printf("copying the right image \n");
              //  pThread->copyRight(imageRight);
            }
            if(imageRightBW != 0) {
                //pThread->copyRightBW(imageRightBW);
            }
    
        
        // debug check point! Checking the time interval required for processing
        //endTimer = Time::now();
        //double intervalProc  = (endTimer - interTimer) * 1000000; //interval in us
        //printf("processing time %f \n", intervalProc);
    
    // --- writing vBottle and commands on buffered output ports --- //
    if (strictness){
        smEventsPort.writeStrict();
        outputCmdPort.writeStrict();
    }
    else{
        smEventsPort.write();
        outputCmdPort.write();
    }

}
/**********************************************************/
void vSaliencyMapManager::threadRelease() {
    
    fclose(fout);
    fclose(fstore);
    fclose(istore);
    fclose(latencyFile);
    printf("vSaliencyMapManager thread release:freeing bufferCopy \n");
    
    delete imageLeft;
    delete imageRight;
    delete imageLeftBW;
    delete imageRightBW;
    
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
    //free(unmaskedEvents);
    
    std::cout << "vSaliencyMapManager thread release: closing ports" << std::endl;
    
    smEventsPort.close();
    outputCmdPort.close();
    
    std::cout << "vSaliencyMapManager thread release: stopping Threads" << std::endl;
    //pThread->stop();
    if(bptA1!=0)
        bptA1->close();
    if(bptA2!=0)
        bptA2->close();
    if(bpt41!=0)
        bpt41->close();
    if(bpt42!=0)
        bpt42->close();
    if(bpt43!=0)
        bpt43->close();
    
}


void vSaliencyMapManager::resizeAll(int widthp, int heightp) {
    imageLeft = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    imageLeft->resize(retinalSize,retinalSize);
    imageRight = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    imageRight->resize(retinalSize,retinalSize);
    imageLeftBW = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    imageLeftBW->resize(retinalSize,retinalSize);
    imageRightBW = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    imageRightBW->resize(retinalSize,retinalSize);
}


/**********************************************************/


//empty line to make gcc happy
