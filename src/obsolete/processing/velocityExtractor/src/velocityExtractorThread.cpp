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
 * @file velocityExtractorThread.cpp
 * @brief Implementation of the thread (see header velocityExtractorThread.h)
 */

#include <iCub/velocityExtractorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define dim_window   5
#define synch_time   1
#define INTERVFACTOR 1              // resolution 160ns = 6.25Mhz, 128ns = 7.8125Mhz ... gaep 1us
#define COUNTERRATIO 1              // 1.25 is the ratio 0.160/0.128
#define THRATE       5
#define DECRFACTOR   30
#define INCRFACTOR   3
#define MAXVALUE 0xFFFFFF //4294967295

#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
#define CHUNKSIZE 32768 //65536 //8192

//#define retinalSize 128


//#define VERBOSE

velocityExtractorThread::velocityExtractorThread() : Thread() {
    width            = 128;
    responseGradient = 127;
    retinalSize      = 128;  //default value before setting 
    velWTA_direction = 0;
    count            = 0;
    minCount         = 0; //initialisation of the timestamp limits of the first frame
    lc               = 0;	
    rc               = 0;
    minCount         = 0;
    minCountRight    = 0;
    countStop        = 0;
    egoMotionU       = 0;
    egoMotionV       = 0;
    alfa             = 0.0;
  
    synchronised = false;
    greaterHalf  = false;
    firstRun     = true;
    firstCycle   = true;
    
    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
    
    verb = false;
    string i_fileName("events.log");
    raw = fopen(i_fileName.c_str(), "wb");

    
    
}

velocityExtractorThread::~velocityExtractorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool velocityExtractorThread::threadInit() {
    printf(" \n------------------------- velocityExtractorThread::threadInit:starting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    fout = fopen("./dump.txt","w+");
    resize(retinalSize, retinalSize);

    printf("starting the converter!!!.... \n");
       
    printf("\n opening retina\n");
    printf("starting the plotter \n");

    startTimer = Time::now();

    count            = 0;
    countMedian      = 0;
    microsecondsPrev = 0;
    minCount         = 0;
    minCountRight    = 0;
    countSent        = 0;

    vbh = 0;
    
    vbh = new velocityBottleHandler();
    vbh->useCallback();
    vbh->open(getName("/velocity:i").c_str());
    

    pt = 0;
    
    pt = new plotterThread();
    pt->setName(getName("/plotter").c_str());
    pt->setMaxFiringRate(maxFiringRate);
    pt->start();
    
    
    
    if(!outBottlePort.open(getName("/cmd:o").c_str())) {
        printf("error in opening the output port \n");
        return false;
    }
    //if(!outImagePort.open(getName("/histo:o").c_str())) {
    //    printf("error in opening the output port \n");
    //    return false;
    // }
    
    receivedBottle = new Bottle();
    bottleToSend   = new Bottle();

    for(int i = 0 ; i < numberOfAngles; i++) {
        histogram[i] = 0;
        magnitude[i] = 0;
        counter  [i] = 0;
    }
    
    for(int i=0 ; i < medianDim; i++) {
        medianVector[i] = 0.0;
    }
    
   
    printf("-----------------------------  velocityExtractorThread::threadInit:Initialisation of collector thread correctly ended \n");
    return true;
}

void velocityExtractorThread::interrupt() {
    printf("velocityExtractorThread::interrupt:interrupt for ports \n");
    outBottlePort.interrupt();
    outImagePort.interrupt();
    outPortRight.interrupt();
    printf("velocityExtractorThread::interrupt: port interrupt! \n");
}

void velocityExtractorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string velocityExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void velocityExtractorThread::resize(int widthp, int heightp) {
    imageOut = new ImageOf<PixelMono>;
    imageOut->resize(NUMANGLES,20);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(widthp,heightp);
}

/*
void velocityExtractorThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image){
    for (int i = 0; i < 360; i++) {
        
    }
}
*/

int velocityExtractorThread::prepareUnmasking(char* bufferCopy, Bottle* res) {
    // navigate the 32bit words in the bufferCopy and create a bottle outofvalid
    int numberofwords = CHUNKSIZE / 4; //4bytes made a 32bits word
    u32* pointerWord = (u32*) bufferCopy;
    int countValid  = 0;
    for (int i = 0; i< numberofwords; i ++) {
        
        if(*pointerWord != 0) {
            //this is a valid word
            Bottle tmp;
            tmp.addInt(*pointerWord);
            if(i == 100) {
                printf("%08X \n", *pointerWord);
            }
            res->append(tmp);
            countValid++;
            pointerWord++;
        }
    }
    return countValid;
}


void velocityExtractorThread::setHistoValue() {
    //int histoValue[360];
    //int* pHisto = &histoValue[0];

    int* pHisto = new int[NUMANGLES];
    double sumHist = 0;

    // finding the mean value
    for (int i = 0; i < NUMANGLES; i++) {
        double posDouble = i / (NUMANGLES / numberOfAngles);
        int pos = floor(posDouble);       
        
        //histoValue[i] = histogram[pos];
        pHisto[i] =  histogram[pos];        
        sumHist += histogram[pos];
    }

    // removing background noise
    meanHist = sumHist / NUMANGLES ;
    /*
    for (int i = 0; i < NUMANGLES; i++) {
        pHisto[i] -= meanValue;  
        if(pHisto[i] < 0) {
            pHisto[i] = 0;
        }
    }
    */
    
    pt->setHistoValue(pHisto);
}


void velocityExtractorThread::run() {

    while(!isStopping()) {
        
    
    if(!suspendFlag) {
        if(firstCycle) {
            firstCycle = false;
            Time::delay(3.5);
            printf("getting out of the first cycle \n");
        }
        //printf("inNotSuspend \n");
        //Bottle* readBottle = inBottlePort.read(true);
        VelocityBuffer* vb; // = new VelocityBuffer();
        sumHist = 0;    
        vb = vbh->extractBottle(vb);
        //printf("just extracted \n");
        if(vb != 0) {
            //printf("extracting not null bottle %08x \n", vb);
            //Vector medianVector(5);
            for (int i = 0 ; i < vb->getSize(); i++) {
                
                int u          = vb->getX(i);
                int v          = vb->getY(i);
                
                //printf("%d %d \n", u, v);
               
                if((u > umin) && (u < umax) && (v > vmin) && (v < vmax)) {
                    //printf("read u = %d v = %d \n", u, v);
                    double uDot    = vb->getVx(i);
                    double vDot    = vb->getVy(i);
                    //correcting the acquired retinal speed with egoMotion
                    //uDot = uDot - egoMotionU;
                    //vDot = vDot - egoMotionV;
                    //printf("egoMotion u %f v %f \n", egoMotionU, egoMotionV);
                    if((egoMotionU != 0) || (egoMotionV!=0)) {
                        uDot = 0; //uDot - 0.5;
                        vDot = 0; //vDot - 0.5;
                    }else {
                        //printf("uDot %f vDot %f \n", uDot, vDot);
                    }
                    
                    double theta   = atan2(vDot,uDot);
                    double mag     = sqrt(vDot * vDot + uDot * uDot);
                    //printf("uDot %f vDot %f \n", uDot,vDot);
                    int thetaDeg   = floor((theta / PI) * 180);
                    //printf("theta %f thetaDeg %d \n", theta, thetaDeg);
                    if(thetaDeg < 0){
                        thetaDeg = 360 + thetaDeg;
                    }
                    int pos        =  thetaDeg / (360 / numberOfAngles);
                    //printf("thetaDeg %d pos %d \n", thetaDeg, pos);
                    
                    //printf("uDot %f vDot %f theta %f thetaDeg %d pos %d \n",uDot,vDot, theta,thetaDeg, pos);
                    if((egoMotionU != 0) || (egoMotionV!=0)) {
                        // the head is moving
                        // either compensate or avoid computation
                    }
                    else {

                        histogram[pos] =  mag * 500 + (histogram[pos] ) - meanHist;
                        if(histogram[pos] < 0) {
                            histogram[pos] = 0;
                        } 
                        counter  [pos] = counter[pos]   + 1;
                        magnitude[pos] = magnitude[pos] + mag;
 
                        if(histogram[pos] > maxFiringRate) {

                            maxReached = true;                     
                            velWTA_direction = pos * (360 / numberOfAngles);
                            velWTA_magnitude = magnitude[pos] / counter[pos];

                            if(countMedian == 0) {
                                medianMag   [0] = velWTA_magnitude;
                                medianVector[0] = velWTA_direction;                                                               
                            }
                            else {
                                for(int i = 0; i < 5; i++) {
                                    
                                    if(i == countMedian) {
                                        medianVector[i] = velWTA_direction;
                                        medianMag   [i] = velWTA_magnitude;
                                        break;
                                    }
                                    else if(velWTA_direction < medianVector[i]){

                                        //printf("vetWtadirction %d <= medianVector(i) %f \n",velWTA_direction,medianVector[i] );
                                        //printf("medianVector %f %f %f %f %f \n",medianVector[0],medianVector[1],medianVector[2],medianVector[3],medianVector[4]);

                                        //shift
                                        int jump =  countMedian - i;
                                        
                                        for (int k = jump; k > 0; k--) {
                                            medianVector[i + k] = medianVector[i + k - 1];
                                            medianMag   [i + k] = medianMag   [i + k - 1];
                                        }
                                        medianVector[i] = velWTA_direction;
                                        medianMag   [i] = velWTA_magnitude;
                                        break;
                                    }
                                }
                            }
                            countMedian++;
                            //printf("increment of the value of countMedian \n");

                            if(countMedian == 5) {
                                velWTA_direction = medianVector[2];
                                velWTA_magnitude = medianMag   [2];
                                printf("--------------------------------\n");
                                printf("Reached max value countMedian %f \n",medianVector[2] );
                                printf("medianVector %f %f %f %f %f \n",medianVector[0],medianVector[1],medianVector[2],medianVector[3],medianVector[4]);

                                 //sending command to the oculomotor performer
                                if(outBottlePort.getOutputCount()) {
                                    printf("after reached the count of 5 for countMedian \n");
                                    printf("sending the command number %d\n \n\n", countSent);
                                    //after 5 steps, in the middle of the ordered mediaVector there is the median valie
                                    double velWTA_rad = medianVector[2] * (PI / 180);                            
                                    //double velWTA_rad = velWTA_direction * (PI / 180);                            
                                    double u = cos(velWTA_rad) * velWTA_magnitude; 
                                    double v = sin(velWTA_rad) * velWTA_magnitude;
                                    double pixelU = u * 100;
                                    double pixelV = v * 100;
                                    
                                    Bottle& b = outBottlePort.prepare();
                                    b.clear();
                                    b.addString("SM_PUR");
                                    b.addDouble(pixelU);          // velocity along u axis
                                    b.addDouble(pixelV);          // velocity along v axis
                                    b.addDouble(0.5);          // smooth pursuit time extension
                                    b.addDouble(velWTA_direction);
                                    b.addInt(countSent);
                                    outBottlePort.write();
                                    
                                    countSent++;
                                    
                                    suspendFlag = true;
                                    
                                }                        
                                
                                countMedian = 0;
                                for(int i = 0; i < 5; i++) {
                                    medianVector[i] = 0;
                                }
                            }        
                                                    
                            // resetting the hist/mag
                            //printf("resetting the histogram \n");
                            for(int k = 0; k < numberOfAngles; k++ ) {
                                histogram[k] = 0;
                                magnitude[k] = 0;                        
                                counter  [k] = 0;
                            }// end for k
                            
                            
                            
                        }
                    }//end if !egoMotion
                }   
            } //end of the for                        
        } // end vb! = 0

        
        //printf("setting histo value \n");
        setHistoValue();
        pt->setVelResult(velWTA_direction, velWTA_magnitude, maxReached, meanHist);
        
        // resetting the maxReached in this class
        if(maxReached) {
            maxReached = false;
        }

        decayingProcess();
    
        Time::delay(0.05);
    } //end if !suspend    
        


    } //end of while
       
}



void velocityExtractorThread::decayingProcess() {
    for(int i = 0 ; i < numberOfAngles; i++) {
        if(histogram[i] > DECRFACTOR)
            histogram[i] = histogram[i] - DECRFACTOR;
    }

}

void velocityExtractorThread::onStop() {
    printf("in the onStop function \n");
    outBottlePort.interrupt();
    outImagePort.interrupt();
    outPortRight.interrupt();
    
    outImagePort.close();
    outPortRight.close();
    outBottlePort.close();
    
}

void velocityExtractorThread::threadRelease() {
    printf("velocityExtractorThrea::threadRelease:entering  \n");
    idle = false;
    fclose(fout);

    printf("velocityExtractorThread release:closing ports \n");
    outBottlePort.close();
    outImagePort.close();
    outPortRight.close();

    delete receivedBottle;
    delete bottleToSend;
    printf("correctly freed memory from the bottleHandler \n");
    if (pt!=0) {
        pt->stop();
    }
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

