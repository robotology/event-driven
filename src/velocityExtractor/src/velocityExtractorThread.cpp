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
#define MAXVALUE 0xFFFFFF //4294967295

#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
#define CHUNKSIZE 32768 //65536 //8192

//#define retinalSize 128


//#define VERBOSE

velocityExtractorThread::velocityExtractorThread() : Thread() {
    width = 128;
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

    count = 0;
    microsecondsPrev = 0;
    minCount = 0;
    minCountRight= 0;

    vbh = new velocityBottleHandler();
    vbh->useCallback();
    vbh->open(getName("/velocity:i").c_str());

    if(!outBottlePort.open(getName("/retinaBottle:o").c_str())) {
        printf("error in opening the output port \n");
        return false;
    }
    if(!outImagePort.open(getName("/histo:o").c_str())) {
        printf("error in opening the output port \n");
        return false;
    }
    
    receivedBottle = new Bottle();
    bottleToSend   = new Bottle();

    for(int i = 0 ; i < numberOfAngles; i++) {
        histogram[i] = 0;
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
    imageOut->resize(360,20);
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

void velocityExtractorThread::run() {
    while(!isStopping()) {

        //Bottle* readBottle = inBottlePort.read(true);
        VelocityBuffer* vb; // = new VelocityBuffer();
            
        vb = vbh->extractBottle(vb);
        
        if(vb != 0) {
            printf("extracting not null bottle %08x \n", vb);
            for (int i = 0 ; i < vb->getSize(); i++) {
                
                int u          = vb->getX(i);
                int v          = vb->getY(i);
                //printf("%d %d \n", u, v);
               
                if((u > umin) && (u < umax) && (v > vmin) && (v < vmax)) {
                    printf("read u = %d v = %d \n", u, v);
                    double uDot    = vb->getVx(i);
                    double vDot    = vb->getVy(i);
                    double theta   = atan2(vDot,uDot);
                    double mag     = sqrt(vDot * vDot + uDot * uDot);

                    int thetaDeg   = floor((theta / 3.1415) * 180);
                    if(thetaDeg < 0)
                        thetaDeg = 360 + thetaDeg;

                    int pos        =  thetaDeg / (360 / numberOfAngles);
                    
                    // printf("uDot %f vDot %f theta %f thetaDeg %d pos %d \n",uDot,vDot, theta,thetaDeg, pos);
                    histogram[pos] = histogram[pos] + 1;
                    if(histogram[pos] > maxFiringRate) {
                        //printf("max reached in pos %d \n", pos);
                        histogram[pos] = 0;
                    }
                }   
            } //end of the for                        
        } // end vb! = 0
    
        Time::delay(0.1);
        
    } //end of while
}

void velocityExtractorThread::onStop() {
    printf("in the onStop function \n");
    outBottlePort.interrupt();
    
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
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

