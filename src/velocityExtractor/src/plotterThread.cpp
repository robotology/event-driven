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
 * @file plotterThread.cpp
 * @brief Implementation of the thread that represent the frame conversion of events (see header plotterThread.h)
 */

#include <iCub/plotterThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 25

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    retinalSize = 128;   //default value retinal size before setting    

    for(int i = 0; i < 360; i++) {
        histoValue[i] = 10;
    }
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n plotterThread::threadInit:starting the thread.... \n");
    // opening ports 
    histoPort.open      (getName("/histo:o").c_str());
    velocPortOut.open   (getName("/vel:o").c_str());
    velocPortIn.open    (getName("/vel:i").c_str());
   

    // initialising images
    imageHisto     = new ImageOf<PixelRgb>;
    imageHisto->resize(retinalSize,retinalSize);
    imageVelocOut     = new ImageOf<PixelRgb>;
    imageVelocOut->resize(retinalSize,retinalSize);
   
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    histoPort.interrupt();
    velocPortIn.interrupt();
    velocPortOut.interrupt();
}

void plotterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string plotterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void plotterThread::resize(int widthp, int heightp) {
}

void plotterThread::copyLeft(ImageOf<PixelMono>* image) {
    /*
    //printf("retinalSize in plotterThread %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0; c < retinalSize; c++) {                
                *pleft++ = *pimage++;
            }
            pleft  += padding;
            pimage += padding;
        }
    }
    */
}

void plotterThread::copyLeft(ImageOf<PixelRgb>* image) {
    /*
    //printf("retinalSize in plotterThread %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0; c < retinalSize; c++) {                
                *pleft++ = *pimage++;
                *pleft++ = *pimage++;
                *pleft++ = *pimage++;
            }
            pleft  += padding;
            pimage += padding;
        }
    }
    */
}

void plotterThread::copyRight(ImageOf<PixelMono>* image) {
    /*
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pright = imageRight->getRawImage();
    if(imageRight != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {
                *pright++ = *pimage++;
            }
            pright += padding;
            pimage += padding;
        }
    } 
    */
}

void plotterThread::copyRight(ImageOf<PixelRgb>* image) {
    /*
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pright = imageRight->getRawImage();
    if(imageRight != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {
                *pright++ = *pimage++;
                *pright++ = *pimage++;
                *pright++ = *pimage++;
            }
            pright += padding;
            pimage += padding;
        }
    } 
    */
}

void plotterThread::prepareHistoImage(ImageOf<PixelRgb>& out) {
    for (int i = 0; i < 360; i++) {
        unsigned char* tmp = out.getPixelAddress(i,histoValue[i]);
        *tmp = (unsigned char) 255; tmp++;
        *tmp = (unsigned char) 255; tmp++;
        *tmp = (unsigned char) 255; tmp++;
    }

}

void plotterThread::prepareVelocImage(ImageOf<PixelMono> in, ImageOf<PixelRgb>& out){
    unsigned char* pin =  in.getRawImage();
    unsigned char* pout = out.getRawImage();
    int padding3 = out.getPadding();
    int padding  = in.getPadding();

    for (int r = 0; r < 128; r++) {
        for(int c = 0; c < 128; c++) {
            for (int k = 0; k < 3; k++) {
                *pout = *pin;
                pout++;
            }
            pin++;
        }
        pout += padding3;
        pin  += padding;
    }    
}

void plotterThread::run() {
    
    count++;
    ImageOf<PixelRgb>& imageHisto    = histoPort.prepare();
    imageHisto.resize(retinalSize, retinalSize);
    imageHisto.zero();
    ImageOf<PixelRgb>& imageVelocOut = velocPortOut.prepare();
    imageVelocOut.resize(retinalSize, retinalSize);
    synchronised = true;

    if(velocPortIn.getInputCount()) {
        imageVelocIn = velocPortIn.read(false);
    }

    //printf("plotter::run %d  \n", histoPort.getOutputCount());
    if(histoPort.getOutputCount()) {
        prepareHistoImage(imageHisto);        
        histoPort.write();
        
    }
    if(velocPortOut.getOutputCount()) {        
        prepareVelocImage(*imageVelocIn, imageVelocOut);
        velocPortOut.write();
        
    }
}

void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    histoPort.close();
    velocPortOut.close();
    velocPortIn.close();
    
    printf("freeing memory \n");

    printf("freed images \n");
    delete imageVelocIn;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
