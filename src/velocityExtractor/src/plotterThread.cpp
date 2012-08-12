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

#define THRATE 15

plotterThread::plotterThread() : RateThread(THRATE) {
    count       = 0;
    retinalSize = 128;   //default value retinal size before setting    

    firstInputImage = true;
    synchronised    = false;
   
    for(int i = 0; i < NUMANGLES; i++) {
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
    imageHisto->resize(NUMANGLES,retinalSize);
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


void plotterThread ::setHistoValue(int* hValuePointer){
    mutexHisto.wait();    
    
    for(int i = 0; i < NUMANGLES; i++) {
        //printf("%d %d \n",i, hValuePointer[i]);
        histoValue[i] = hValuePointer[i];
    }
    
    mutexHisto.post();
}

void plotterThread::setVelResult(int angle, float magnitude, bool _maxReached) {
    //printf("setting valResult %d \n", angle);
    mutexVeloc.wait();
    velWTA_direction = angle;
    velWTA_magnitude = magnitude;
    maxReached = _maxReached;
    mutexVeloc.post();
}

void plotterThread::prepareHistoImage(ImageOf<PixelRgb>& out) {
    for (int i = 0; i < NUMANGLES; i++) {        
        unsigned char* tmp = out.getPixelAddress(i,200 - 1 - histoValue[i]);
        *tmp = (unsigned char) 255; tmp++;
        *tmp = (unsigned char) 255; tmp++;
        *tmp = (unsigned char) 255; tmp++;        
    }

    cvLine(out.getIplImage(), cvPoint(0, 100), 
           cvPoint(NUMANGLES,100), 
           cvScalar(255,0,0), 2, 8, 0); //line thick, line type, shift 

}

void plotterThread::prepareVelocImage(ImageOf<PixelMono> in, ImageOf<PixelRgb>& out){
    unsigned char* pin  =  in.getRawImage();
    unsigned char* pout = out.getRawImage();
    int padding3 = out.getPadding();
    int padding  = in.getPadding();
    int rowSize3 = out.getRowSize();
    int rowSize  = in.getRowSize();
    int halfRetinalSize  = retinalSize >> 1;
    int fourthRetinalSize = halfRetinalSize >> 1;
    

    pin  += fourthRetinalSize * rowSize  + fourthRetinalSize;
    pout += fourthRetinalSize * rowSize3 + fourthRetinalSize * 3;
    
    for (int r = halfRetinalSize - fourthRetinalSize ; r < halfRetinalSize + fourthRetinalSize; r++) {
        for(int c =  halfRetinalSize - fourthRetinalSize; c <  halfRetinalSize + fourthRetinalSize; c++) {
            
            for (int k = 0; k < 3; k++) {
                *pout = *pin;
                pout++;
            }
            
            pin++;
        }
        pout += halfRetinalSize * 3;
        pin  += halfRetinalSize;
    } 
 
    double velWTA_rad = (velWTA_direction / 180.0) * PI;
    
    //printf("velWTA: %f %d \n", velWTA_rad, velWTA_direction );
    int uComp = (int) round(cos(velWTA_rad) * (velWTA_magnitude * 1000));
    int vComp = (int) round(sin(velWTA_rad) * (velWTA_magnitude * 1000));
    //printf("uComp %d vComp %d \n",uComp, vComp);
    
    CvScalar arrowColor;
    if(maxReached) {
        maxReached  = false;
        arrowColor = CV_RGB( 0,255,0);
    }
    else {
        arrowColor = CV_RGB(255,0,0);
    }


    cvLine(out.getIplImage(), cvPoint(halfRetinalSize, halfRetinalSize), 
           cvPoint((halfRetinalSize) + uComp,(halfRetinalSize) + vComp), 
           arrowColor, 2, 8, 0); //line thick, line type, shift

    cvCircle(out.getIplImage(), cvPoint((halfRetinalSize) + uComp,(halfRetinalSize) + vComp), 5, arrowColor, -1, 8, 0);

    string angleStr;
    sprintf((char *)angleStr.c_str(), "%d", velWTA_direction );
    
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
    cvPutText(out.getIplImage(), angleStr.c_str(), cvPoint(halfRetinalSize, halfRetinalSize - 20), &font, arrowColor);
    
    /*
    cvRectangle(out.getIplImage(), cvPoint(halfRetinalSize - fourthRetinalSize,halfRetinalSize - fourthRetinalSize ),
                cvPoint(halfRetinalSize + fourthRetinalSize, halfRetinalSize + fourthRetinalSize),
                cvScalar(0, 255,0), 1, 8, 0);
    */
    
    
    
    
}

void plotterThread::run() {
    
    count++;
    synchronised = true;
    ImageOf<PixelRgb>& imageHisto    = histoPort.prepare();
    imageHisto.resize(NUMANGLES, 200);
    imageHisto.zero();
    
    if(velocPortIn.getInputCount()) {
        imageVelocIn = velocPortIn.read(false);
        if ( (imageVelocIn != NULL) && (firstInputImage)) {
            retinalSize = imageVelocIn->width();
            firstInputImage = false;
        }
        
        if(!firstInputImage) {
            ImageOf<PixelRgb>& imageVelocOut = velocPortOut.prepare();
            imageVelocOut.resize(retinalSize, retinalSize);
            imageVelocOut.zero();
            
            if(velocPortOut.getOutputCount() && (imageVelocIn != NULL)) {        
                prepareVelocImage(*imageVelocIn, imageVelocOut);
                velocPortOut.write();
            }
        }
        
    }
    
    //printf("plotter::run %d  \n", histoPort.getOutputCount());
    if(histoPort.getOutputCount()) {
        prepareHistoImage(imageHisto);        
        histoPort.write();
        
    }
    
}

void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    histoPort.close();
    velocPortOut.close();
    velocPortIn.close();
    printf("freed images \n");
   

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
