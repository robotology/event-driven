// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * Public License fo
 r more details
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

#define THRATE 30
//#define retinalSize 128

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    retinalSize = 128;   //default value retinal size before setting    
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    // opening ports 
    leftPort.open(getName("/left:o").c_str());
    rightPort.open(getName("/right:o").c_str());
    leftIntPort.open(getName("/leftInt:o").c_str());
    rightIntPort.open(getName("/rightInt:o").c_str());
    leftGrayPort.open(getName("/leftGrey:o").c_str());
    rightGrayPort.open(getName("/rightGrey:o").c_str());
    eventPort.open(getName("/event:o").c_str());

    // initialising images
    imageLeft      = new ImageOf<PixelMono>;
    imageLeft->resize(retinalSize,retinalSize);
    imageRight     = new ImageOf<PixelMono>;
    imageRight->resize(retinalSize,retinalSize);
    imageLeftInt   = new ImageOf<PixelMono>;
    imageLeftInt->resize(retinalSize,retinalSize);
    imageLeftInt->zero();
    imageRightInt  = new ImageOf<PixelMono>;
    imageRightInt->resize(retinalSize,retinalSize);
    imageRightInt->zero();
    imageLeftBW    = new ImageOf<PixelMono>;
    imageLeftBW->resize(retinalSize,retinalSize);
    imageLeftBW->zero();
    imageRightBW   = new ImageOf<PixelMono>;
    imageRightBW->resize(retinalSize,retinalSize);
    imageRightBW->zero();
    imageLeftGrey  = new ImageOf<PixelMono>;
    imageLeftGrey->resize(retinalSize,retinalSize);
    imageLeftGrey->zero();
    imageRightGrey = new ImageOf<PixelMono>;
    imageRightGrey->resize(retinalSize,retinalSize);
    imageRightGrey->zero();
    imageLeftThreshold  = new ImageOf<PixelMono>;
    imageLeftThreshold->resize(retinalSize,retinalSize);
    imageLeftThreshold->zero();
    imageRightThreshold = new ImageOf<PixelMono>;
    imageRightThreshold->resize(retinalSize,retinalSize);
    imageRightThreshold->zero();
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    leftPort.interrupt();
    leftIntPort.interrupt();
    rightPort.interrupt();
    rightIntPort.interrupt();
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
}

void plotterThread::copyRight(ImageOf<PixelMono>* image) {
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
}


void plotterThread::run() {
    count++;
    imageLeft  = &leftPort.prepare();
    imageLeft->resize(retinalSize, retinalSize);
    imageRight = &rightPort.prepare();
    imageRight->resize(retinalSize, retinalSize);
    synchronised = true;

    
    if(leftPort.getOutputCount()) {
        leftPort.write();
    }
    if(rightPort.getOutputCount()) {
        rightPort.write();
    }

    /*
    // obtaining the left and right integrated images
    int positionLeft = 0, positionRight = 0;
    int ul = 0, vl = 0, ur = 0, vr = 0;
    if ((leftIntPort.getOutputCount())||(leftGrayPort.getOutputCount())) {
        ImageOf<PixelMono>& leftInt = leftIntPort.prepare();
        ImageOf<PixelMono>& leftGrey = leftGrayPort.prepare();
        leftInt.resize(imageLeftInt->width(), imageLeftInt->height());
        if(count % 500 == 0) {
            imageLeftInt->copy(*imageLeft);
            imageLeftGrey->zero();
        }
        else {
            positionLeft = integrateImage(imageLeft, imageLeftInt,imageLeftBW, imageLeftGrey, imageLeftThreshold);
        }
        leftInt.copy(*imageLeftBW);
        leftIntPort.write();
        leftGrey.copy(*imageLeftThreshold);
        leftGrayPort.write(); 
        vl = floor(positionLeft/retinalSize);
        ul = positionLeft%retinalSize;       
    }
    
    if ((rightIntPort.getOutputCount())||(rightGrayPort.getOutputCount())) {
        ImageOf<PixelMono>& rightInt = rightIntPort.prepare();
        ImageOf<PixelMono>& rightGrey = rightGrayPort.prepare();
        rightInt.resize(imageRightInt->width(), imageRightInt->height());
        if(count % 500 == 0) {
            imageRightInt->copy(*imageRight);
            imageRightGrey->zero();
        }
        else {
            positionRight = integrateImage(imageRight, imageRightInt,imageRightBW, imageRightGrey, imageRightThreshold);
        }
        rightInt.copy(*imageRightBW);
        rightIntPort.write();
        rightGrey.copy(*imageRightThreshold);
        rightGrayPort.write();
        vr = floor(positionRight/retinalSize);
        ur = positionRight%retinalSize;
    }
    */
    

    /*
    // extracting the centroid of the integrated image
    //Bottle& eventBottle = eventPort.prepare();
    Vector& centroidStereo = eventPort.prepare();
    if (( ul!= 0) && ( vl!= 0) && ( ur!= 0) && ( vr!= 0)) {
        //eventBottle.addInt(ul);
        //eventBottle.addInt(vl);
        //eventBottle.addInt(ur);
        //eventBottle.addInt(vr);
        //eventPort.write();
        ul = (((((ul - 64)/ 128.0)/ 7.4) * 4) * 320) + 160;
        vl = (((((vl - 64)/ 128.0)/ 7.4) * 4) * 240) + 120;
        ur = (((((ur - 64)/ 128.0)/ 7.4) * 4) * 320) + 160;
        vr = (((((vr - 64)/ 128.0)/ 7.4) * 4) * 240) + 120;
        centroidStereo.clear();
        
        centroidStereo.push_back(ul);
        centroidStereo.push_back(vl);
        centroidStereo.push_back(ur);
        centroidStereo.push_back(vr);
        eventPort.write();
        
        //printf("positionLeft %d-%d, positionRight %d \n", (int) floor(positionLeft/128.0),positionLeft%128, positionRight);
    }
    */
}


int plotterThread::integrateImage(ImageOf<PixelMono>* imageIn, ImageOf<PixelMono>* imageOut, ImageOf<PixelMono>* imageBW, ImageOf<PixelMono>* imageGrey,ImageOf<PixelMono>* imageThreshold ){
    int padding= imageIn->getPadding();
    int centroidX, centroidY;
    unsigned char* pimagein = imageIn->getRawImage();
    unsigned char* pimageout = imageOut->getRawImage();
    unsigned char* pimagebw = imageBW->getRawImage();
    unsigned char* pimagegray = imageGrey->getRawImage();
    //unsigned char* pimagethreshold = imageThreshold->getRawImage();
    int point = 0;
    if(imageIn != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {
	      
                // if(*pimageout == 0) *pimageout = 127;
                
                //if(*pimagein != 127) {
                //*pimagein = 255;
                //}
                unsigned char v = *pimageout;
                
                if ((*pimageout!=127)||(*pimagein!=127)) {	  
                    *pimageout = 250;
                }
                if ((*pimageout!=127) && (*pimagein == 127)){
                    *pimageout = 127;
                }	 	     
                
                if(*pimageout > 128) {
                    *pimagebw = 255;
                }
                else {
                    *pimagebw = 0;
                }
                
                
                unsigned char value = *pimagegray;
                unsigned char valuebw = *pimagebw;
                if(valuebw >  130){
                    value+=16;
                    if(value>200)
                        value=200;
                    //if (value >= 100){ 
                    //    value = 255;
                    //    point = r * retinalSize + c ;
                    //}
                    //imagegray = value;
                }
                //else if(valuebw < 120){
                //value--;
                //if (value <= 10) { 
                //  value = 10;
                //}
                //imagegray = value;
                //}
                *pimagegray = value;
                
                
                pimagegray++;
                pimageout++;
                pimagein++;
                pimagebw++;
            }
            pimageout += padding;
            pimagein += padding;
            pimagebw += padding;
            pimagegray += padding;
        }
        
        cvSmooth(imageGrey->getIplImage(),imageThreshold->getIplImage(), CV_BLUR, 5, 5);
        cvThreshold(imageThreshold->getIplImage(),imageThreshold->getIplImage(), 100,255, CV_THRESH_BINARY);
        //calculating the centroid
        unsigned char* pimageth = imageThreshold->getRawImage();
        int sumX = 0,sumY = 0 ,countX = 0,countY = 0;
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {
                
                if(*pimageth > 127) {
                    countX++;
                    countY++;
                    sumX += c;
                    sumY += r;
                    
                    
                }
                pimageth++;
            }
            pimageth += padding;
        }
        if((countX!=0) && (countY!=0)) {
            centroidX = (int) floor(sumX / countX);
            centroidY = (int )floor(sumY / countY);
        }
    }
    return  centroidY * retinalSize + centroidX;
}


void plotterThread::threadRelease() {
  printf("plotterThread: portClosing \n");  
  leftPort.close();
  leftIntPort.close();
  rightPort.close();
  rightIntPort.close();

  printf("freeing memory \n");
 
  delete imageLeft;
  delete imageRight;
  printf("freed images \n");
  delete imageLeftInt;
  delete imageRightInt;
  printf("freed integrated images \n");
  delete imageLeftBW;
  delete imageRightBW;
  printf("freed bw images \n");
  delete imageLeftGrey;
  delete imageRightGrey;
  printf("freed grey images \n");
  delete imageLeftThreshold;
  delete imageRightThreshold;

  printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
