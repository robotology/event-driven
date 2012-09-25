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
    count        = 0;
    countCalib   = 0;
    retinalSize  = 128;   //default value retinal size before setting    
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    // opening ports 
    leftPort.open(      getName("/left:o"      ).c_str());
    rightPort.open(     getName("/right:o"     ).c_str());
    leftPortBW.open(    getName("/leftBW:o"    ).c_str());
    rightPortBW.open(   getName("/rightBW:o"   ).c_str());
    leftIntPort.open(   getName("/leftInt:o"   ).c_str());
    rightIntPort.open(  getName("/rightInt:o"  ).c_str());
    leftGrayPort.open(  getName("/leftGrey:o"  ).c_str());
    rightGrayPort.open( getName("/rightGrey:o" ).c_str());
    eventPort.open(     getName("/event:o"     ).c_str());
    
    leftPortBWCalib.open(    getName("/leftBWCalib:i"    ).c_str());
    rightPortBWCalib.open(   getName("/rightBWCalib:i"   ).c_str());
    maxCalibPort.open(       getName("/maxCalib:o"       ).c_str());     

    // initialising images
    imageLeft      = new ImageOf<PixelRgb>;
    imageLeft->resize(retinalSize,retinalSize);
    imageLeft->zero();
    imageRight     = new ImageOf<PixelRgb>;
    imageRight->resize(retinalSize,retinalSize);
    imageRight->zero();

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

    for (int i = 0; i < 256; i++) {
            counterWTA[i] = 0;
        }
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    leftPort.interrupt();
    leftIntPort.interrupt();
    rightPort.interrupt();
    rightIntPort.interrupt();
    leftPortBW.interrupt();
    rightPortBW.interrupt();
    leftGrayPort.interrupt();
    rightGrayPort.interrupt();
    eventPort.interrupt();
    leftPortBWCalib.interrupt();
    rightPortBWCalib.interrupt();
    maxCalibPort.interrupt();     
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

void plotterThread::copyLeft(ImageOf<PixelRgb>* image) {
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
}

void plotterThread::copyLeftBW(ImageOf<PixelMono>* image) {
    //printf("retinalSize in plotterThread %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeftBW->getRawImage();
    if(imageLeftBW != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0; c < retinalSize; c++) {                
                *pleft++ = *pimage++;
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}

void plotterThread::copyRight(ImageOf<PixelRgb>* image) {
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
}

void plotterThread::copyRightBW(ImageOf<PixelMono>* image) {
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pright = imageRightBW->getRawImage();
    if(imageRightBW != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {                
                *pright++ = *pimage++;                   
            }
            pright += padding;
            pimage += padding;
        }
    }    
}

void plotterThread::resetCounterWTA() {
    for (int i = 0; i < 256 ; i++) {
        counterWTA[i] = 0;
    }
}

bool plotterThread::centroidCalib(ImageOf<PixelMono>* imageIn) {
    //extracting the centroid
    bool success = false;
    unsigned char* pImage = imageIn->getRawImage();
    int padding           = imageIn->getPadding();
    double rSum = 0;
    double cSum = 0;
    int countC  = 0;
    int countR  = 0;
   
    for (int r = 0; r < retinalSize; r++) {
        for (int c = 0; c < retinalSize; c++) {
            if(*pImage != 0) {
                success = true;
                rSum += r;
                cSum += c;
                countC++;
                countR++;
            } 
            pImage++;
        }
        pImage += padding;
    }

    // check whether centroid has been found
    if(!success) {
        // no active pixel has been found
        //printf("insuccess \n");
        return success;
    }

    // finding the position and updating the maxCounterWTA
    double meanC = cSum / countC;
    double meanR = rSum / countR;
    int x = floor(meanC / 8.0);
    int y = floor(meanR / 8.0);

    int pos  = y * 16 + x;
    //printf("mean %f %f position %d %d > %d \n", meanC, meanR, x, y, pos);
    
    counterWTA[pos]++;
    if(maxCounterWTA < counterWTA[pos]) {
        maxCounterWTA = counterWTA[pos];
        maxCalib = pos;
        //printf("%d with counts %d \n", maxCalib, maxCounterWTA);
    }
    return true;
}


void plotterThread::run() {
    count++;

    synchronised = true;
    if(leftPortBW.getOutputCount()) {
        imageLeftBW  = &leftPortBW.prepare();
        imageLeftBW->resize(retinalSize, retinalSize);
        leftPortBW.write();
    }
    if(rightPortBW.getOutputCount()) {
        imageRightBW = &rightPortBW.prepare();
        imageRightBW->resize(retinalSize, retinalSize);
        rightPortBW.write();
    }
    if(leftPort.getOutputCount()) {
        imageLeft  = &leftPort.prepare();
        imageLeft->resize(retinalSize, retinalSize);
        leftPort.write();
    }
    if(rightPort.getOutputCount()) {
        imageRight = &rightPort.prepare();
        imageRight->resize(retinalSize, retinalSize);
        rightPort.write();
    }
    
    if(leftPortBWCalib.getInputCount()) {
        ImageOf<PixelMono>* leftBWCalib = leftPortBWCalib.read(false);
        if(leftBWCalib != NULL) {
            
            int x = 10; int y = 10;
            //int pos  = y * retinalSize + x;
            //int xpos = x / 16;
            //int ypos = y / 16;

            bool success = centroidCalib(leftBWCalib);
            if(success) {
                //printf("success!; counting WTAs \n");
                countCalib++;
                
            }
            
            if (countCalib > 20 && maxCalibPort.getOutputCount()) {
                //printf("counted upto 10 WTA \n");
                //send maxCalibX and maxCaliby
                int maxCalibX, maxCalibY;
                maxCalibY = (int) std::floor((double)maxCalib / 16);
                maxCalibX = maxCalib - maxCalibY * 16;
                maxCalibY = maxCalibY * 8;
                maxCalibX = maxCalibX * 8;
                
                Bottle& calibBottle=maxCalibPort.prepare();
                calibBottle.clear();
                calibBottle.addInt(maxCalibX);
                calibBottle.addInt(maxCalibY);
                maxCalibPort.write();
                countCalib    = 0;
                maxCounterWTA = 0; 
                resetCounterWTA();
            }
        }
    }
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
            centroidX = (int) std::floor((double)sumX / countX);
            centroidY = (int )std::floor((double)sumY / countY);
        }
    }
    return  centroidY * retinalSize + centroidX;
}


void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    leftPort.close();
    rightPort.close();
    leftPortBW.close();
    rightPortBW.close();
    leftIntPort.close();
    rightIntPort.close();
    leftGrayPort.close();
    rightGrayPort.close();
    eventPort.close();
    leftPortBWCalib.close();
    rightPortBWCalib.close();
    maxCalibPort.close();

    printf("freeing memory \n");

    // delete imageLeft;
    //delete imageRight;
    delete imageLeftInt;
    delete imageRightInt;
    delete imageLeftBW;
    delete imageRightBW;
    delete imageLeftGrey;
    delete imageRightGrey;
    delete imageLeftThreshold;
    delete imageRightThreshold;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
