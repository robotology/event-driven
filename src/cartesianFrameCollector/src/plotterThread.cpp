
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


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 30 
#define retinalSize 128

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    lambda = 0.9;
    imageLeft = 0;
    imageRight = 0;
    imageLeftInt = new ImageOf<PixelMono>;
    imageLeftInt->resize(retinalSize,retinalSize);
    imageLeftInt->zero();
    imageRightInt = new ImageOf<PixelMono>;
    imageRightInt->resize(retinalSize,retinalSize);
    imageRightInt->zero();
    imageLeftBW = new ImageOf<PixelMono>;
    imageLeftBW->resize(retinalSize,retinalSize);
    imageLeftBW->zero();
    imageRightBW = new ImageOf<PixelMono>;
    imageRightBW->resize(retinalSize,retinalSize);
    imageRightBW->zero();
    imageLeftGray = new ImageOf<PixelMono>;
    imageLeftGray->resize(retinalSize,retinalSize);
    imageLeftGray->zero();
    imageRightGray = new ImageOf<PixelMono>;
    imageRightGray->resize(retinalSize,retinalSize);
    imageRightGray->zero();
}

plotterThread::~plotterThread() {
    printf("freeing memory in collector");
    delete imageLeftInt;
    delete imageRightInt;
    delete imageLeftBW;
    delete imageRightBW;
    delete imageLeftGray;
    delete imageRightGray;
}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    /* open ports */
    leftPort.open(getName("/left:o").c_str());
    rightPort.open(getName("/right:o").c_str());
    leftIntPort.open(getName("/leftInt:o").c_str());
    rightIntPort.open(getName("/rightInt:o").c_str());
    leftGrayPort.open(getName("/leftGray:o").c_str());
    rightGrayPort.open(getName("/rightGray:o").c_str());
    eventPort.open(getName("/event:o").c_str());
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
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0; c < retinalSize; c++) {
                *pleft++ = *pimage++;
            }
            pleft += padding;
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

    int positionLeft = 0, positionRight = 0;
    int ul = 0, vl = 0, ur = 0, vr = 0;

    if ((leftIntPort.getOutputCount())&&(leftGrayPort.getOutputCount())) {
        ImageOf<PixelMono>& leftInt = leftIntPort.prepare();
	ImageOf<PixelMono>& leftGray = leftGrayPort.prepare();
        leftInt.resize(imageLeftInt->width(), imageLeftInt->height());
        if(count % 500 == 0) {
	  imageLeftInt->copy(*imageLeft);
	  imageLeftGray->zero();
	}
        else {
	  positionLeft = integrateImage(imageLeft, imageLeftInt,imageLeftBW, imageLeftGray);
	}
	leftInt.copy(*imageLeftBW);
	leftIntPort.write();
	leftGray.copy(*imageLeftGray);
	leftGrayPort.write(); 
	ul = floor(positionLeft/retinalSize);
	vl = positionLeft%retinalSize;       
    }
    
    if (rightIntPort.getOutputCount()) {
        ImageOf<PixelMono>& rightInt = rightIntPort.prepare();
	ImageOf<PixelMono>& rightGray = rightGrayPort.prepare();
        rightInt.resize(imageRightInt->width(), imageRightInt->height());
        if(count % 500 == 0) {
	  imageRightInt->copy(*imageRight);
	  imageRightGray->zero();
	}
        else {
	  positionRight = integrateImage(imageRight, imageRightInt,imageRightBW, imageRightGray);
	}
	rightInt.copy(*imageRightBW);
	rightIntPort.write();
	rightGray.copy(*imageRightGray);
	rightGrayPort.write();
	ur = floor(positionRight/retinalSize);
	vr = positionLeft%retinalSize;
    }

    Bottle& eventBottle = eventPort.prepare();
    if (( ul!= 0) && ( vl!= 0) && ( ur!= 0) && ( vr!= 0)) {
      eventBottle.addInt(ul);
      eventBottle.addInt(vl);
      eventBottle.addInt(ur);
      eventBottle.addInt(vr);
      eventPort.write();
      printf("positionLeft %d-%d, positionRight %d \n", (int) floor(positionLeft/128.0),positionLeft%128, positionRight);
    }
}


int plotterThread::integrateImage(ImageOf<PixelMono>* imageIn, ImageOf<PixelMono>* imageOut, ImageOf<PixelMono>* imageBW, ImageOf<PixelMono>* imageGray){
    int padding= imageIn->getPadding();
    unsigned char* pimagein = imageIn->getRawImage();
    unsigned char* pimageout = imageOut->getRawImage();
    unsigned char* pimagebw = imageBW->getRawImage();
    unsigned char* pimagegray = imageGray->getRawImage();
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
	      	value+=10;
		if (value >= 100){ 
		    value = 234;
		    point = r * retinalSize + c ;
		}
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
    }
    return point;
}


void plotterThread::threadRelease() {
  printf("plotterThread: portClosing \n");  
  leftPort.close();
  leftIntPort.close();
  rightPort.close();
  rightIntPort.close();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
