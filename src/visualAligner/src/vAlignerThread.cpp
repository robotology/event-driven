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
 * @file vAlignerThread.cpp
 * @brief Implementation of the thread (see header vaThread.h)
 */

#include <iCub/vAlignerThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define THRATE 30
#define SHIFTCONST 100

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

vAlignerThread::vAlignerThread() : RateThread(THRATE) {
    resized=false;
    count=0;
    leftDragonImage = 0;
    rightDragonImage = 0;
    shiftValue=20;
}

vAlignerThread::~vAlignerThread() {
    if(leftDragonImage!=0) {
        delete leftDragonImage;
    }
    if(rightDragonImage!=0) {
        delete leftDragonImage;
    }
}

bool vAlignerThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    leftDragonPort.open(getName("/leftDragon:i").c_str());
    rightDragonPort.open(getName("/rightDragon:i").c_str());
    vergencePort.open(getName("/vergence:i").c_str());
    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;
    return true;
}

void vAlignerThread::interrupt() {
    outPort.interrupt();
    leftDragonPort.interrupt();
    rightDragonPort.interrupt();
    vergencePort.interrupt();
}

void vAlignerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string vAlignerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void vAlignerThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    leftDragonImage=new ImageOf<PixelRgb>;
    leftDragonImage->resize(width, height);
    rightDragonImage=new ImageOf<PixelRgb>;
    rightDragonImage->resize(width, height);
}

void vAlignerThread::run() {
    count++;
    if(outPort.getOutputCount()) {
        if(leftDragonPort.getInputCount()) {
             tmp = leftDragonPort.read(false);
             if(tmp!=0) {
                 if(!resized) {
                    resized = true;
                    resize(tmp->width(), tmp->height());
                 }
                 else {
                    ImageOf<yarp::sig::PixelRgb>& outputImage=outPort.prepare();
                    outputImage.resize(width, height);
                    copy_8u_C3R(tmp,leftDragonImage);
                 }
             }
        }
        if(rightDragonPort.getInputCount()) {
             tmp = rightDragonPort.read(false);
             if(tmp!=0) {
                 if(!resized) {
                    resized = true;
                    resize(tmp->width(), tmp->height());
                 }
                 else {
                    copy_8u_C3R(tmp,rightDragonImage);
                 }
             }
        }
        
        ImageOf<yarp::sig::PixelRgb>& outputImage=outPort.prepare();
        if(resized) {
            Vector angles(3);
            bool b = igaze->getAngles(angles);
            double h, alfa, rightHat, leftHat;
            printf("azim %f, elevation %f, vergence %f \n",angles[0],angles[1],angles[2]);
            double vergence = (angles[2] * 3.14) / 180;
            double version = (angles[0] * 3.14) / 180;
            outputImage.resize(width + shiftValue, height);
            shift(shiftValue,outputImage);
            outPort.write();
        }
    }
}

void vAlignerThread::shift(int shift, ImageOf<PixelRgb>& outImage) {
    unsigned char* pRight = rightDragonImage->getRawImage();
    unsigned char* pLeft = leftDragonImage->getRawImage();
    int padding = leftDragonImage->getPadding();
    int paddingOut = outImage.getPadding();
    unsigned char* pOutput=outImage.getRawImage();
    double d;
    double centerX=width + shift;
    double centerY=height;
    if(shift >= 0) {
        for (int row = 0;row < height;row++) {
            //pRight += shift*3;
            for (int col = 0 ; col < shift ; col++) {
                //d=sqrt( (row - centerY) * (row - centerY) 
                //    + (col - centerX) * (col - centerX));
                *pOutput++ = (unsigned char) *pLeft++;
                *pOutput++ = (unsigned char) *pLeft++;
                *pOutput++ = (unsigned char) *pLeft++;
            }
            for (int col = shift ; col < width ; col++) {
                //d=sqrt( (row - centerY) * (row - centerY) 
                //    + (col - centerX) * (col - centerX));
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
            }
            for(int col = width ; col < width + shift ; col++){
                //d=sqrt( (row - centerY) * (row - centerY) 
                //    + (col - centerX) * (col - centerX));
                *pOutput++ = (unsigned char) *pRight++;
                *pOutput++ = (unsigned char) *pRight++;
                *pOutput++ = (unsigned char) *pRight++;
            }
            //padding
            pLeft += padding;
            pRight += padding;
            pOutput += paddingOut;
        }
    }
    else {
        shift=0-shift;
        for (int row=0;row<height;row++) {
            pLeft+=shift*3;
            for (int col=0;col<width-shift;col++) {
                //d=sqrt( (row - centerY) * (row - centerY) 
                //    + (col - centerX) * (col - centerX));
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor( (0.5 * *pLeft + 0.5 * *pRight));
                pLeft++;pRight++;pOutput++;
            }
            for(int col=width-shift;col<width;col++){
                //d=sqrt( (row - centerY) * (row - centerY) 
                //    + (col - centerX) * (col - centerX));
                *pOutput++=(unsigned char) *pRight++;
                *pOutput++=(unsigned char) *pRight++;
                *pOutput++=(unsigned char) *pRight++;
            }
            //padding
            pLeft+=padding;
            pRight+=padding;
            pOutput+=padding;
        }
    }
}


void vAlignerThread::threadRelease() {
    delete clientGazeCtrl;
    outPort.close();
    leftDragonPort.close();
    rightDragonPort.close();
    vergencePort.close();
}

