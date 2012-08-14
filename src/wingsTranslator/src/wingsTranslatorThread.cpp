// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea  starting from code by Ugo Pattacini
  * email: francesco.rea@iit.it, ugo.pattacini@iit.it
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
 * @file targetFinderThread.cpp
 * @brief Implementation of the thread (see header targetFinderThread.h)
 */


#define CHUNKSIZE 32768
#define THRATE    30
//#define VERBOSE


#include <iCub/wingsTranslatorThread.h>

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cstring>
#include <cassert>

using namespace iCub::iKin;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace std;


/*******************************************************************/

inline int convertChar2Dec(char value) {
    if (value > 60)
        return value - 97 + 10;
    else
        return value - 48;
}

/**********************************************************************/

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

/************************************************************************/

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


/************************************************************************/

bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble();
                double fy = parType.find("fy").asDouble();

                Matrix K  = eye(3,3);
                Matrix Pi = zeros(3,4);

                K(0,0) = fx;
                K(1,1) = fy;
                K(0,2) = cx;
                K(1,2) = cy; 
                
                Pi(0,0) = Pi(1,1) = Pi(2,2) = 1.0;

                *Prj = new Matrix;
                **Prj = K * Pi;

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}

/**********************************************************************************/




void iCubHeadCenter::allocate(const std::string &_type) {
    // change DH parameters
    (*this)[getN()-2].setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);
}

/********************************************************************/

wingsTranslatorThread::wingsTranslatorThread() : RateThread(THRATE) {
    resized = false;
    count           = 0;
    countEvent      = 0;
    leftInputImage  = 0;
    rightInputImage = 0;
    shiftValue      = 20;

    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE); 

}

wingsTranslatorThread::~wingsTranslatorThread() {
    free(bufferCopy);
}

bool wingsTranslatorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/coord:o").c_str());
    outRightPort.open(getName("/edgesRight:o").c_str());
    inPort.open(getName("/coord:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    

}


/************************************************************************/
Vector wingsTranslatorThread::getAbsAngles(const Vector &x) {
    Vector fp = x;
    if(fp.size() == 3) {
        fp.push_back(1.0);  // impose homogeneous coordinates
    }

    // get fp wrt head-centered system
    //printf("get fp wrt head-centered system \n");
    //printf(" %s \n",invEyeCAbsFrame.toString().c_str() );
    //printf(" %s \n", fp.toString().c_str());
    Vector fph = invEyeCAbsFrame * fp;
    fph.pop_back();

    Vector ang(3);
    //printf("angles extraction \n");
    ang[0] =       atan2(fph[0],fph[2]);
    ang[1] =      -atan2(fph[1],fabs(fph[2]));
    ang[2] = 2.0 * atan2(eyesHalfBaseline,norm(fph));

    //printf("end of the function \n");
    return ang;
}


/************************************************************************/
Vector wingsTranslatorThread::get3DPoint(const string &type, const Vector &ang)
{
    /*
    double azi = ang[0];
    double ele = ang[1];
    double ver = ang[2];

    Vector q(8,0.0);
    if (type == "rel")
    {
        //Vector torso = commData->get_torso();
        //Vector head  = commData->get_q();
        
        // reading proprioception information
        Vector torso(3);        
        encTorso->getEncoder(0,&torso[0]);
        encTorso->getEncoder(1,&torso[1]);
        encTorso->getEncoder(2,&torso[2]);
        
        Vector head(6);
        encHead->getEncoder(0,&head[0]);
        encHead->getEncoder(1,&head[1]);
        encHead->getEncoder(2,&head[2]);
        encHead->getEncoder(3,&head[3]);
        encHead->getEncoder(4,&head[4]);
        encHead->getEncoder(4,&head[5]);

        q[0] = torso[0];
        q[1] = torso[1];
        q[2] = torso[2];
        q[3] = head[0];
        q[4] = head[1];
        q[5] = head[2];
        q[6] = head[3];
        q[7] = head[4];

        ver += head[5];
    }
    
    // impose vergence != 0.0
    if (ver < 0.0)
        ver = 0.0;

    mutex.wait();

    q[7] += ver/2.0;
    eyeL->setAng(q);

    q[7] -= ver;
    eyeR->setAng(q);

    Vector fp(4);
    fp[3]=1.0;  // impose homogeneous coordinates

    // compute new fp due to changed vergence
    computeFixationPointOnly(*(eyeL->asChain()),*(eyeR->asChain()),fp);
    mutex.post();

    // compute rotational matrix to
    // account for elevation and azimuth
    Vector x(4), y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=ele;    y[3]=azi;   
    Matrix R = axis2dcm(y)*axis2dcm(x);

    Vector fph, xd;
    if (type == "rel")
    {
        Matrix frame = commData->get_fpFrame();
        fph = SE3inv(frame)*fp;       // get fp wrt relative head-centered frame
        xd = frame*(R*fph);           // apply rotation and retrieve fp wrt root frame
    }
    else
    {
        fph = invEyeCAbsFrame*fp;     // get fp wrt absolute head-centered frame
        xd = eyeCAbsFrame*(R*fph);    // apply rotation and retrieve fp wrt root frame
    }

    return xd.subVector(0,2);
    */
}

void wingsTranslatorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string wingsTranslatorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void wingsTranslatorThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    //leftInputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(width, height);
    //rightInputImage = new ImageOf<PixelMono>;
    //rightInputImage->resize(width, height);
}

void wingsTranslatorThread::run() {
    
    
}

void wingsTranslatorThread::interrupt() {
    outPort.interrupt();
    outRightPort.interrupt();
    inPort.interrupt();
    inRightPort.interrupt();

}

void wingsTranslatorThread::threadRelease() {
    printf("wingsTranslatorThread::threadRelease \n");
    /* closing the ports*/
    outPort.close();
    outRightPort.close();
    inPort.close();
    inRightPort.close();

 
    delete eyeL;
    delete eyeR;
    igaze->restoreContext(originalContext);
    delete clientGazeCtrl;
   
    /* closing the file */   
    if(pFile!=NULL) {
        fclose (pFile); 
    }

    if(fout!=NULL) {
        fclose (fout); 
    }
    if(fdebug!=NULL) {
        fclose (fdebug); 
    }
}

