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
                //double cx = parType.find("w").asDouble() / 2.0;
                //double cy = parType.find("h").asDouble() / 2.0;
                double cx = parType.find("cx").asDouble();
                double cy = parType.find("cy").asDouble();
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

bool project(yarp::dev::IEncoders *encTorso,yarp::dev::IEncoders *encHead,
yarp::sig::Matrix *invPrjL,iCub::iKin::iCubEye *eye,int u, int v, double varDistance, Vector &xo) {
    Vector fp(3);
    
    Vector torso(3);
    encTorso->getEncoder(0,&torso[0]);
    encTorso->getEncoder(1,&torso[1]);
    encTorso->getEncoder(2,&torso[2]);
    //printf("torso %f %f %f \n", torso[0], torso[1], torso[2]);
    Vector head(5);
    encHead->getEncoder(0,&head[0]);
    encHead->getEncoder(1,&head[1]);
    encHead->getEncoder(2,&head[2]);
    encHead->getEncoder(3,&head[3]);
    encHead->getEncoder(4,&head[4]);
    //printf("head %f %f %f %f %f %f \n", head[0], head[1], head[2], head[3], head[4]);
    
    Vector q(8);
    double ratio = M_PI /180;
    q[0]=torso[0] * ratio;
    q[1]=torso[1] * ratio;
    q[2]=torso[2] * ratio;
    q[3]=head[0]  * ratio;
    q[4]=head[1]  * ratio;
    q[5]=head[2]  * ratio;
    q[6]=head[3]  * ratio;
    q[7]=head[4]  * ratio;
    double ver = head[5];                    
    
    Vector x(3);
    //printf("varDistance %f \n", varDistance);
    x[0] = varDistance * u;   //epipolar correction excluded the focal lenght
    x[1] = varDistance * v;
    x[2] = varDistance;
    
    // find the 3D position from the 2D projection,
    // knowing the distance z from the camera
    //printf("finding the 3D position from the 2D projection \n");
    Vector xe = yarp::math::operator *(*invPrjL, x);
    xe[3]=1.0;  // impose homogeneous coordinates                
    
    // update position wrt the root frame
    //printf("updating the 3D position wrt the root frame %d \n",eye->getN());
    
    Matrix eyeH = eye->getH(q);
    // printf("success in getH(q) \n");
    //printf("eyeH : %f %f %f \n %f %f %f \n %f %f %f \n ", 
    //       eyeH(0,0), eyeH(0,1), eyeH(0,2),
    //       eyeH(1,0), eyeH(1,1), eyeH(1,2),
    //       eyeH(2,0), eyeH(2,1), eyeH(2,2)
    //       );
    //xo = yarp::math::operator *(eyeH,xe).subVector(0,2);
    xo=(eye->getH(q)*xe).subVector(0,2);

//printf("object %f,%f,%f \n",xo[0],xo[1],xo[2]);    

    return true;
}

/********************************************************************/

void iCubHeadCenter::allocate(const std::string &_type) {
    // change DH parameters
    (*this)[getN()-2].setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);
}

/********************************************************************/

wingsTranslatorThread::wingsTranslatorThread() /*: RateThread(THRATE)*/ {
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

    Vector v(3);
    p0 = v;
    n  = v;
    
    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append("testWings");
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;
   
    igaze->storeContext(&originalContext);
  
    blockNeckPitchValue = -1;
    if(blockNeckPitchValue != -1) {
        igaze->blockNeckPitch(blockNeckPitchValue);
        printf("pitch fixed at %f \n",blockNeckPitchValue);
    }
    else {
        printf("pitch free to change\n");
    }

    // -----------------------------------------------------------------
    
    Bottle info;
    igaze->getInfo(info);
    printf("getting info from iKinGazeCtrl \n");    
    int head_version = info.check("head_version", Value(1)).asInt();
    
    printf("head_version extracted from gazeArbiter %d \n",head_version);
    ikl = new iCubEye();
    ikr = new iCubEye();
    
    head_version = 2;
    if(head_version == 1) {
        eyeL = new iCubEye("left");
        eyeR = new iCubEye("right");
    }
    else {
        eyeL = new iCubEye("left_v2");
        eyeR = new iCubEye("right_v2");
    }
    printf("correctly istantiated the head \n");
    
    // if it isOnWings, move the eyes on top of the head 
    //isOnWings = true;
    if (isOnWings) {
        printf("changing the structure of the chain \n");
           
        yarp::os::Property p; 
        printf("left camera : extracting properties from %s \n",wingsLeftFile.c_str() );
        p.fromConfigFile(wingsLeftFile.c_str()); 
        ikl->fromLinksProperties(p);
        eyeL = ikl; 
        printf("eyeL is valid? %d \n", eyeL->isValid());
        
        printf("H0 = \n %s \n", eyeL->getH0().toString().c_str());
        iKinChain* tmpChain = eyeL->asChain();
        for(int j = 0; j < eyeL->getN(); j++) {
            printf("LINK %d :",j);
            iKinLink tmpLink = tmpChain->operator[](j);
            printf("              A %f D %f alpha %f offset %f min %f max %f blocked %d\n",
                   tmpLink.getA(), tmpLink.getD(),tmpLink.getAlpha(), tmpLink.getOffset(), tmpLink.getMin(), tmpLink.getMax(), tmpLink.isBlocked() );
        }
        
        printf("right camera : extracting properties from %s \n",wingsRightFile.c_str() );
        p.fromConfigFile(wingsRightFile.c_str()); 
        ikr->fromLinksProperties(p);
        eyeR = ikr; 
        printf("eyer is valid? %d \n", eyeR->isValid());
        
        printf("H0 = \n %s \n", eyeR->getH0().toString().c_str());
        tmpChain = eyeR->asChain();
        for(int j = 0; j < eyeR->getN(); j++) {
            printf("LINK %d :",j);
            iKinLink tmpLink = tmpChain->operator[](j);
            printf("              A %f D %f alpha %f offset %f min %f max %f blocked %d\n",
                   tmpLink.getA(), tmpLink.getD(),tmpLink.getAlpha(), tmpLink.getOffset(), tmpLink.getMin(), tmpLink.getMax(), tmpLink.isBlocked() );
        }
        
    }
    else {
        printf("isOnWing false \n");
        printf("H0 = \n %s \n", eyeL->getH0().toString().c_str());
        iKinChain* tmpChain = eyeL->asChain();
        for(int j = 0; j < eyeL->getN(); j++) {
            printf("LINK %d :",j);
            iKinLink tmpLink = tmpChain->operator[](j);
            printf("              A %f D %f alpha %f offset %f min %f max %f blocked %d\n",
                   tmpLink.getA(), tmpLink.getD(),tmpLink.getAlpha(), tmpLink.getOffset(), tmpLink.getMin(), tmpLink.getMax(), tmpLink.isBlocked());
        }
        tmpChain = eyeR->asChain();
        for(int j = 0; j < eyeR->getN(); j++) {
            printf("LINK %d :",j);
            iKinLink tmpLink = tmpChain->operator[](j);
            printf("              A %f D %f alpha %f offset %f min %f max %f blocked %d\n",
                   tmpLink.getA(), tmpLink.getD(),tmpLink.getAlpha(), tmpLink.getOffset(), tmpLink.getMin(), tmpLink.getMax(), tmpLink.isBlocked());
        }
    }
    //---------------------------------------------------------------------------------------------------------------
    
    printf("introducing additional constraints \n");
    
    // remove constraints on the links
    // we use the chains for logging purpose
    // eyeL->setAllConstraints(false);
    // eyeR->setAllConstraints(false);
    
    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);    
    
    //eyeL->alignJointsBounds()

    // get camera projection matrix from the configFile
    printf("get Camera configuration \n");
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        cxl=Prj(0,2);
        cyl=Prj(1,2);
        printf("pixel fovea in the config file %f %f %f %f \n",  Prj(0,0), Prj(1,1), cxl,cyl);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR)) {
        Matrix &Prj = *PrjR;
        cxr=Prj(0,2);
        cyr=Prj(1,2);
        printf("pixel fovea in the config file %f %f %f %f \n",  Prj(0,0), Prj(1,1), cxr,cyr);
        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    // ------------------------------------------------------------------

    printf("initialising polyDriver head \n");
    string headPort = "/" + robot + "/head";
    string nameLocal("testWings");

    //initialising the head polydriver
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", ("/"+nameLocal+"/localhead").c_str());
    optionsHead.put("remote", headPort.c_str());
    robotHead = new PolyDriver (optionsHead);

    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    else {
        printf("robotHead is valid \n");
    }
    robotHead->view(encHead);
    robotHead->view(limHead);
    
    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    optPolyTorso.put("remote",("/"+robot+"/torso").c_str());
    optPolyTorso.put("local",("/"+nameLocal+"/torso/position").c_str());
    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
        return false;
    }
    polyTorso->view(encTorso);
    polyTorso->view(limTorso);
    //------------------------------------------------------------------------

    printf("defining limits \n");
    std::deque<yarp::dev::IControlLimits *> limQueue;
    limQueue.push_back(limTorso);
    limQueue.push_back(limHead);
    eyeL->alignJointsBounds(limQueue);
    eyeR->alignJointsBounds(limQueue);
    printf("success in aligning the JointsBounds \n");

    fmatch = fopen("match.txt", "w+");

    return true;
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
Vector wingsTranslatorThread::get3DPoint(const string &type, const Vector &ang) {

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

Vector wingsTranslatorThread::get3dWingsLeft(int u , int v) {
    printf("starting with the projections \n");
    Vector errorVector(3);
    Vector x(3);
    Vector x2(3);
    errorVector.zero();
    ConstString type = "left";
    bool isLeft=(type=="left");
    iCubEye *eye=(isLeft?eyeL:eyeR);
    
    int operation = 1;
    switch (operation) {
        
    case 0 : {
        Vector xo;
        //calculating the 3d position and sending it to database
        int u = 160; 
        int v = 120;
        int varDistance  = 0.5;
        project(encTorso,encHead,invPrjL,eyeL, u,v, varDistance, xo);
        
        return xo;
    }break;
    case 1: {
        
        //Vector px(2);   // specify the pixel where to look
        //px[0]=160.0;
        //px[1]=120.0;
        
        //int u = 160;
        //int v = 120;
        
        eye = eyeL;
        
        printf("going to project the point \n");
        //if (projectPoint(type,u,v,1.0,x))
        if(project(encTorso,encHead,invPrjL,eyeL,u,v,1.0,x)) {
            // pick up a point belonging to the plane
            
            
            //mutex.wait();
            Vector e = eye->EndEffPose().subVector(0,2);
            //mutex.post();
            
            // compute the projection
            printf("computing the projection e=%s  \n", e.toString().c_str());
            Vector v = x - e;

            mutexN.wait();
            mutexP0.wait();
            x = e + ( dot(p0-e,n ) / dot(x - e,n)) * v;
            mutexP0.post();
            mutexN.post();
            

            //Vector k = e;
            printf("------------ x %s \n",x.toString().c_str());
            
            
        }
        else{
            return errorVector;
        }
    }break;
    } // end of the switch

    
    bool searchParam = false;
    if (searchParam) {
        
        Matrix &Prj = *PrjL;
        double origPrj00 = Prj(0,0);
        double origPrj11 = Prj(1,1);
        double origPrj02 = Prj(0,2);
        double origPrj12 = Prj(1,2);
        
        Vector e,ve;
        invPrjL = new Matrix(pinv(Prj.transposed()).transposed());
        int count;
        
        for (int cx= -0; cx < 1; cx+=1) {
            for  (int cy= -20; cy < 20; cy+=1) {
                
                count = 0;
                for (int fx= -40; fx < 200; fx+=1) {
                    for (int fy = -160; fy < 200; fy+=1) {
                            
                            cxl=Prj(0,2);
                            cyl=Prj(1,2);
                            //printf("pixel fovea in the config file %d %d \n", cxl,cyl);
                            Prj(0,0) = origPrj00 + fx;
                            Prj(1,1) = origPrj11 + fy;
                            Prj(0,2) = origPrj02 + cx;
                            Prj(1,2) = origPrj12 + cy;
                            *invPrjL = pinv(Prj.transposed()).transposed();
                            
                            project(encTorso,encHead,invPrjL,eyeL,u,v,1.0,x2); 
                            
                            e = eye->EndEffPose().subVector(0,2);
                            //mutex.post();
                            
                            // compute the projection
                            //printf("computing the projection e=%s  \n", e.toString().c_str());
                            ve = x2 - e;
                            
                            mutexN.wait();
                            mutexP0.wait();
                            x2 = e + ( dot(p0-e,n ) / dot(x2 - e,n)) * ve;
                            mutexP0.post();
                            mutexN.post();
                            
                            double distanceTarget = sqrt(
                                                         (x2(0) - xTarget) * (x2(0) - xTarget)
                                                         +
                                                         (x2(1) - yTarget) * (x2(1) - yTarget)
                                                         );
                            
                            //printf("  dist %f > %f %f %f %f -- x2 %s \n  ",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                            
                            if (distanceTarget < 0.001) {
                                printf("  dist %f > %f %f %f %f -- x2 %s \n  ",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                                fprintf(fmatch, "%f %f %f %f %f\n",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                                count++;
                            }
                        }
                             
                             }
                    
                    
                    if(count > 0) {
                        printf("----------------------------------- cx %d cy %d \n", cx, cy);
                        //fprintf(fmatch,"%d %d %d \n", cx, cy, count);
                        printf("--------------------------------count %d \n\n\n\n", count);
                    }
                }
            }
            printf("END of the SEARCH \n");
        
            Prj(0,0) = origPrj00;
            Prj(1,1) = origPrj11;
            Prj(0,2) = origPrj02;
            Prj(1,2) = origPrj12;
        }
    

    return x;
}

Vector wingsTranslatorThread::get3dWingsRight(int u , int v) {
    printf("starting with the projections \n");
    Vector errorVector(3);
    Vector x(3);
    Vector x2(3);
    errorVector.zero();
    ConstString type = "right";
    bool isLeft=(type=="left");
    iCubEye *eye=(isLeft?eyeL:eyeR);
    
    int operation = 1;
    switch (operation) {
        
    case 0 : {
        Vector xo;
        //calculating the 3d position and sending it to database
        int u = 160; 
        int v = 120;
        int varDistance  = 0.5;
        project(encTorso,encHead,invPrjR,eyeR, u,v, varDistance, xo);
        
        return xo;
    }break;
    case 1: {
        
        //Vector px(2);   // specify the pixel where to look
        //px[0]=160.0;
        //px[1]=120.0;
        
        //int u = 160;
        //int v = 120;
        
        eye = eyeR;
        
        printf("going to project the point \n");
        //if (projectPoint(type,u,v,1.0,x))
        if(project(encTorso,encHead,invPrjR,eyeR,u,v,1.0,x)) {
            // pick up a point belonging to the plane
            //mutex.wait();
            Vector e = eye->EndEffPose().subVector(0,2);
            //mutex.post();
            
            // compute the projection
            printf("computing the projection e=%s  \n", e.toString().c_str());
            Vector v = x - e;

            mutexN.wait();
            mutexP0.wait();
            x = e + ( dot(p0-e,n ) / dot(x - e,n)) * v;
            mutexP0.post();
            mutexN.post();
            

            //Vector k = e;
            printf("------------ x %s \n",x.toString().c_str());
            
            
        }
        else{
            return errorVector;
        }
    }break;
    } // end of the switch

    
    bool searchParam = false;
    if (searchParam) {
        
        Matrix &Prj = *PrjR;
        double origPrj00 = Prj(0,0);
        double origPrj11 = Prj(1,1);
        double origPrj02 = Prj(0,2);
        double origPrj12 = Prj(1,2);
        
        Vector e,ve;
        invPrjR = new Matrix(pinv(Prj.transposed()).transposed());
        int count;
        
        for (int cx= -0; cx < 1; cx+=1) {
            for  (int cy= -20; cy < 20; cy+=1) {
                
                count = 0;
                for (int fx= -40; fx < 200; fx+=1) {
                    for (int fy = -160; fy < 200; fy+=1) {
                            
                            cxl=Prj(0,2);
                            cyl=Prj(1,2);
                            //printf("pixel fovea in the config file %d %d \n", cxl,cyl);
                            Prj(0,0) = origPrj00 + fx;
                            Prj(1,1) = origPrj11 + fy;
                            Prj(0,2) = origPrj02 + cx;
                            Prj(1,2) = origPrj12 + cy;
                            *invPrjL = pinv(Prj.transposed()).transposed();
                            
                            project(encTorso,encHead,invPrjR,eyeR,u,v,1.0,x2); 
                            
                            e = eye->EndEffPose().subVector(0,2);
                            //mutex.post();
                            
                            // compute the projection
                            //printf("computing the projection e=%s  \n", e.toString().c_str());
                            ve = x2 - e;
                            
                            mutexN.wait();
                            mutexP0.wait();
                            x2 = e + ( dot(p0-e,n ) / dot(x2 - e,n)) * ve;
                            mutexP0.post();
                            mutexN.post();
                            
                            double distanceTarget = sqrt(
                                                         (x2(0) - xTarget) * (x2(0) - xTarget)
                                                         +
                                                         (x2(1) - yTarget) * (x2(1) - yTarget)
                                                         );
                            
                            //printf("  dist %f > %f %f %f %f -- x2 %s \n  ",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                            
                            if (distanceTarget < 0.001) {
                                printf("  dist %f > %f %f %f %f -- x2 %s \n  ",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                                fprintf(fmatch, "%f %f %f %f %f\n",distanceTarget, Prj(0,0), Prj(1,1), Prj(0,2), Prj(1,2), x2.toString().c_str());
                                count++;
                            }
                        }
                             
                             }
                    
                    
                    if(count > 0) {
                        printf("----------------------------------- cx %d cy %d \n", cx, cy);
                        //fprintf(fmatch,"%d %d %d \n", cx, cy, count);
                        printf("--------------------------------count %d \n\n\n\n", count);
                    }
                }
            }
            printf("END of the SEARCH \n");
        
            Prj(0,0) = origPrj00;
            Prj(1,1) = origPrj11;
            Prj(0,2) = origPrj02;
            Prj(1,2) = origPrj12;
        }
    

    return x;
}

/*
Vector wingsTranslatorThread::get3dWingsRight(int u , int v) {
    printf("starting with the projections \n");
    Vector errorVector(3);
    Vector x(3);
    errorVector.zero();
    
    int operation = 1;
    switch (operation) {
        
    case 0 : {
        Vector xo;
        //calculating the 3d position and sending it to database
        int u = 160; 
        int v = 120;
        int varDistance  = 0.5;
        project(encTorso,encHead,invPrjR,eyeR, u,v, varDistance, xo);
        
        return xo;
    }break;
    case 1: {
        
        ConstString type = "right";
        bool isRight=(type=="right");
        iCubEye *eye=(isRight?eyeR:eyeL);
        eye = eyeR;
        
        printf("going to project the point \n");
        //if (projectPoint(type,u,v,1.0,x))
        if(project(encTorso,encHead,invPrjR,eyeR,u,v,1.0,x)) {
            // pick up a point belonging to the plane
            
            
            //mutex.wait();
            Vector e = eye->EndEffPose().subVector(0,2);
            //mutex.post();
            
            // compute the projection
            printf("computing the projection e=%s  \n", e.toString().c_str());
            Vector v = x - e;

            mutexN.wait();
            mutexP0.wait();
            x = e + ( dot(p0-e,n ) / dot(x - e,n)) * v;
            mutexP0.post();
            mutexN.post();
            

            //Vector k = e;
            printf("------------ x %s \n",x.toString().c_str());
            
            return x;
        }
        else{
            return errorVector;
        }
    }break;
    } // end of the switch
}
*/

void wingsTranslatorThread::run() {
    while(!isStopping()) {
        //Vector v = get3dWingsLeft(160,120);
        //printf("resultVector %s \n ", v.toString().c_str());

        Vector plane(4);        // specify the plane in the root reference frame as ax+by+cz+d=0; z=-0.12 in this case
        plane[0]=0.0;           // a
        plane[1]=0.0;           // b
        plane[2]=1.0;           // c
        plane[3]=tableHeight;   // d

        //printf("using tableHeight %f \n", tableHeight);
 
        //Vector x;
        if (plane.length() < 4) {
            fprintf(stdout,"Not enough values given for the projection plane!\n");
            
        }
        
        //printf("defining the point p0 belonging to the plane \n");
        mutexP0.wait();
        p0(3);
        p0.zero();
        
        if (plane[0]!=0.0)
            p0[0]=-plane[3]/plane[0];
        else if (plane[1]!=0.0)
            p0[1]=-plane[3]/plane[1];
        else if (plane[2]!=0.0)
            p0[2]=-plane[3]/plane[2];
        else  {
            fprintf(stdout,"Error while specifying projection plane!\n");
        }
        mutexP0.post();
        
        
        // take a vector orthogonal to the plane
        
        //Vector n(3);
        mutexN.wait();
        n[0]=plane[0];
        n[1]=plane[1];
        n[2]=plane[2];
        mutexN.post();

        //printf("p0 = %s ; n = %s \n", p0.toString().c_str(), n.toString().c_str());
        
        Time::delay(0.1);
    }
    
}

void wingsTranslatorThread::interrupt() {
    outPort.interrupt();
    outRightPort.interrupt();
    inPort.interrupt();
    inRightPort.interrupt();

}

void wingsTranslatorThread::onStop(){
    printf("wingsTranslatorThread::onStop \n");
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
    printf("correctly deleting the client \n");

   
    /* closing the file */   
    /*if(pFile!=NULL) {
        fclose(pFile); 
    }

    if(fout!=NULL) {
        fclose(fout); 
    }
    if(fdebug!=NULL) {
        fclose(fdebug); 
        }*/
    
    if(fmatch!=NULL) {
        fclose(fmatch);
    }
}

