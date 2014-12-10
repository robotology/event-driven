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
#include <yarp/math/SVD.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace yarp::math;
using namespace std;

#define THRATE 30
#define SHIFTCONST 100
#define VERTSHIFT 34
#define retinalSize 128

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

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;

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

/**********************************************************************************/

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

/**********************************************************************************/

vAlignerThread::vAlignerThread(string _configDragon) : RateThread(THRATE) {
    resized = false;
    count = 0;
    leftDragonImage = 0;
    rightDragonImage = 0;
    leftEventImage = 0;
    rightEventImage = 0;
    shiftValue = 20;
    configDragon =  _configDragon;
}

vAlignerThread::~vAlignerThread() {
    if(leftDragonImage!=0) {
        delete leftDragonImage;
    }
    if(rightDragonImage!=0) {
        delete leftDragonImage;
    }
    if(tmpMono!=0) {
        delete tmp;
    }
    if(tmp!=0) {
        delete tmp;
    }
    if(leftEventImage!=0) {
        delete leftEventImage;
    }
    if(rightDragonImage!=0) {
        delete leftEventImage;
    }    
}

bool vAlignerThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    leftDragonPort.open(getName("/leftDragon:i").c_str());
    rightDragonPort.open(getName("/rightDragon:i").c_str());
    leftEventPort.open(getName("/leftDvs:i").c_str());
    rightEventPort.open(getName("/rightDvs:i").c_str());
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

    //initialising the head polydriver
    printf("starting the polydrive for the head.... \n");
    Property optHead("(device remote_controlboard)");
    string remoteHeadName="/"+robotName+"/head";
    string localHeadName="/"+name+"/head";
    optHead.put("remote",remoteHeadName.c_str());
    optHead.put("local",localHeadName.c_str());
    robotHead =new PolyDriver(optHead);
    if (!robotHead->isValid()) {
        fprintf(stdout,"Head device driver not available!\n");
        delete robotHead;
        return false;
    }
    robotHead->view(encHead);

    //initilisation of the torso
    string torsoPort, headPort;
    Property optPolyTorso("(device remote_controlboard)");
    optPolyTorso.put("remote",("/"+robotName+"/torso").c_str());
    optPolyTorso.put("local",("/"+name+"/torso/position").c_str());
    
    torsoPort = "/" + robotName + "/torso";
    headPort = "/" + robotName + "/head";

    optionsTorso.put("device", "remote_controlboard");
    optionsTorso.put("local", "/localTorso");
    optionsTorso.put("remote", torsoPort.c_str() );

    robotTorso = new PolyDriver(optPolyTorso);

    if (!robotTorso->isValid()) {
        printf("Cannot connect to robot torso\n");
    }
    robotTorso->view(encTorso);
    if ( encTorso==NULL) {
        printf("Cannot get interface to robot torso\n");
        robotTorso->close();
    }
    

    /*
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", "/localhead");
    optionsHead.put("remote", headPort.c_str());

    robotHead = new PolyDriver (optionsHead);

    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    robotHead->view(encHead);
    if (encHead == NULL) {
        printf("cannot get interface to the head\n");
        robotHead->close();
    }
    */

    rightEye = new iCubEye("right");
    leftEye = new iCubEye("left");
    // remove constraints on the links
    // we use the chains for logging purpose
    //leftEye->setAllConstraints(false);
    //rightEye->setAllConstraints(false);


    // release links
    leftEye->releaseLink(0);
    rightEye->releaseLink(0);
    leftEye->releaseLink(1);
    rightEye->releaseLink(1);
    leftEye->releaseLink(2);
    rightEye->releaseLink(2);

    rightEyeDragon = new iCubEye("right");
    leftEyeDragon = new iCubEye("left");
    // remove constraints on the links
    // we use the chains for logging purpose
    leftEyeDragon->setAllConstraints(false);
    rightEyeDragon->setAllConstraints(false);

    // release links
   
    leftEyeDragon->releaseLink(1);
    rightEyeDragon->releaseLink(1);
    leftEyeDragon->releaseLink(2);
    rightEyeDragon->releaseLink(2);

    for (int i = 0; i < 8; i++ ){
        leftEye->releaseLink(i);
        rightEye->releaseLink(i);
        leftEyeDragon->releaseLink(i);
        rightEyeDragon->releaseLink(i);
    }
    // get camera projection matrix for the Dragonfly from the configDragon
    if (getCamPrj(configDragon,"CAMERA_CALIBRATION_LEFT",&PrjLeftDragon)) {
        Matrix &Prj=*PrjLeftDragon;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjLeftDragon=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of left %f %f %f \n", Prj(0,0),Prj(1,1),Prj(2,2));
    }
    if (getCamPrj(configDragon,"CAMERA_CALIBRATION_RIGHT",&PrjRightDragon)) {
        Matrix &Prj=*PrjRightDragon;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjRightDragon=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of right %f %f %f \n", Prj(0,0),Prj(1,1),Prj(2,2));
    }

    // get camera projection matrix for the DVS camera from the configDVS
    if (getCamPrj(configDVS,"CAMERA_CALIBRATION_LEFT",&PrjLeftDVS)) {
        Matrix &Prj=*PrjLeftDVS;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjLeftDVS=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of left %f %f %f \n", Prj(0,0),Prj(1,1),Prj(2,2));
    }
    if (getCamPrj(configDVS,"CAMERA_CALIBRATION_RIGHT",&PrjRightDVS)) {
        Matrix &Prj=*PrjRightDVS;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjRightDVS=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of right %f %f %f \n", Prj(0,0),Prj(1,1),Prj(2,2));
    }
    
    chainRightEye = rightEye->asChain();
    chainLeftEye = leftEye->asChain();
    chainRightEyeDragon = rightEyeDragon->asChain();
    chainLeftEyeDragon = leftEyeDragon->asChain();
    for (int i = 0; i < 3; i++) {
        chainRightEyeDragon->popLink();
        chainLeftEyeDragon->popLink();
    }
    double _A = 0;
    double _D = 0;
    double _Alpha = 0;
    double _Offset = 0;
    double _Min = 0;
    double _Max = 0;
    // adding the 5th link common to left and right
    iKinLink* link = new iKinLink( -54.0,82.5, -M_PI / 2, 90.0, 35.0, 145.0);
    chainRightEyeDragon->pushLink(*link);
    chainLeftEyeDragon->pushLink(*link);
    // adding the 6th link different from the left and right
    iKinLink* leftLink = new iKinLink( 0.0, 34, -M_PI / 2 , 0.0, -35, 15);
    iKinLink* rightLink = new iKinLink( 0.0, -34, -M_PI / 2 , 0.0, -35, 15);
    chainRightEyeDragon->pushLink(*rightLink);
    chainLeftEyeDragon->pushLink(*leftLink);
    // adding the last rotation for the 7th link (different between left and right)
    leftLink = new iKinLink( 0, 0, M_PI / 2, -90, -140, -40);
    rightLink = new iKinLink( 0, 0, M_PI / 2, -90, -140, -40); 
    chainRightEyeDragon->pushLink(*rightLink);
    chainLeftEyeDragon->pushLink(*leftLink);
    return true;
}


void vAlignerThread::setRobotname(string str) {
    robotName = str;
    printf("robotname: %s \n", robotName.c_str());
}

void vAlignerThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
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
    leftEventImage = new ImageOf<PixelMono>;
    leftEventImage->resize(retinalSize, retinalSize);
    rightEventImage = new ImageOf<PixelMono>;
    rightEventImage->resize(retinalSize, retinalSize);
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

        if(leftEventPort.getInputCount()) {
             tmpMono = leftEventPort.read(false);
             if((tmpMono!=0)&&(resized)) {
                 copy_8u_C1R(tmpMono,leftEventImage);
             }
        }

        if(rightEventPort.getInputCount()) {
             tmpMono = rightEventPort.read(false);
             if((tmpMono!=0)&&(resized)) {
                copy_8u_C1R(tmpMono,rightEventImage);
             }
        }
        
        ImageOf<yarp::sig::PixelRgb>& outputImage=outPort.prepare();
        if(resized) {
            Vector angles(3);
            bool b = igaze->getAngles(angles);
            //printf("azim %f, elevation %f, vergence %f \n",angles[0],angles[1],angles[2]);
            double vergence = (angles[2] * 3.14) / 180;
            double version = (angles[0] * 3.14) / 180;
            outputImage.resize(width + shiftValue, height + VERTSHIFT);  
            remap(*leftEventImage, *eventImage,1);
            remap(*rightEventImage, *eventImage,0);
            shift(shiftValue, *eventImage, *rightEventImage, outputImage);
            outPort.write();
        }
    }
}

void vAlignerThread::remap(ImageOf<PixelMono> event,ImageOf<PixelMono> result, bool isLeft) {
    
    Matrix  *invPrj=(isLeft?invPrjLeftDragon:invPrjRightDragon);
    iCubEye *eye=(isLeft?leftEyeDragon:rightEyeDragon);
    
    if (invPrj) {
        int u = 160, v = 120, z = 0.5;
        
        Vector torso(3);
        encTorso->getEncoder(0,&torso[0]);
        encTorso->getEncoder(1,&torso[1]);
        encTorso->getEncoder(2,&torso[2]);
  
        Vector head(5); 
        encHead->getEncoder(0,&head[0]);
        encHead->getEncoder(1,&head[1]);
        encHead->getEncoder(2,&head[2]);
        encHead->getEncoder(3,&head[3]);
        encHead->getEncoder(4,&head[4]);
        
        
        Vector q(8);
        double ratio = M_PI /180;
        q[0]=torso[0] * ratio;
        q[1]=torso[1] * ratio;
        q[2]=torso[2] * ratio;
        q[3]=head[0] * ratio;
        q[4]=head[1] * ratio;
        q[5]=head[2] * ratio;
        q[6]=head[3] * ratio;  
        q[7]=head[4] * ratio;
        
        Vector x(3);
        x[0]=z * u;
        x[1]=z * v;
        x[2]=z;
            
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates                
        
        // update position wrt the root frame
        Vector xo = yarp::math::operator *(eye->getH(q),xe);
            
        //fp.resize(3,0.0);
        //fp[0]=xo[0];
        //fp[1]=xo[1];
        //fp[2]=xo[2];
        printf("object left eye %f,%f,%f \n",xo[0],xo[1],xo[2]);
    }
    
}

void vAlignerThread::shift(int shift,ImageOf<PixelMono> leftEvent,
                           ImageOf<PixelMono> rightEvent, ImageOf<PixelRgb>& outImage) {
    unsigned char* pRightDragon = rightDragonImage->getRawImage();
    unsigned char* pLeftDragon = leftDragonImage->getRawImage();
    unsigned char* pRightEvent = rightEventImage->getRawImage();
    unsigned char* pLeftEvent = leftEventImage->getRawImage();
    
    int padding = leftDragonImage->getPadding();
    int paddingEvent = leftEventImage->getPadding();
    printf("paddingEvent %d \n", paddingEvent);
    int paddingOut = outImage.getPadding();
    
    unsigned char* pOutput=outImage.getRawImage();
    double centerX=width + shift;
    double centerY=height;
    if(shift >= 0) {
        int row = 0;
        while (row < height + VERTSHIFT) {
            if(row < VERTSHIFT ){
                int col = 0;
                while(col < width + shift) {
                    if(col < width) {
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                    }
                    else if(col >= width) {
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                            
                    }
                    col++;
                }
                //padding
                pLeftDragon += padding;
                pOutput += paddingOut;
            }
            
            else if((row >= VERTSHIFT)&&(row < height)) {
                int col = 0;
                if ((row > 120 - 64) && (row < 120 + 64)) {
                    eventLeft = true;
                }
                else {
                    eventLeft = false;
                }
                    
                while(col < width + shift) {
                    //pRight += shift*3;
                    if (col < shift) {
                        //d=sqrt( (row - centerY) * (row - centerY) 
                        //    + (col - centerX) * (col - centerX));
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                    }
                    else if((col >= shift)&&(col < width)) {
                        //d=sqrt( (row - centerY) * (row - centerY) 
                        //    + (col - centerX) * (col - centerX));
                        if((col >= 160 - 64)&&(col < 160 + 64) && (row >= 120 - 64) && (row < 120 + 64)) {
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.25 + *pRightDragon *0.25 + *pLeftEvent *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++; 
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.25 + *pRightDragon *0.25 + *pLeftEvent *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.25 + *pRightDragon *0.25 + *pLeftEvent *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            pLeftEvent++;
                        }
                        else {
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++;pRightDragon++;pOutput++;
                        }
                    }
                    else if((col >= width)&&(col < width + shift)){
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                    }
                    col++;
                }
                //padding
                if(eventLeft) {
                    pLeftEvent += paddingEvent;
                }
                pLeftDragon += padding;
                pRightDragon += padding;
                pOutput += paddingOut;
            }            
            else if ((row >= height)&&(row < height + VERTSHIFT)) {
                int col = 0;

                while(col < width + shift) {
                    if(col < shift ) {
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                    }
                    else {
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                    }
                    col++;
                }                
                //padding
                pRightDragon += padding;
                pOutput += paddingOut;
            }
            row ++;
        }
    }
}


void vAlignerThread::interrupt() {
    outPort.interrupt();
    leftEventPort.interrupt();
    rightEventPort.interrupt();
    leftDragonPort.interrupt();
    rightDragonPort.interrupt();
    vergencePort.interrupt();
}

void vAlignerThread::threadRelease() {
    delete clientGazeCtrl;
    outPort.close();
    leftEventPort.close();
    rightEventPort.close();
    leftDragonPort.close();
    rightDragonPort.close();
    vergencePort.close();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------


