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
 * @file targetFinderThread.cpp
 * @brief Implementation of the thread (see header targetFinderThread.h)
 */


#define DIM 10
#define THRATE 30
#define SHIFTCONST 100
#define RETINA_SIZE 128
#define FEATUR_SIZE 32
#define ADDRESS 0x40000
#define X_MASK 0x000000FE
#define X_MASK_DEC    254
#define X_SHIFT 1
#define Y_MASK 0x00007F00
#define Y_MASK_DEC  32512    
#define Y_SHIFT 8
#define POLARITY_MASK 0x00000001
#define POLARITY_SHIFT 0
#define CONST_DECREMENT 5

#define CHUNKSIZE 32768

//#define VERBOSE


#include <iCub/targetFinderThread.h>

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

targetFinderThread::targetFinderThread() : RateThread(THRATE) {
    resized = false;
    count           = 0;
    countEvent      = 0;
    leftInputImage  = 0;
    rightInputImage = 0;
    shiftValue      = 20;

    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);

    

}

targetFinderThread::~targetFinderThread() {
    free(bufferCopy);
}

bool targetFinderThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outLeftPort.open(getName("/edgesLeft:o").c_str());
    outRightPort.open(getName("/edgesRight:o").c_str());
    inPort.open(getName("/coord:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    outEventPort.open(getName("/event:o").c_str());


    int SIZE_OF_EVENT = CHUNKSIZE >> 3; // every event is composed by 4bytes address and 4 bytes timestamp
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);
    
    printf("allocating memory for the LUT \n");
    lut = new int[RETINA_SIZE * RETINA_SIZE * 5];
    printf("initialising memory for LUT \n");

    for(int i = 0; i < RETINA_SIZE * RETINA_SIZE * 5; i++) {
        lut[i] = -1;
        //printf("i: %d  value : %d \n",i,lut[i]);
    }
    printf("successfully initialised memory for LUT \n");



    printf("counted the number of mapping %d \n", countMap);

    
    printf("\n starting the thread.... \n");
    
    eyeL = new iCubEye("left");
    eyeR = new iCubEye("right");    
    
    // remove constraints on the links
    // we use the chains for logging purpose
    //eyeL->setAllConstraints(false);
    //eyeR->setAllConstraints(false);
    
    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);
    
    // if it isOnWings, move the eyes on top of the head 
    isOnWings = true;
    
    if (isOnWings) {
        printf("changing the structure of the chain \n");
        iKinChain* eyeChain = eyeL->asChain();
        //eyeChain->rmLink(7);
        //eyeChain->rmLink(6); ;
        iKinLink* link = &(eyeChain-> operator ()(5));
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //double a_value = link->getA();
        //printf("a value %f \n", a_value);
        link->setD(0.145);
        link = &(eyeChain-> operator ()(6));
        link->setD(0.0);
        //eyeChain->blockLink(6,0.0);
        //eyeChain->blockLink(7,0.0);
        //link = &(eyeChain-> operator ()(6));
        //link->setA(0.0);
        //link->setD(0.034);
        //link->setAlpha(0.0);
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //iKinLink twistLink(0.0,0.034,M_PI/2.0,0.0,-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD);
        //*eyeChain << twistLink;
        //eyeL->releaseLink(6);
        
    }
    else {
        printf("isOnWing false \n");
    }
    
    // get camera projection matrix from the configFile
    printf("trying to extract camera parameter from %s \n", configFile.c_str());
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        double cxl=Prj(0,2);
        double cyl=Prj(1,2);
        printf("center %f,%f \n", cxl, cyl);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR)) {
        Matrix &Prj = *PrjR;
        double cxr=Prj(0,2);
        double cyr=Prj(1,2);
        printf("center %f,%f \n", cxr, cyr);
        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    
    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze/");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());
    
    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;
    
    if (clientGazeCtrl->isValid()) {
        clientGazeCtrl->view(igaze);
    }
    else {
        return false;
    }
        
    igaze->storeContext(&originalContext);
    
    if(blockNeckPitchValue != -1) {
        igaze->blockNeckPitch(blockNeckPitchValue);
        printf("pitch fixed at %f \n",blockNeckPitchValue);
    }
    else {
        printf("pitch free to change\n");
    }
        
    string headPort = "/" + robot + "/head";
    string nameLocal("local");
    
    //initialising the head polydriver
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", "/localhead");
    optionsHead.put("remote", headPort.c_str());
    robotHead = new PolyDriver (optionsHead);
    
    printf("starting the polydrive for the head.... \n");
    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    else {
        printf("succes in starting the polyfrive for the head. \n");
    }
    robotHead->view(encHead);
    
    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    optPolyTorso.put("remote",("/"+robot+"/torso").c_str());
    optPolyTorso.put("local",("/"+nameLocal+"/torso/position").c_str());
    polyTorso = new PolyDriver;
    if (!polyTorso->open(optPolyTorso)) {
        return false;
    }
    else {
        printf("success in starting the polydrive for the torso. \n");
    }
    polyTorso->view(encTorso);
    
    /*opening the file of the mapping and creating the LUT*/
    pFile = fopen (mapURL.c_str(),"rb");    
    fout  = fopen ("lut.txt","w+");
    fdebug  = fopen ("dumpSet.txt","w+");
    
    if (pFile!=NULL) {
        long lSize;
        size_t result;        
        // obtain file size:
        fseek (pFile , 0 , SEEK_END);
        lSize = ftell (pFile);
        printf("dimension of the file %lu \n", lSize);
        rewind(pFile);
        // saving into the buffer
        char * buffer;
        buffer = (char*) malloc (sizeof(char) * lSize);
        //fputs ("fopen example",pFile);
        printf("The file was correctly opened \n");
        result = fread (buffer,1,lSize,pFile);        
        printf(" Saved the data of the file into the buffer lSize:%lu \n", lSize);
        // checking the content of the buffer
        long word = 0;
        long input = -1, output = -1;
        short x, y;
        countMap = 0;
        
        for (int i = 0; i < lSize; i++) {            
            
            int value = convertChar2Dec(*buffer);
            //looking for EOL
            if(*buffer == 10) {
                //sac words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }

                x = word & 0x001F;
                y = word >> 5;
                //printf("sac output: %d --> %d %d \n", word, x, y);
                output = y * 32 + x;
                word = 0;
            }
            //looking for space
            else if(*buffer == 32)  {
                //angel words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }                
                
                x = (word & X_MASK) >> X_SHIFT;
                y = (word & Y_MASK) >> Y_SHIFT;
                //printf("angel input: %d --> %d %d \n", word, x, y);
                input = y * 128 + x;
                word = 0;
            }
            else{
                //printf("%d,%d ", value, word );
                word = word << 4;
                word += value;
            }
            buffer++;
            if((input != -1) && (output!=-1)) {
                
                int inputy  = input / 128;
                int inputx  = input - inputy * 128;
                int outputy = output / 32;
                int outputx = output - outputy * 32;
                
                bool continueSaving = true;
                int i = 0;
                while(continueSaving) {
                    if(lut[input + i * RETINA_SIZE * RETINA_SIZE] != -1) {
                        if(lut[input + i * RETINA_SIZE * RETINA_SIZE ]!= output) { 
                            i++;
                            if (i>= 5) {
                                continueSaving = false;
                            }                                                      
                        }
                        else {
                            continueSaving = false;
                        }
                    }
                    else {
                        //saving
                        lut[input + i * RETINA_SIZE * RETINA_SIZE] = output;
                        printf("lut : %d-->%d (%d) \n", input, output, i);
                        fprintf(fout," %d %d > %d %d > %d %d   \n",input, output, inputy, inputx,  outputy, outputx);
                        input  = -1;
                        output = -1;
                        continueSaving = false;
                        countMap++;
                    }
                } //end of while
            }            
        }

        
        
    }
    
    leftInputImage = new ImageOf<PixelMono>;
    leftInputImage->resize(FEATUR_SIZE, FEATUR_SIZE);
    //initialisation of the memory image
    unsigned char* pLeft = leftInputImage->getRawImage();
    int rowsize = leftInputImage->getRowSize();
    for(int r = 0 ; r < FEATUR_SIZE ; r++){
        for(int c = 0 ; c < FEATUR_SIZE ; c++){
            pLeft[r * rowsize + c] = 127;
        }
    }
    
    printf("initialisation correctly ended \n");
    return true;
}

void targetFinderThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string targetFinderThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void targetFinderThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    //leftInputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(width, height);
    //rightInputImage = new ImageOf<PixelMono>;
    //rightInputImage->resize(width, height);
}

void targetFinderThread::run() {
    int u, v;
    float zDistance = 1.5;
    Bottle* b  = inPort.read(false);
    if(b!=NULL) {
        //printf("b bottle : \n");
        //printf("%s \n", b->toString().c_str());
        for (int i = 0; i < b->size(); i++) {
            valueInput[i] = b->get(i).asDouble();
            //printf(" %f ", valueInput[i]);
        }
        //printf("\n");
        
        Vector pxl(2);
        pxl[0] = 100;
        pxl[1] = 100;
        Vector pxr(2);
        pxr[0] = 100;
        pxr[1] = 100;

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
        

        Vector qL(8);
        qL[0]=torso[0]; printf("%f \n", qL[0]);
        qL[1]=torso[1]; printf("%f \n", qL[1]);
        qL[2]=torso[2]; printf("%f \n", qL[2]);
        qL[3]=head[0];  printf("%f \n", qL[3]);
        qL[4]=head[1];  printf("%f \n", qL[4]);
        qL[5]=head[2];  printf("%f \n", qL[5]);
        qL[6]=head[3];  printf("%f \n", qL[6]);
        qL[7]=head[4]+head[5]/2.0;

        Vector qR=qL;
        qR[7]-=head[5];

        
        Matrix HL=SE3inv(eyeL->getH(qL));
        Matrix HR=SE3inv(eyeR->getH(qR));
        //printf("HR : \n"); printf("%s \n", HR.toString().c_str());
        

        //printf("inverse of rototraslation matrix 4by4 \n");
        Matrix tmp=zeros(3,4); tmp(2,2)=1.0;
        tmp(0,2)=pxl[0]; tmp(1,2)=pxl[1];
        Matrix AL=(*PrjL - tmp) * HL;
        
        //printf("matrixAL created \n");
        tmp(0,2)=pxr[0]; tmp(1,2)=pxr[1];
        Matrix AR=(*PrjR - tmp) * HR;

        //printf("matrixAR created \n");

        Matrix A(4,3);
        Vector b(4);
        for (int i=0; i<2; i++)
        {
            b[i]=-AL(i,3);
            b[i+2]=-AR(i,3);

            for (int j=0; j<3; j++)
            {
                A(i,j)=AL(i,j);
                A(i+2,j)=AR(i,j);
            }
        }

        // solve the least-squares problem
        printf("Solving the least-squares problem with pseudoinvers \n");
        
        Vector x(3);
        x=pinv(A)*b;
        printf("x: \n"); printf(" %s \n",x.toString().c_str());
        
        /*
        bool isLeft = true;  // TODO : the left drive is hardcoded but in the future might be either left or right
        
        Matrix  *invPrj = (isLeft?invPrjL:invPrjR);
        iCubEye *eye = (isLeft?eye:eyeR);               
        
        //function that calculates the 3DPoint where to redirect saccade and add the offset
        Vector torso(3);
        printf("vector torso \n");
        encTorso->getEncoder(0,&torso[0]);
        encTorso->getEncoder(1,&torso[1]);
        encTorso->getEncoder(2,&torso[2]);
        printf("read the torso values \n");
        Vector head(5);
        encHead->getEncoder(0,&head[0]);
        encHead->getEncoder(1,&head[1]);
        encHead->getEncoder(2,&head[2]);
        encHead->getEncoder(3,&head[3]);
        encHead->getEncoder(4,&head[4]);
        printf("reading the head values \n");
        
        
        Vector x(3);
        x[0] = zDistance * u;
        x[1] = zDistance * v;
        x[2] = zDistance;
        printf("assigned the value to the vector x \n");
        
        
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xe = yarp::math::operator *(*invPrj, x);
        xe[3] = 1.0;  // impose homogeneous coordinates 
        printf("imposing homogeneous coordinates \n");
        
        Vector xo;
        if(isOnWings) {
            
            Vector qw(8);
            double ratio = M_PI /180; 
            qw[0]=torso[0] * ratio;
            qw[1]=torso[1] * ratio;
            qw[2]=torso[2] * ratio;
            qw[3]=head[0]  * ratio;
            qw[4]=head[1]  * ratio;
            qw[5]=head[2]  * ratio;
            qw[6]=0.0 * CTRL_DEG2RAD;
            qw[7]=0.0 * CTRL_DEG2RAD;
            
            double ver = head[5];
            xo = yarp::math::operator *(eye->getH(qw),xe);
            //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
        }
        else {    
            
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
            
            xo = yarp::math::operator *(eye->getH(q),xe);
            //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
        }
        
        // update position wrt the root frame        
        //Vector xo = yarp::math::operator *(eye->getH(q),xe);
        printf("fixation point estimated %f %f %f \n",xo[0], xo[1], xo[2]);

        */
        
    }
}

void targetFinderThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    inPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
}

void targetFinderThread::threadRelease() {
    printf("targetFinderThread::threadRelease \n");
    /* closing the ports*/
    outLeftPort.close();
    outRightPort.close();
    inPort.close();
    inRightPort.close();
    outEventPort.close();

 
    delete eyeL;
    delete eyeR;
    igaze->restoreContext(originalContext);
    delete clientGazeCtrl;
   
    /* closing the file */
    delete[] lut;
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

