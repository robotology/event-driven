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
#define PI 3.14159265

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




void iCubHeadCenter::allocate(const std::string &_type) {
    // change DH parameters
    (*this)[getN()-2].setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);
}

/********************************************************************/

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
    outPort.open(getName("/coord:o").c_str());
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
    
    headV2 = true;
    eyeL  = new iCubEye(headV2?"left_v2":"left");
    eyeR  = new iCubEye(headV2?"right_v2":"right");
    iCubHeadCenter eyeC("right_v2");
    
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
    isOnWings = false;
    
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
        link->setD(0.034);
        //link->setD(0.0);
        
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

        /***************************************************/
        
        iKinChain* eyeChainR = eyeR->asChain();
        //eyeChain->rmLink(7);
        //eyeChain->rmLink(6); ;
        iKinLink* linkR = &(eyeChainR-> operator ()(5));
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //double a_value = link->getA();
        //printf("a value %f \n", a_value);
        linkR->setD(0.145);
        linkR = &(eyeChainR-> operator ()(6));
        //linkR->setD(0.0);
        linkR->setD(-0.034);
        
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

    // get the absolute reference frame of the head
    
    Vector q(eyeC.getDOF(),0.0);
    eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    // get the lenght of the half of the eyes baseline
    eyesHalfBaseline = 0.5 * norm(eyeL->EndEffPose().subVector(0,2) - eyeR->EndEffPose().subVector(0,2));
    
    
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
   
    blockNeckPitchValue = -1;
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


/************************************************************************/
Vector targetFinderThread::getAbsAngles(const Vector &x) {
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
Vector targetFinderThread::get3DPoint(const string &type, const Vector &ang)
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
    float zDistance = 3.5;
    Bottle* b  = inPort.read(false);
    if(b!=NULL) {
        printf("*************************************************** \n");
        //printf("%s \n", b->toString().c_str());
        for (int i = 0; i < b->size(); i++) {
            valueInput[i] = b->get(i).asDouble();
            //printf(" %f ", valueInput[i]);
        }
        //printf("\n");
        //centerLeftX, centerLeftY, upperLeftX, upperLeftY, lowerLeftX, lowerLeftY
        //centerRightX, centerRightY, upperRightX, upperRightY, lowerRightX, lowerRightY

        u = valueInput[0];
        v = valueInput[1];
        

        //preparing vector of position of left and right image plane
        Vector pxl(2);
        pxl[0] = valueInput[0]; 
        //pxl[0] = 160;
        printf("%f ", valueInput[0]);
        pxl[1] = valueInput[1]; 
        //pxl[1] = 120;
        printf("%f ", valueInput[1]);
        Vector pxr(2);
        pxr[0] = valueInput[6]; 
        //pxr[0] = 160;
        printf("%f ", valueInput[6]);
        pxr[1] = valueInput[7]; 
        //pxr[1] = 120;
        printf("%f ", valueInput[7]);
        printf("\n");

        
        //********************* stereo tringulation component of vision ***************************/
        Vector txl(2), txr(2);
        txl[0]=10;         // specify somehow the pixel within the left image plane
        txl[1]=100;
        
        txr[0]=20;         // specify somehow the pixel within the right image plane
        txr[1]=80;
        Vector xs(3);
        igaze->triangulate3DPoint(pxl,pxr,xs);
        printf("xs: \n"); printf(" %s \n",xs.toString().c_str());  
        

        //********************* stereo component of vision ***************************/

        // reading proprioceptive information
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
        encHead->getEncoder(5,&head[5]);

        printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", torso[0],torso[1],torso[2],head[0],head[1],head[2],head[3],head[4]);
        

        Vector xo, qL(8), qR(8);
        Vector qw(9);
        Vector q(9);

        isOnWings = true;
        if(isOnWings) {
            double ratio = M_PI /180; 
            qw[0]=torso[0] * ratio;
            qw[1]=torso[1] * ratio;
            qw[2]=torso[2] * ratio;
            qw[3]=head[0]  * ratio;
            qw[4]=head[1]  * ratio;
            qw[5]=head[2]  * ratio;
            qw[6]=0.0 * CTRL_DEG2RAD;
            qw[7]=0.0 * CTRL_DEG2RAD;
            qw[8]=0.0 * CTRL_DEG2RAD;   
            double ver = head[5];
            
            qL[0]= qw[0]; //printf("%f \n", qL[0]);
            qL[1]= qw[1]; //printf("%f \n", qL[1]);
            qL[2]= qw[2]; //printf("%f \n", qL[2]);
            qL[3]= qw[3];  //printf("%f \n", qL[3]);
            qL[4]= qw[4];  //printf("%f \n", qL[4]);
            qL[5]= qw[5];  //printf("%f \n", qL[5]);
            qL[6]= qw[6];  //printf("%f \n", qL[6]);
            qL[7]= qw[7] + qw[8] / 2.0; //printf("%f \n", qL[7]);

            qR=qL;
            qR[7] -= qw[8]; //printf("%f \n", qR[7]);
            //xo = yarp::math::operator *(eye->getH(qw),xe);
            //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
        }
        else { 
            double ratio = M_PI /180;
            printf("difference ratioCTRL_DEG2RAD %f \n", CTRL_DEG2RAD - ratio );
            q[0] = torso[0] * CTRL_DEG2RAD;
            q[1] = torso[1] * CTRL_DEG2RAD;
            q[2] = torso[2] * CTRL_DEG2RAD;
            q[3] = head[0]  * ratio;
            q[4] = head[1]  * ratio;
            q[5] = head[2]  * ratio;
            q[6] = head[3]  * ratio;
            q[7] = head[4]  * ratio;
            q[8] = head[5]  * ratio;
            double ver = head[5];
            
            qL[0]= q[0]; //printf("%f \n", qL[0]);
            qL[1]= q[1]; //printf("%f \n", qL[1]);
            qL[2]= q[2]; //printf("%f \n", qL[2]);
            qL[3]= q[3];  //printf("%f \n", qL[3]);
            qL[4]= q[4];  //printf("%f \n", qL[4]);
            qL[5]= q[5];  //printf("%f \n", qL[5]);
            qL[6]= q[6];  //printf("%f \n", qL[6]);
            qL[7]= q[7] + q[8] / 2.0; //printf("%f \n", qL[7]);

            qR=qL;
            qR[7] -= q[8]; //printf("%f \n", qR[7]);
            //xo = yarp::math::operator *(eye->getH(q),xe);
            //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
        }
                 

        //head[3] = 0;
        //head[4] = 0;
        //head[5] = 0;

        // processing information for triangulation
        Matrix HL = SE3inv(eyeL->getH(qL));
        Matrix HR = SE3inv(eyeR->getH(qR));
        //printf("HL : \n"); printf("%s \n", HL.toString().c_str());
        //printf("HR : \n"); printf("%s \n", HR.toString().c_str());        
        
        //printf("inverse of rototraslation matrix 4by4 \n");
        Matrix tmp = zeros(3,4); tmp(2,2) = 1.0;
        tmp(0,2)  = pxl[0]; tmp(1,2) = pxl[1];
        Matrix AL =(*PrjL - tmp) * HL;
        
        tmp(0,2)  = pxr[0]; tmp(1,2) = pxr[1];
        Matrix AR = (*PrjR - tmp) * HR;

        Matrix A(4,3);
        Vector b(4);
        for (int i=0; i<2; i++)
        {
            b[i]   = -AL(i,3);
            b[i+2] = -AR(i,3);

            for (int j=0; j<3; j++)
            {
                A(i,j)   = AL(i,j);
                A(i+2,j) = AR(i,j);
            }
        }

        // solve the least-squares problem
        //printf("Solving the least-squares problem with pseudoinvers \n");
        //printf("A =   \n");
        //printf(" %s  \n", A.toString().c_str());
        //printf("b =   \n");
        //printf(" %s  \n", b.toString().c_str());
        
        Vector x(3);
        Vector xAngles(3);
        x = pinv(A) * b;
        xAngles = getAbsAngles(x);
        printf("x: \n"); printf(" %s \n",x.toString().c_str());  
        //printf("        angles %s \n", xAngles.toString().c_str());
              

        //********************* monocular component of vision ***************************/
        
        bool isLeft = true;  // TODO : the left drive is hardcoded but in the future might be either left or right
        
        Matrix  *invPrj = (isLeft?invPrjL:invPrjR);
        iCubEye *eye = (isLeft?eyeL:eyeR);               
        
        /*
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
        */
        
        Vector x2(3);
        x2[0] = zDistance * u;
        x2[1] = zDistance * v;
        x2[2] = zDistance;        
        
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xe = yarp::math::operator *(*invPrj, x2);
        xe[3] = 1.0;  // impose homogeneous coordinates 
              
        if(isOnWings) {
            xo = yarp::math::operator *(eye->getH(qw),xe);
        }
        else {
            xo = yarp::math::operator *(eye->getH(q),xe);
        }

       

        // update position wrt the root frame        
        //Vector xo = yarp::math::operator *(eye->getH(q),xe);
        printf("xo: \n  %f %f %f \n",xo[0], xo[1], xo[2]);
        //Vector xoAngles = getAbsAngles(xo);
        //printf("        angles %f %f %f \n", xoAngles[0] * 180.0 / PI, xoAngles[1] * 180.0 / PI, xoAngles[2] * 180.0 / PI);
        

        /*
        Vector start(3);
        start[0] = -0.5;
        start[1] = 0.0;
        start[2] = 0.35;
        */

        Vector xoAngles; 
        bool performAction = true;
        if(performAction) {
            igaze->lookAtFixationPoint(xs);
            //igaze->lookAtStereoPixels(pxl, pxr);
            bool done;
            igaze->checkMotionDone(&done);
            double timestart = Time::now();
            double timediff = 0, timestop = 0;
            while ((!done) && (timediff < 5.0)) {
                igaze->checkMotionDone(&done);
                printf(".");
                Time::delay(0.1);
                timestop = Time::now();
                timediff = timestop - timestart;
            }
            printf("\n"); 
            if(timediff >= 5.0) {
                printf("timeout in gaze action \n");
            }
            else {
                printf("success in the gaze action \n");
                igaze->getAngles(xoAngles);
                printf("        angles %f %f %f \n", xoAngles[0] , xoAngles[1], xoAngles[2]);
            }
        }
        
        Time::delay(0.1);
            
        //***********************************************************************/
        
        if(outPort.getOutputCount()) {
            //Vector angleVector(2);
            //igaze->getAngles(angleVector);
            Bottle& angleBottle = outPort.prepare();
            angleBottle.clear();
            angleBottle.add(xoAngles[0]);
            angleBottle.add(xs[0]);
            outPort.write();
        }
    }
}

void targetFinderThread::interrupt() {
    outPort.interrupt();
    outRightPort.interrupt();
    inPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
}

void targetFinderThread::threadRelease() {
    printf("targetFinderThread::threadRelease \n");
    /* closing the ports*/
    outPort.close();
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

