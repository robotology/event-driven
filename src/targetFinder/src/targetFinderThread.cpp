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
    inLeftPort.open(getName("/left:i").c_str());
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
        printf("counted the number of mapping %d \n", countMap);


        printf("starting the thread.... \n");
        
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
        bool isOnWings = true;
        
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
        if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
            Matrix &Prj = *PrjL;
            //cxl=Prj(0,2);
            //cyl=Prj(1,2);
            invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
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
        
        if (!robotHead->isValid()){
            printf("cannot connect to robot head\n");
        }
        robotHead->view(encHead);
        
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
    
}

void targetFinderThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    inLeftPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
}

void targetFinderThread::threadRelease() {
    /* closing the ports*/
    outLeftPort.close();
    outRightPort.close();
    inLeftPort.close();
    inRightPort.close();
    outEventPort.close();

    
    /* closing the file */
    delete[] lut;
    fclose (pFile);
}

