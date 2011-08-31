// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email: shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/


#include <iCub/eventConvolutionThread.h>
#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781
#define ONE_BY_ROOT_THREE 0.577350269

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;




eventConvolutionThread::eventConvolutionThread() {

    kernelMatrices      = new  kernelMatrix[TYPE_OF_S];
    outputImages        = new imageOrient[NBR_KERNELS];
    inputEventBuffer    = new eventBuffer;
    unmasking           = new unmask;
    
    initializeAllKernels();    
}

eventConvolutionThread::~eventConvolutionThread() {    
    delete kernelMatrices;
    delete outputImages;
    delete inputEventBuffer;
    delete unmasking;       
}

bool eventConvolutionThread::threadInit() {
    printf("opening ports \n");
    /* open ports */ 
    
    inputPort.useCallback();
    if (!inputPort.open(getName("/events:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/eventsConvolved:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    return true;
}

void eventConvolutionThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string eventConvolutionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void eventConvolutionThread::run() {
   
    
   while (isStopping() != true) {           
        if(inputPort.hasNewEvent) {
           
           inputEventBuffer = &inputPort.eventTrain; // shallow copy
            onReadEvent();          
            //if((outputPort.getOutputCount())) {
                //outputPort.prepare() = *(intensImg);
                //outputPort.write();
            //}
            inputPort.setHasNewEvent(false);
        }            
        
   }
}

void eventConvolutionThread::onReadEvent(){
    
    onReadLock.wait();          // wait till done with current read
    // start of CS
    {
        short x,y,s,dummyCamera;                  // unmask an event and get (x,y) co-ordinates as well as polarity 's'
        int dim = inputEventBuffer->get_sizeOfPacket() ;      // number of bits received / 8 = bytes received              
        char* ptrBufferData = inputEventBuffer->get_packet();       
        long int blob;
        unsigned int timestamp, currentTimestamp;
                
        for (int i = 0 ; i < dim ; i+=4) {
            unsigned int part_1 = 0xFF & (*ptrBufferData+i);      //extracting the 1 byte        
            unsigned int part_2 = 0xFF & (*(ptrBufferData+i+1));  //extracting the 2 byte        
            unsigned int part_3 = 0xFF & (*(ptrBufferData+i+2));  //extracting the 3 byte
            unsigned int part_4 = 0xFF & (*(ptrBufferData+i+3));  //extracting the 4 byte
            blob      = (part_1)|(part_2<<8);          //16bits
            timestamp = ((part_3)|(part_4<<8));        //16bits
            unmasking->unmaskEvent(blob,x,y,s, dummyCamera);
            applyAllKernels(x,y,s);             
                
        }
        printf("Out of the packet \n");
        reconstructEventPacket();
    }// end of CS
    onReadLock.post();
}

void eventConvolutionThread::initializeAllKernels(){

    for(int i=0; i<NBR_KERNELS; ++i){
        for(int j=0; j<KERNEL_SIZE; ++j){
            for(int k=0; k<KERNEL_SIZE; ++k){
                kernelMatrices[0][i][j][k] = 1/81.0;
                kernelMatrices[1][i][j][k] = -1/81.0;
            }
        }
    }

    
}

void eventConvolutionThread::applyAllKernels(short i, short j, short s){

        // current pixel point is anchor
        int eff_wd = min(IMAGE_ROW, i + KERNEL_SIZE/2)- max(0,i-KERNEL_SIZE/2)+1;
        int eff_ht = min(IMAGE_COL,j+KERNEL_SIZE/2)-max(0,j-KERNEL_SIZE/2)+1;
        int polarityOfKernel = s;
        int kernelStartX = KERNEL_SIZE - eff_wd;
        int kernelStartY = KERNEL_SIZE - eff_ht;
        int imageStartX = max(0,i-KERNEL_SIZE/2);
        int imageStartY = max(0,j-KERNEL_SIZE/2);
        
        for(int currentKernel=0; currentKernel<NBR_KERNELS; ++currentKernel){
        
            for(int k=kernelStartX; k<KERNEL_SIZE;++k,++imageStartX){
                for(int l=kernelStartY; l<KERNEL_SIZE;++l,++imageStartY){
                    outputImages[currentKernel][imageStartX][imageStartY] = kernelMatrices[polarityOfKernel][currentKernel][k][l];                   
                }                
            }
            
        }

}

void eventConvolutionThread::reconstructEventPacket(){
        // We have the convolved data matrices now. Hence, #kernels different packets of event based on some threshold might be calculated. The logic goes here.
}

void eventConvolutionThread::threadRelease() {
   
    printf("Release complete!\n");
    
    
}

void eventConvolutionThread::onStop() {

    printf("Stopping the thread ....\n");
    outputPort.interrupt();
    outputPort.close();
    inputPort.interrupt();
    inputPort.close();
    printf("Done with stopping the thread.\n");
    
}


