// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email:shashank.pathak@iit.it
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



/**
 * @file sfCreator.cpp
 * @brief Implementation of the thread (see header sfCreator.h)
 */

#include <iCub/sfCreator.h>


#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace emorph::ecodec;


//#define retinalSize 128

//#define VERBOSE
template<class T>
T absolute(T a, T b)
{
    return a > b? (a-b):(b-a);
}

sfCreator::sfCreator(bool cam)
{
    retinalSize  = 128;  //default value before setting
    cameraType = cam;

    imagePort = new ImageOf<PixelMono>;
    imagePort->resize(retinalSize,retinalSize);
    imagePort->zero();
    unmask_events = new unmask();
    listOfEvents = new AER_struct;

    if(cam)
    {

        if (!outPort.open(getName("/spatialFrame/left:o").c_str()))
        {
            cout <<": unable to open port "  << endl;
            //return false;  // unable to open; let RFModule know so that it won't run
        }
    }
    else
    {
        if (!outPort.open(getName("/spatialFrame/right:o").c_str()))
        {
            cout <<": unable to open port "  << endl;
            //return false;  // unable to open; let RFModule know so that it won't run
        }
    }
    isInitialized = isUpdating = false;
    currentTime = Time::now();
    deltaTimeWindow = .3; // refreshed after this time (secs)
    count =0;

}

sfCreator::~sfCreator()
{
    printf("freeing memory in creator");
    delete listOfEvents;
    delete unmask_events;
    delete imagePort;
    printf("sfCreator release:closing ports \n");
    outPort.close();
    printf("correctly freed memory from the creator \n");

}

void sfCreator::setName(string str)
{
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string sfCreator::getName(const char* p)
{
    string str(name);
    str.append(p);
    return str;
}

void sfCreator::updateMonoImage(ImageOf<yarp::sig::PixelMono>* image, double timeNow )
{
    //assert(image!=0);
    isUpdating = true;
    if(count > 25)   //absolute<double>(timeNow, currentTime)>deltaTimeWindow){
    {
        printf("Refreshing the image since time now is %f \n",absolute<double>(timeNow, currentTime));
        image->zero();
        currentTime = timeNow;
        count = 0;
    }
    //printf("retinalSize in getMonoImage %d \n", retinalSize);
    //image->resize(retinalSize,retinalSize);
    //printf("Updating the image after on read \n");
    unsigned char* pImage = (unsigned char* )image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();

    for (int i=0; i<queueEventsRcvd.size(); ++i)
    {
        // to identify the type of the packet
        // user can rely on the getType() method
        if (queueEventsRcvd[i]->getType()=="AE")
        {
            AddressEvent* ptr=dynamic_cast<AddressEvent*>(queueEventsRcvd[i]);
            if (ptr!=NULL)
            {
                *(pImage+imageRowSize*(ptr->getX())+ ptr->getY()) = min(255, (int)*(pImage+imageRowSize*(ptr->getX())+ ptr->getY())+255 * abs(ptr->getPolarity()));

            }
        }
    }
    printf("\n\n\n");
    //printf("end of the function get in mono \n");
    //unmask_events->setLastTimestamp(0);
    isUpdating = false;
}

void sfCreator::onRead(Bottle& rcvdPackets)
{
    count++;
    if(isAccessing) return;
    //while(isAccessing) ;
    if(outPort.getOutputCount()>0)
    {
        isInitialized = true;
        //printf("onRead ");
        // receives the buffer and saves it
        mutexForRead.wait();
        bool ok=eEvent::decode(rcvdPackets,queueEventsRcvd);
        int dim = queueEventsRcvd.size();
        //int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
        //printf("dim %d \n", dim);
        //printf("---------------------------------------- \n");
        //receivedBufferSize = dim;
        //receivedBuffer = i_ub.get_packet();
        if(!isAccessing)
        {
            updateMonoImage(this->imagePort,Time::now());
        }

        //printf("Status%d dim%d totDim%d overflow%d \n",status,dim,totDim,overflow);
        mutexForRead.post();
    }
    else
    {
        isInitialized = false;
    }
}


void sfCreator::plotMonoImage()
{
    isAccessing = true;

    outPort.prepare() = *imagePort;
    outPort.write();

    isAccessing = false;
}

void sfCreator::getMonoImage(ImageOf<PixelMono>* retImage)
{
    //printf("Getting the Mono image and port count%d\n",outPort.getOutputCount());
    /*if(!(outPort.getOutputCount()>0)){
        retImage->resize(128,128);
        retImage->zero();
        return;
    }*/
    isAccessing = true;
    mutexForGetImage.wait();
    while(isUpdating) {};
    //memcpy(retImage,imagePort->getRawImage(),imagePort->getRawImageSize());
    retImage->copy(*imagePort);
    mutexForGetImage.post();
    isAccessing = false;
    //printf("Copied the Mono image \n");
    //isAccessing = false;


}





//----- end-of-file --- ( next line intentionally left blank ) ------------------

