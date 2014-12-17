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
/**
 * @file eventConvolutionThread.h
 * @brief Definition of a thread that receives events and convolves them with respective convolution filters
 * (see eventConvolutionModule.h).
 */

#ifndef _EVENT_CONVOLUTION_THREAD_H_
#define _EVENT_CONVOLUTION_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>

// LATER: Remove unnecessary includes
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_fft_complex.h>
#include <gsl/gsl_sort_double.h>
#include <gsl/gsl_statistics.h>

// eMoprh library
#include <iCub/eventBuffer.h>
#include <iCub/eventConversion.h>


#define NBR_KERNELS 12 //NBR_SCALE*NBR_ORIENTATION
#define KERNEL_SIZE 9
#define TYPE_OF_S 2
#define IMAGE_ROW 128   // Nbr of rows ie height
#define IMAGE_COL 128   // Nbr of cols ie width
//#define EVENT_THRESHOLD 2 // threshold value above which kernel-summation will send event to output


typedef float kernelMatrix[NBR_KERNELS][KERNEL_SIZE][KERNEL_SIZE];          // there is a predefined kernel matrix for each scale, each orientation and each polarity
typedef float imageOrient[IMAGE_ROW][IMAGE_COL];                            // image (as matix of float data for each pixel location) for each kernel applied to event train

class eventPort : public yarp::os::BufferedPort<eventBuffer> {
public:
    eventBuffer eventTrain;                       // object made by the train of events
    yarp::os::Semaphore mutex;                    // Semaphore for the flage variable
    bool hasNewEvent;                             // flag that indicates whether there are new events
    virtual void onRead(eventBuffer& eventTrain) {
        if(!hasNewEvent){
            setHasNewEvent(true);             // to be set to false once done with the events
            // receives the buffer and saves it
            this->eventTrain = eventTrain;
        }
    }

    void setHasNewEvent(bool value){
        mutex.wait();
        hasNewEvent = value;
        mutex.post();
    }
};

class eventConvolutionThread : public yarp::os::Thread 
{
private:

    float sum[IMAGE_ROW][IMAGE_COL];        // stores sum of the results of kernel multiplication
    float weightage[NBR_KERNELS];           // weight given to each kernel in summation
    int nbrOfProximityBuckets;
    kernelMatrix* kernelMatrices;           // to store kernel matrix for each orientation and each scale
    imageOrient* outputImages;              // output of convolutions
    
    eventBuffer* inputEventBuffer;          // event train read from DVS camera
    eventBuffer outputEventBuffer;         // event train after convolution and leaky summations
    

    yarp::os::Semaphore onReadLock;         // Lock reading while computing current train. LATER: something better

    unmask* unmasking;                      // to unmask (x,y, polarity) of events


               
    eventPort inputPort;                    // port to read event-train from camera
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono > > outputPort;           // output port for convolved and summed image

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;
    

    

public:
    /**
    * constructor
    */
    eventConvolutionThread();

    /**
     * destructor
     */
    ~eventConvolutionThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * Read the event-trains as they arrive, extract co-ordinates and call applyKernels to each of them
    */
    //virtual void onRead(eventBuffer& b);
    
    /* Simply set the kernels to desired hard-coded values. Note this should actually be reflection about the anchor point for any kernel.
    */ 
    void initializeAllKernels();

    /**
    * To a given address space (with negative or positive polarity) apply all the kernels. LATER: multithreaded
    * @param x the x coordinate of the event within a packet
    * @param y the y-coordinate of the event within a packet
    * @param s polarity of the event (high-to-low or low-to-high)
    */
    void applyAllKernels(short x, short y, short s);

    /**
    * reconstruct event-train by applying the threshold to the sum
    */
    void reconstructEventPacket(); 

    void onReadEvent();   
    
};




#endif  //_EVENT_CONVOLUTION_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

