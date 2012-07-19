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
 * @file plotterThread.h
 * @brief Definition of a thread that sends frame-based representation of the events
 * (see plotterthread.h).
 */

#ifndef _PLOTTER_THREAD_H_
#define _PLOTTER_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <iostream>

#define NUMANGLES 360
#define PI        3.1415

class plotterThread : public yarp::os::RateThread {
private:
    bool maxReached;                      // indicates when a new max is present in the memory
    
    int count;                            // loop counter of the thread
    int velWTA_direction;                 // direction of the main velocity component in fovea
    int retinalSize;                      // dimension of the squared retina
    int histoValue[NUMANGLES];                  // value of the representation of the histogram
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > histoPort;                 // port whre the output is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > velocPortIn;                // port whre the output (right) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > velocPortOut;                // port whre the output (right) is
    
    
    yarp::os::BufferedPort<yarp::sig::Vector > eventPort;
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imageHisto;                                       //image representing the histogram of angles
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageVelocIn;                                      //image representing the velocities
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imageVelocOut;                                      //image representing the velocities
    
    std::string name;                           // rootname of all the ports opened by this thread
    bool synchronised;                          // flag to check whether the microsecond counter has been synchronised
    bool stereo;                                // flag indicating the stereo characteristic of the synchronization
    bool firstInputImage;                       // flag that indicates the first image received

    yarp::os::Semaphore mutexHisto;             // semaphore for the mutex
    yarp::os::Semaphore mutexVeloc;             // semaphore for the selected velocity
    
public:
    /**
    * default constructor
    */
    plotterThread();

    /**
     * destructor
     */
    ~plotterThread();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);

    /**
    * function that set the stereo mode for event synchronization
    * @param value boolean to be assigned
    */
    void setStereo(bool value){ stereo = value;};    
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

    /**
     * function that copies the image in the left output
     * @param img passed input of the image to be copied
     */
    void copyLeft(yarp::sig::ImageOf<yarp::sig::PixelMono>* img);

    /**
     * function that copies the RGB image in the left output 
     * @param img passed input of the image to be copied
     */
    void copyLeft(yarp::sig::ImageOf<yarp::sig::PixelRgb>* img);

    /**
     * function that copies the RGB image in the right output
     * @param img passed input of the image to be copied
     */
    void copyRight(yarp::sig::ImageOf<yarp::sig::PixelMono>* img);

    /**
     * function that copies the RGB image in the right output
     * @param img passed input of the image to be copied
     */
    void copyRight(yarp::sig::ImageOf<yarp::sig::PixelRgb>* img);

    /**
    * function that integrates the current dvs image with previous images
    * @param imgIn image of the current dvs camera
    * @param output of the process
    * @param imageBW black and white image
    */
    int integrateImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* imgIn,  yarp::sig::ImageOf<yarp::sig::PixelMono>* imgOut,
                       yarp::sig::ImageOf<yarp::sig::PixelMono>* imgBW, yarp::sig::ImageOf<yarp::sig::PixelMono>* imgGray,  
                       yarp::sig::ImageOf<yarp::sig::PixelMono>* imgTh );
    
    /**
     * @brief function thatset the dimension of the output image
     * @param value the dimension in pixels of the retina device
     */
    void setRetinalSize(int value) {
        retinalSize = value;
    }

    /**
     * @brief function that sets the histogram value
     */
    void setHistoValue(int* hValuePointer);

    /**
     * @brief function that sets the velocity selected
     * @param angle angle of the wta velocity profile
     * @param magnitude magnitude of the wta velocity profile
     */
    void setVelResult(int angle, float magnitude, bool maxreached);
    
    /**
     * @brief image for preparing the histogram of the angle
     */
    void prepareHistoImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& out) ;
    
    /**
     * @brief image for preparing the image of the velocities
     */
    void prepareVelocImage(yarp::sig::ImageOf<yarp::sig::PixelMono> in, yarp::sig::ImageOf<yarp::sig::PixelRgb>& out);

};

#endif  //_PLOTTER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

