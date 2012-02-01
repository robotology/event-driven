// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * Public License for more details
 */

/**
 * @file targetFinderThread.h
 * @brief Definition of a thread that receives events and extracts features
 * (see efExtractorModule.h).
 */

#ifndef _TARGET_FINDER_THREAD_H_
#define _TARGET_FINDER_THREAD_H_

#include <iostream>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/emorph/eventConversion.h>
#include <iCub/emorph/eventBuffer.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>



class targetFinderThread : public yarp::os::RateThread {
private:
    bool idle;                          // flag that exclude code from the execution loop
    bool isOnWings;
    int count;                          // loop counter of the thread
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    std::string configFile;             // configuration file of cameras (LEFT RIGHT)
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;                                     // port where the left event image is received
    yarp::os::BufferedPort<yarp::os::Bottle> inRightPort;                                // port where the right event image is received
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outLeftPort;       // port where the output edge (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outRightPort;      // port where the output edge (right) is sent
    yarp::os::BufferedPort<eventBuffer> outEventPort;                                    // port sending events
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftInputImage;                           // image input left 
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightInputImage;                          // image input right 
    std::string robot;                      // name of the robot read by the ResourceFinder
    
    std::string name;                       // rootname of all the ports opened by this thread
    std::string mapURL;                     // mode name and name of the map
    bool resized;                           // flag to check if the variables have been already resized
    int shiftValue;                         // value of the shift between dragonfly (this is vergence related)
    FILE *pFile;                            // file that contains the rules for the LUT
    FILE *fout;                             // file where the extracted LUT is saved
    FILE *fdebug;                           // file for debug
    int* lut;                               // lut that route the event in a different location 
    int monBufSize_b;                       // dimension of the bufferFEA in bytes
    int countEvent;                         // counter of event that are going to be sent
    int countMap;                           // counter of the mapped events

    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::sig::Matrix *invPrjL, *invPrjR;   // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;         // projection matrix
    yarp::os::Property optionsHead;
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    yarp::dev::PolyDriver *polyTorso, *robotHead;   // polydriver for the control of the head
    yarp::dev::IEncoders *encTorso, *encHead;       // measure of the encoder  (head and torso)

    int originalContext;                    // original context for the gaze Controller
    double blockNeckPitchValue;             // value for blocking the pitch of the neck
    double valueInput[12];                  // vector of 12 values read from the input port

    char* bufferCopy;                       // local copy of the events read
    char* flagCopy;                         // copy of the unreadBuffer
    char* resultCopy;                       // buffer resulting out of the selection
    char* buffer;                           // buffer where the events to send are stored

public:
    /**
    * default constructor
    */
    targetFinderThread();

    /**
     * destructor
     */
    ~targetFinderThread();

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
    * function that set operating mode
    * @param str name of the mode
    */
    void setMapURL(std::string str) { mapURL = str; };

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);

    /**
    * function that set the robotname for the ports to which the module connects
    * @param str robotname as a string
    */
    void setRobotName(std::string str) {robot = str; };

    /**
    * function that set the confiFile
    * @param str robotname as a string
    */
    void setConfigFile(std::string str) {configFile = str; };
    
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
    * shift one image with respect to the other 
    * @param shift number of pixel of shifts
    * @param outImage reference to the output image
    */
    void shift(int shift, yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);

};

#endif  //_TARGET_FINDER_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
