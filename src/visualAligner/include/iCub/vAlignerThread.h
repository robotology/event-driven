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
 * Public License for more details
 */

/**
 * @file vAlignerThread.h
 * @brief Definition of a thread that receive cameres information and align them all
 * (see vAlignerModule.h).
 */

#ifndef _VISUAL_ALIGNER_THREAD_H_
#define _VISUAL_ALIGNER_THREAD_H_

#include <yarp/os/RateThread.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>

#include <iCub/utils.h>

using namespace iCub::ctrl;

#include <iostream>



class vAlignerThread : public yarp::os::RateThread {
private:
    int count;                                                                              // loop counter of the thread
    int width, height;                                                                      // dimension of the extended input image (extending)
    int height_orig, width_orig;                                                            // original dimension of the input and output images
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelRgb> > leftDragonPort;       // port where the output of the dragonfly left is read
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelRgb> > rightDragonPort;      // port where the output of the dragonfly right is read
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelMono> > leftEventPort;       // port where the output of the event camera left is read
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelMono> > rightEventPort;      // port where the output of the event camera right is read
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort;               // port whre the output is sent
    yarp::os::BufferedPort<yarp::os::Bottle > vergencePort;                                 // port where the value of the vergence is received
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* leftDragonImage;                              // image output of the dragonfly left is saved
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* rightDragonImage;                             // image where the output of the dragonfly right is saved
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftEventImage;                              // image where the output of the event image left is saved
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightEventImage;                             // image where the output of the event image right is saved
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* tmp;                                          // temporary image for correct port reading
    yarp::sig::ImageOf <yarp::sig::PixelMono>* tmpMono;
    std::string name;                                                                       // rootname of all the ports opened by this thread
    std::string configFile;
    bool resized;                                                                           // flag to check if the variables have been already resized
    bool eventLeft;                                                                         // flag for the presence of the left event image in the image
    int shiftValue;                                                                         // value of the shift between dragonfly (this is vergence related)
    yarp::dev::IGazeControl* igaze;                                                         // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;                                                  // polydriver for the gaze controller
    iCub::iKin::iCubEye *leftEye, *rightEye;                                                // reference to the eye kinematics
    yarp::dev::PolyDriver *robotHead,*robotTorso;                                           // driver for the torso
    iCub::iKin::iKinLink *leftLink, *rightLink;                                             // ikinLink of the left and right eye
    iCub::iKin::iKinChain *chainRightEye,  *chainLeftEye;                                   // ikinChain of the left and right eye
    yarp::os::Property optionsHead, optionsTorso;                                           // option for the torso
    yarp::dev::IEncoders *encHead, *encTorso;                                               // encoder for the torso
    yarp::sig::Matrix *invPrjL, *invPrjR;                                                   // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;                                                         // projection matrix
    std::string robotName;                                                                  // name of the robot
public:
    /**
    * default constructor
    * @param _configFile name of the file of eyes configuraration
    */
    vAlignerThread(std::string _configFile);

    /**
     * destructor
     */
    ~vAlignerThread();

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
    * function that set the robotname for the ports that will be opened
    * @param str robotname as a string
    */
    void setRobotname(std::string str);
    
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
    * @param leftEvent image from the leftDvSCamera
    * @param rightEvent image from the right DVS Camera
    * @param outImage reference to the output image
    */
    void shift(int shift, yarp::sig::ImageOf<yarp::sig::PixelMono> leftEvent,yarp::sig::ImageOf<yarp::sig::PixelMono> right, yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);

    

};

#endif  //_VISUAL_ALIGNER_THREAD_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
