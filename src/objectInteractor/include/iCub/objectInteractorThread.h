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
 * @file oInteractorThread.h
 * @brief Definition of a thread that allows the robot to perform actions on the environment
 * (see oInteractorModule.h).
 */

#ifndef _OBJECT_INTERACTOR_THREAD_H_
#define _OBJECT_INTERACTOR_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/all.h>
#include <iostream>

#include <yarp/dev/Drivers.h>	// new method
YARP_DECLARE_DEVICES(icubmod)

class oInteractorThread : public yarp::os::RateThread {
private:
    int count;                                  //loop counter of the thread
    std::string name;                           // rootname of all the ports opened by this thread
    std::string robotName;                      // name of the robot
    yarp::dev::PolyDriver* armRobotDevice;      //device necessary to control the robot arm
    yarp::dev::PolyDriver* cartCtrlDevice;      //device necessary to control in the cartesian frame of reference

    yarp::dev::IPositionControl *armPos;        //element that encodes the arm position
    yarp::dev::IEncoders *armEnc;               //element that encodes the encoder values of the arm
    yarp::dev::ICartesianControl *armCart;      //cartesian control of the arm
    
    int jointNum;                               // number of joints controlled

    yarp::os::BufferedPort < yarp::os::Bottle > locationPort;          //port that reads the position of the salient object in the world frame of reference
public:
    /**
    * default constructor
    */
    oInteractorThread();

    /**
     * destructor
     */
    ~oInteractorThread();

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
    * function that set the robotname
    * @param str robotname as a string
    */
    void setRobotName(std::string str);

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

};

#endif  //_OBJECT_INTERACTOR_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
