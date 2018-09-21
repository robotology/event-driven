/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// \defgroup HardwareIO HardwareIO
// \defgroup zynqGrabber zynqGrabber
// \ingroup HardwareIO
// \brief flexibile YARP-hardware interface for multiple sensor types

#ifndef _ZYNQ_GRABBER_MODULE_H_
#define _ZYNQ_GRABBER_MODULE_H_

#define COMMAND_VOCAB_HELP    createVocab('h','e','l','p')
#define COMMAND_VOCAB_NAME    createVocab('n','a','m','e')
#define COMMAND_VOCAB_SUSPEND createVocab('s','u','s')
#define COMMAND_VOCAB_RESUME  createVocab('r','e','s')
#define COMMAND_VOCAB_FAILED  createVocab('f','a','i','l')
#define COMMAND_VOCAB_OK      createVocab('o','k')
#define COMMAND_VOCAB_GETBIAS createVocab('g','e','t') // get biasName <channel>
#define COMMAND_VOCAB_SETBIAS createVocab('s','e','t') // set biasName biasValue <channel>
#define COMMAND_VOCAB_PROG    createVocab('p','r','o','g')
#define COMMAND_VOCAB_PWROFF  createVocab('o','f','f')
#define COMMAND_VOCAB_PWRON   createVocab('o','n')
#define COMMAND_VOCAB_RST     createVocab('r','s','t')
#define COMMAND_VOCAB_SETSKIN createVocab('s','s','e','t') // set regName regValue



#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include "yarpInterface.h"
#include "visionController.h"
#include "skinController.h"

class zynqGrabberModule : public yarp::os::RFModule {

    yarp::os::Port handlerPort; // a port to handle messages

    //HANDLES DEVICE CONFIGURATION
    vVisionCtrl vsctrlMngLeft;
    vVisionCtrl vsctrlMngRight;
    vSkinCtrl   skctrlMng;

    hpuInterface hpu;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

};

#endif // __ZYNQ_GRABBER_MODULE_H__
