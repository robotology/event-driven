/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
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

/// \defgroup HardwareIO HardwareIO
/// \defgroup chronocamGrabber chronocamGrabber
/// \ingroup HardwareIO
/// \brief flexibile YARP-hardware interface for multiple sensor types

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

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <yarpInterface.h>
#include <deviceController.h>

class chronocamGrabberModule : public yarp::os::RFModule {

    yarp::os::Port handlerPort; // a port to handle messages

    //HANDLES DEVICE CONFIGURATION
    vDevCtrl vsctrlMng;

    //HANDLES READING WRITING TO DATA DEVICE AND YARP
    device2yarp D2Y; // ratethread that reads the device and writes to yarp vBottle

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

};

#endif // __ZYNQ_GRABBER_MODULE_H__
