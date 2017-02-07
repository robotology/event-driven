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
/// \defgroup zynqGrabber zynqGrabber
/// \ingroup HardwareIO
/// \brief flexibile YARP-hardware interface for multiple sensor types

#ifndef _ZYNQ_GRABBER_MODULE_H_
#define _ZYNQ_GRABBER_MODULE_H_

#define COMMAND_VOCAB_HELP    VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_NAME    VOCAB4('n','a','m','e')
#define COMMAND_VOCAB_SUSPEND VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME  VOCAB3('r','e','s')
#define COMMAND_VOCAB_FAILED  VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK      VOCAB2('o','k')
#define COMMAND_VOCAB_GETBIAS VOCAB3('g','e','t') // get biasName <channel>
#define COMMAND_VOCAB_SETBIAS VOCAB3('s','e','t') // set biasName biasValue <channel>
#define COMMAND_VOCAB_PROG    VOCAB4('p','r','o','g')
#define COMMAND_VOCAB_PWROFF  VOCAB3('o','f','f')
#define COMMAND_VOCAB_PWRON   VOCAB2('o','n')
#define COMMAND_VOCAB_RST     VOCAB3('r','s','t')

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

class zynqGrabberModule : public yarp::os::RFModule {

    yarp::os::Port handlerPort; // a port to handle messages

    //HANDLES DEVICE CONFIGURATION
    vDevCtrl vsctrlMngLeft;
    vDevCtrl vsctrlMngRight;

    //HANDLES READING WRITING TO DATA DEVICE AND YARP
    device2yarp D2Y; // ratethread that reads the device and writes to yarp vBottle
    yarp2device Y2D; // bufferedport that reads yarp vBottles and writes to the device

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

};

#endif // __ZYNQ_GRABBER_MODULE_H__
