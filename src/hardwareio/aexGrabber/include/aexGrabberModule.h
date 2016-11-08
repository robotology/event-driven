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

/// \defgroup HardwareIO HardwareIO
/// \defgroup aexGrabber aexGrabber
/// \ingroup HardwareIO
/// \brief pushes events to YARP using the iHead board

#ifndef _AEX_GRABBER_MODULE_H_
#define _AEX_GRABBER_MODULE_H_

#define COMMAND_VOCAB_HELP    VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_NAME    VOCAB4('n','a','m','e')
#define COMMAND_VOCAB_SET     VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET     VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN     VOCAB3('r','u','n')
#define COMMAND_VOCAB_PROG    VOCAB4('p','r','o','g')
#define COMMAND_VOCAB_SUSPEND VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME  VOCAB3('r','e','s')
#define COMMAND_VOCAB_IS      VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED  VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK      VOCAB2('o','k')
#define COMMAND_VOCAB_LEFT    VOCAB4('l','e','f','t')
#define COMMAND_VOCAB_RIGHT   VOCAB4('r','i','g','h')
#define COMMAND_VOCAB_DUMP    VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_OFF     VOCAB3('o','f','f')
#define COMMAND_VOCAB_ON      VOCAB2('o','n')
#define COMMAND_VOCAB_PR      VOCAB2('p','r')
#define COMMAND_VOCAB_FOLL    VOCAB4('f','o','l','l')
#define COMMAND_VOCAB_DIFF    VOCAB3('d','i','f')
#define COMMAND_VOCAB_DIFFON  VOCAB4('d','i','f','n')
#define COMMAND_VOCAB_PUY     VOCAB3('p','u','y')
#define COMMAND_VOCAB_REFR    VOCAB4('r','e','f','r')
#define COMMAND_VOCAB_REQ     VOCAB3('r','e','q')
#define COMMAND_VOCAB_DIFFOFF VOCAB4('d','i','f','f')
#define COMMAND_VOCAB_PUX     VOCAB3('p','u','x')
#define COMMAND_VOCAB_REQPD   VOCAB4('r','e','q','p')
#define COMMAND_VOCAB_INJGND  VOCAB4('i','n','j','g')
#define COMMAND_VOCAB_CAS     VOCAB3('c','a','s')
#define COMMAND_VOCAB_BIAS    VOCAB4('b','i','a','s')
#define COMMAND_VOCAB_SYNC    VOCAB4('s','y','n','c')

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <device2yarp.h>

class aexGrabberModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string binaryName;                     // name of the file containing biases
    std::string binaryNameComplete;             // complete path for the file cointaing biases
    std::string robotPortName;                  // reference to the head of the robot
    std::string deviceName;                     // name of the device
    std::string devicePortName;                 // reference to the device port
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    std::string dumpName;
    std::string dumpNameComplete;
    int ratethread;                             // time constant for ratethread

    yarp::os::Port handlerPort;                 // a port to handle messages
    yarp::os::Semaphore mutex;                  // semaphore for the respond function
    device2yarp* D2Y;                           // reference to the ratethread that reads the dvs camera

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};


#endif // __AEX_GRABBER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

