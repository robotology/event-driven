// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file aexGrabbberModule.h
 * @brief A module that extracts independent event-driven responses coming from AEX event-based camera
 */

#ifndef _ASV_GRABBER_MODULE_H_
#define _ASV_GRABBER_MODULE_H_

/** 
 *
 * \defgroup icub_asvGrabber asvGrabber
 * @ingroup icub_eMorph
 *
 *
 * This is a module that extracts independent event-driven response to changes in the luminance sensed 
 * 
 * Event driven asynchronous sensors transmit the local pixel-level changes caused by movement in a scene at the time they occur.
 * The result is a stream of events at microsecond time resolution with very low redundancy that drastically reduces power, data storage and computational requirements.
 * 
 * \section reference
 * The address-event representation communication protocol AER 0.02, Caltech, Pasadena, CA, Internal Memo, Feb. 1993 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/scx/std002.pdf
 * 
 * S. R. Deiss, T. Delbrück, R. J. Douglas, M. Fischer, M. Mahowald, T. Matthews, and A. M. Whatley, Address-event asynchronous local broadcast protocol, Inst. Neuroinform., Zurich, Switzerland, 1994 [Online].
 * Available: http://www.ini.uzh.ch/~amw/scx/aeprotocol.html
 * 
 * A. M. Whatley, PCI-AER Board Driver, Library & Documentation, Inst. Neuroinform., Zurich, Switzerland, 2007 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/pciaer/
 * 
 * S. R. Deiss, R. J. Douglas, and A. M. Whatley, "A pulse-coded communications infrastructure for neuromorphic systems", in Pulsed Neural Networks, W. Maass and C. M. Bishop, Eds. Cambridge, MA: MIT Press, 1998, ch. 6, pp. 157–178.
 * 
 * V. Dante, P. Del Giudice, and A. M. Whatley, “PCI-AER—hardware and software for interfacing to address-event based neuromorphic systems,” The Neuromorphic Engineer vol. 2, no. 1, pp.
 * 5–6, 2005 [Online]. Available: http://ine-web.org/research/newsletters/index.html
 * 
 *
 * \section Description
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c asvGrabber.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c asvGrabber/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c asvGrabber \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /asvGrabber \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /visualFilter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /asvGrabber/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /asvGrabber \n
 *    see above
 *
 *  - \c /asvGrabber/image:o \n
 *
 * <b>Port types</b>
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c asvGrabber.ini  in \c $ICUB_ROOT/app/asvGrabber/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>asvGrabber --name asvGrabber --context asvGrabber/conf --from asvGrabber.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/eMorph/asvGrabber/include/iCub/asvGrabberModule.h
 * 
 */
#define COMMAND_VOCAB_SYTH    VOCAB4('s','y','t','h')
#define COMMAND_VOCAB_SYTA    VOCAB4('s','y','t','a')
#define COMMAND_VOCAB_SYPA    VOCAB4('s','y','p','a')
#define COMMAND_VOCAB_SYPH    VOCAB4('s','y','p','h')
#define COMMAND_VOCAB_TPB     VOCAB3('t','p','b')
#define COMMAND_VOCAB_CDR     VOCAB3('c','d','r')
#define COMMAND_VOCAB_CDS     VOCAB3('c','d','s')
#define COMMAND_VOCAB_CDP     VOCAB3('c','d','p')
#define COMMAND_VOCAB_RPX     VOCAB3('r','p','x')
#define COMMAND_VOCAB_RPY     VOCAB3('r','p','y') 
#define COMMAND_VOCAB_IFR     VOCAB3('i','f','r')
#define COMMAND_VOCAB_IFT     VOCAB3('i','f','t')
#define COMMAND_VOCAB_IFL     VOCAB3('i','f','l'    )
#define COMMAND_VOCAB_CDOF    VOCAB4('c','d','o','f')
#define COMMAND_VOCAB_SYPW    VOCAB4('s','y','p','w')
#define COMMAND_VOCAB_SYW     VOCAB3('s','y','w')
#define COMMAND_VOCAB_CDON    VOCAB4('c','d','o','n')
#define COMMAND_VOCAB_CDD     VOCAB3('c','d','d')
#define COMMAND_VOCAB_EMCH    VOCAB4('e','m','c','h')
#define COMMAND_VOCAB_EMCT    VOCAB4('e','m','c','t')
#define COMMAND_VOCAB_CDI     VOCAB3('c','d','i')
#define COMMAND_VOCAB_CDRG    VOCAB4('c','d','r','g')
#define COMMAND_VOCAB_SELF    VOCAB4('s','e','l','f')
#define COMMAND_VOCAB_FOLL    VOCAB4('f','o','l','l')
#define COMMAND_VOCAB_ARBP    VOCAB4('a','r','b','p')
#define COMMAND_VOCAB_EMVL    VOCAB4('e','m','v','l')
#define COMMAND_VOCAB_CDC     VOCAB3('c','d','c')
#define COMMAND_VOCAB_EMVH    VOCAB4('e','m','v','h')
#define COMMAND_VOCAB_I2V     VOCAB3('i','2','v')

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


#define COMMAND_VOCAB_BIAS VOCAB4('b','i','a','s')


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/asvGrabberThread.h>

class asvGrabberModule:public yarp::os::RFModule {
    std::string moduleName;                       // name of the module (rootname of ports)
    std::string robotName;                        // name of the robot
    std::string binaryName;                       // name of the file containing biases
    std::string binaryNameComplete;               // complete path for the file cointaing biases
    std::string robotPortName;                    // reference to the head of the robot
    std::string deviceName;                       // name of the device
    std::string devicePortName;                   // reference to the device port
    std::string handlerPortName;                  // name of the handler port (comunication with respond function)
    std::string dumpName;
    std::string dumpNameComplete;
    int ratethread;                               // time constant for ratethread

    yarp::os::Port handlerPort;                   // a port to handle messages 
    yarp::os::Semaphore mutex;                    // semaphore for the respond function
    asvGrabberThread* D2Y;                        // reference to the ratethread that reads the dvs camera

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __ASV_GRABBER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

