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
 * @file wingsTranslatorModule.h
 * @brief A module that lauches the eventExtractorThread handling all the user commands
 */

#ifndef _WINGS_TRANSLATOR_MODULE_H_
#define _WINGS_TRANSLATOR_MODULE_H_

/** 
 *
 * \defgroup icub_wingsTranslator wingsTranslator
 * @ingroup icub_eMorph
 *
 *
 * This is a module that given a set of position for the two eyes, calculates the 3d position of the object seen from wing cameras
* 
 * 
 * 
 * \section Description
 * Traditional cameras extract position of a common object in the image plane. Given the kinamatic chain of wing cameras the module extract
 * the 3d position  of the object
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
 * - \c from \c wingsTranslator.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logPolarAttentionSystem/conf \n
 *   specifies the sub-path from \c /app to the configuration file
 *
 * - \c name \c wingsTranslator \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c mode \c intensity,horizontal66,vertical66
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
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
 *  - \c /wingsTranslator \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /cartesianFrameCollector
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *
 * <b>Output ports</b>
 *
 *  - \c /wingsTranslator \n
 *    see above
 *
 *  - \c /wingsTranslator/image:o \n
 *
 * <b>Port types</b>
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
 * \c wingsTranslator.ini  in \c /eMorphApplication/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>wingsTranslator --name /wingsTranslator --context logPolarAttentionSystem/conf --from wingsTranslator.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2012 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */

/**
* /section change_log CHANGE LOG
* 01/02/12 : created the module                                                                       author: Rea \n
*/
#define COMMAND_VOCAB_FAILED        VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_HELP          VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_QUIT          VOCAB4('q','u','i','t')
#define COMMAND_VOCAB_SUSPEND       VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME        VOCAB3('r','e','s')
#define COMMAND_VOCAB_OK            VOCAB2('o','k')
#define COMMAND_VOCAB_GET3D         VOCAB4('g','e','t','3')

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/wingsTranslatorThread.h>

class wingsTranslatorModule:public yarp::os::RFModule {
    bool        isOnWings;                      // indicates whether the camera is installed on the wings
    
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    std::string mapName;                        // name of the operating mode corresponds to the map
    std::string mapNameComplete;                // name of complete of the map 
    std::string configName;                     // name of the configuration file for camera 
    std::string configFile;                     // configuration file of cameras (LEFT RIGHT)
    std::string wingsLeftFile;                  // kinematic configuration file for left camera
    std::string wingsRightFile;                  // kinematic configuration file for left camera
    std::string wingsLeftName;                  // name of the kinematic configuration of the 
    std::string wingsRightName;                  // name of the kinematic configuration of the 
    
    double tableHeight;                          // height z-axis for the plane in homography
    int ratethread;                             // time constant for ratethread
    yarp::os::Semaphore respondLock;            // to lock updating through respond
    yarp::os::RpcServer handlerPort;            // a port to handle messages 
    wingsTranslatorThread* tf;                  // wingsTranslatorThread for processing events

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool updateModule();
};


#endif // _WINGS_TRANSLATOR_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------
