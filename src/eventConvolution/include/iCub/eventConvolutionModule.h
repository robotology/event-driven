// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak, Rea Francesco
 * email:   shashank.pathak@iit.it,francesco.rea@iit.it 
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
 * @file eventConvolutionModule.h
 * @brief Simple module that implements convolution on events received from DVS camera;
 */

#ifndef _EVENT_CONVOLUTION_MODULE_H_
#define _EVENT_CONVOLUTION_MODULE_H_

/** 
 *
 * \defgroup icub_eventConvolution eventConvolution
 * @ingroup icub_logpolarAttention
 *
 * This is a module that applies a given bank of filters (convolutions) on event-train received by DVS camera :
 *
 * - Set up convolution masks and other things
 * - receive event packet from DVS grabber
 * - for each event in that packet apply convolution mask 
 * - send them as a packet
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
 * - \c from \c eventConvolution.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c eventConvolution/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c eventConvolution \n 
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
 *  - \c /eventConvolution \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /eventConvolution
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /eventConvolution/events:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /eventConvolution \n
 *    see above
 *
 *  - \c /eventConvolution/events:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<eventBuffer >   \c myInputPort; \n 
 * \c BufferedPort<eventBuffer >   \c myOutputPort;       
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
 * \c eventConvolution.ini  in \c $ICUB_ROOT/app/eventConvolution/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/eventConvolution/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>eventConvolution --name eventConvolution --context eventConvolution/conf --from eventConvolution.ini --robot icub</tt>
 *
 * \author Rea Francesco, Shashank Pathak
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/eventConvolution/include/iCub/eventConvolution.h
 * 
 */


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/eventConvolutionThread.h>

class eventConvolutionModule:public yarp::os::RFModule
{
   /* module parameters */

   std::string moduleName;
   std::string robotName; 
   std::string robotPortName;  
   std::string inputPortName;
   std::string outputPortName;  
   std::string handlerPortName;
   std::string cameraConfigFilename;

   yarp::os::Port handlerPort;      // a port to handle messages 
   /* pointer to a new thread to be created and started in configure() and stopped in close() */
   eventConvolutionThread *evThread;

public:
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __EVENT_CONVOLUTION_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

