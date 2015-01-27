// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email:shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
/**
 * @file dvsCalibratorModule.h
 * @brief Simple module that implements efficient visual feature maps extraction;
 */

#ifndef _CARTESIAN_FRAME_INTEGRATOR_MODULE_H_
#define _CARTESIAN_FRAME_INTEGRATOR_MODULE_H_

/** 
 *
 * \defgroup icub_dvsCalibrator dvsCalibrator
 * @ingroup icub_eMorph
 *
 * This is a module that takes as input the cartesian images (taken from event driven camera), stabilizes them suitably and extracts known (spatially distributed) grid of events to calibrate 
 * one or more DVS cameras:
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
 * - \c from \c dvsCalibrator.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c dvsCalibrator/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c dvsCalibrator \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
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
 *  - \c /dvsCalibrator \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /dvsCalibrator
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /dvsCalibrator/cartesianImageLeft:i \n
 *  - \c /dvsCalibrator/cartesianImageRight:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /dvsCalibrator \n
 *    see above
 *
 *  - \c /dvsCalibrator/cartesianImageLeft:o \n
 *  - \c /dvsCalibrator/cartesianImageRight:o \n 
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
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
 * \c dvsCalibrator.ini  in \c $ICUB_ROOT/app/dvsCalibrator/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/dvsCalibrator/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>dvsCalibrator --name dvsCalibrator --context dvsCalibrator/conf --from dvsCalibrator.ini --robot icub</tt>
 *
 * \author Shashank Pathak
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/dvsCalibrator/include/iCub/dvsCalibrator.h
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
#include <iCub/dvsCalibratorThread.h>


class dvsCalibratorModule:public yarp::os::RFModule
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
   dvsCalibratorThread *dvsCalibThread;
   yarp::os::Semaphore respondLock; // to lock updating through respond 
   

public:
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod(); 
   bool updateModule();
   
};


#endif // __CARTESIAN_FRAME_INTEGRATOR_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

