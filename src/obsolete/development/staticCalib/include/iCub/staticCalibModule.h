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
 * @file efExtractorModule.h
 * @brief A module that lauches the eventExtractorThread handling all the user commands
 */

#ifndef _STATIC_CALIB_MODULE_H_
#define _STATIC_CALIB_MODULE_H_

/** 
 *
 * \defgroup icub_staticCalib staticCalib
 * @ingroup icub_eMorph
 *
 *
 * This is a module that receives events and extract features using the policy set by the user
 * If the policy requires a look-up-table to redirect event in the right image the module looks for the map in the conf section of the application
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
 * 
 * \section Description
 * DVS cameras extract event from the scene, these events are represented as images. DVS images coming from the ocular system in the iCub have to be
 * aligned with images produced by traditional cameras (dragonfly). However these cameras are located in a different position in the iCub's head.
 * This module defines the geometry of the alignment and puts together the different stereo images.
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
 * - \c from \c staticCalib.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logPolarAttentionSystem/conf \n
 *   specifies the sub-path from \c /app to the configuration file
 *
 * - \c name \c staticCalib \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c mode \c intensity,horizontal66,vertical66
 *
 * - \c verbose 
 *    indicates if the events must be dumped in debug files
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
 *  - \c /staticCalib \n
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
 *  - \c /staticCalib/dvsLeft:i \n
 *  - \c /staticCalib/dvsRight:i \n
 *  - \c /staticCalib/dragonLeft:i \n
 *  - \c /staticCalib/dragonRight:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /staticCalib \n
 *    see above
 *
 *  - \c /staticCalib/image:o \n
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
 * \c staticCalib.ini  in \c /eMorphApplication/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>staticCalib --name /staticCalib --context logPolarAttentionSystem/conf --from staticCalib.ini --robot icub</tt>
 * 
 * it is stricly important to follow a predefined sequence of acquisition
 *
 * preparing the module
 * 1. connect the  /staticCalib/cam/left:o to a viewer with the click output
 * 2. connect the click output of the viewer to /staticCalib/cam/click:i 
 * 3. type start when you are ready with a rpc command to /staticCalib/cmd
 *
 * use of the module
 * 1. close the opencv image created by the module
 * 2. check whether the module extracted corners autonomously (shown as coloured circles)
 * 3. if not click on the internal corners
 * 4. to analyse the next image close the new opencvimage
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */

/**
* @section change_log CHANGE LOG
*
*/

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

#include "iCub/staticCalibThread.h"

class staticCalibModule:public yarp::os::RFModule 
{
    string moduleName;
    string inputLeftPortName;
    string inputRightPortName;
    string outputPortNameRight;
    string outputPortNameLeft;  
    string handlerPortName;
    string outputCalibPath;

    int thresholdValue;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOut;
    yarp::os::Port handlerPort;

    staticCalibThread *calibThread;

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();                       
    bool close();                                 
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    void createFullPath(const char* path);

};

#endif // _STATIC_CALIB_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------
