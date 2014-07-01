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
 * @file objectInteractorModule.h
 * @brief A module that allows the robot to interact with the object in order to extract affordances
 */

#ifndef _OBJECT_INTERACTOR_MODULE_H_
#define _OBJECT_INTERACTOR_MODULE_H_

/** 
 *
 * \defgroup icub_objectInteractor objectInteractor
 * @ingroup icub_eMorph
 *

 * This is a module that allows interaction of the robot with objects present in the reaching space of the robot.
 * 
 * \section Description
 * Interaction with the objects present in the environment is the starting point in order to extract affordances of the objects and eventually 
 * to activate reinforcement learning. The module provides a series of behaviour that can be ground on objects of known position in the world.
 * The behaviours are sent via commands
 The commands sent as bottles to the module port /<modName>/cmd:i
are the following: 
 
(notation: [.] identifies a vocab, <.> specifies a double) 
 
<b>TOUCH</b> 
format: [touch] <x> <y> <z> 
action: a touch is executed on the object placed at (x,y,z) 
cartesian coordinates. These coordinates are previsously 
modified according to the offsets found during kinematic 
calibration. 
 
<b>GRASP</b> 
format: [grasp] <x> <y> <z> 
action: a grasp is executed on the object placed at (x,y,z) 
cartesian coordinates; the object is then lifted and released 
afterwards only in case the grasp was successful. The (x,y,z) 
coordinates are previsously modified according to the offsets 
found during kinematic calibration. 
 
<b>TAP</b> 
format: [tap] <x> <y> <z> 
action: a tap is executed on the object placed at (x,y,z) 
cartesian coordinates. These coordinates are previsously 
modified according to the offsets found during kinematic 
calibration. 
 
<b>CALIB_TABLE</b> 
format: [calib] [table] 
action: the robot will try to find out the table height 
exploiting the contact detection based on force control. If the 
contact is detected then the new table height is sent to the 
eye2world module that is in charge of extracting the 3d 
coordinated of the object from its 2d projection on the image 
plane. The file containing the table information is also updated 
accordingly to avoid calibrating always at start-up. 
 
<b>CALIB_KINEMATIC</b> 
This command is splitted in two consecutive sub-commands: 
 
format subcmd1: [calib] [kin] [start] [left]/[right] <x> <y> <z> 
action: the robot reaches the position (x,y,z) with the 
specified arm and waits for the interaction with human based on 
force control in order to store the corresponding kinematic 
offset; in other words, through this command the user can build 
on-line a map between the 3d input (x,y,z) and the resulting 2d 
quantity (dx,dy) that is required to compensate for the unknown
kinematic offsets that affect the reaching on the table. In this
phase, the user can move the robot arm just by exerting a push 
on it: the quantities acquired during this exploration will be 
used by the tap, grasp, etc. (Note that the compensation is 
achieved only on the position not on the orientation). 
 
format subcmd2: [calib] [kin] [stop] 
action: terminate the calibration phase and update the map. 

\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library. 
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/cmd:i receives a bottle containing commands 
  whose formats are specified in the previous section.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -[status]: returns 1 iff an action is still under way.
    -[track] [on]/[off]: enable/disable the tracking mode of the
     cartesian interface (use with care).
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
 * - \c from \c objectInteractor.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c eMorph/conf \n
 *   specifies the sub-path from \c /app to the configuration file
 *
 * - \c name \c objectInteractor \n 
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
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /objectInteractor \n
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
 *  - \c /objectInteractor/position:o \n
 *
 * <b>Output ports</b>
 *
 *  - \c /objectInteractor \n
 *    see above
 *
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
 * \c objectInteractor.ini  in \c /eMorph/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>objectInteractor --name /objectInteractor --context eMorph/conf --from objectInteractor.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2010 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */


/**
 * CHANGE LOG
 * 27/12/2010 : Added a new layer of ActionPrimitives                                                     @author Rea
 * 28/12/2010 : Added push function in graspThread and ActionPrimitivesLayer3                             @author Rea
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/objectInteractorThread.h>
#include <iCub/graspThread.h>

class oInteractorModule:public yarp::os::RFModule {
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string robotName;                      //name of the robot
    std::string robotPortName;                  //reference to the head of the robot
    std::string handlerPortName;                //name of the handler port (comunication with respond function)
    int ratethread;                             //time constant for ratethread

    yarp::os::Port handlerPort;                 // a port to handle messages 
    oInteractorThread* oiThread;                //objectInteractorThread for processing events
    graspThread* gThread;                       //thread to perform grasping

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // _OBJECT_INTERACTOR_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------
