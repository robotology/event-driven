/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
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
 * @file bottleRepeaterModule.h
 * @brief A module that read independent asynchronous events from a yarp port and represents them as an image
 */
#ifndef TSOPTFLOW_HPP
#define TSOPTFLOW_HPP

/** 
 *
 * \defgroup icub_tsOptFlow tsOptFlow
 * @ingroup icub_eMorph
 *
 *
 * This is a module which compute the optical flow given the address and timestamp of the events, output of the event-driven camera of the eyes.
 * 
 * 
 * \section reference
 * 
 * 
 *
 * \section Description
 *
 * \section lib_sec Libraries
 *
 * YARP.
 * EMORPHLIB.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c name \c tsOptFlow \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c height \c 128\n
 *   specifies the height of the cartesian representation
 *
 * - \c width \c 128 \n
 *   specifies the width of the cartesian representation
 *
 * - \c source \c {icub, dvs} \n
 *   specifies the format in input of the module
 *
 * - \c type \c {4, 6, 8} \n
 *   specifies the format of a dvs input  (to specify only if the source is dvs)

 * - \c acc \c 20000 \n
 *   specifies the accumulation duration of the computed optical flow vectors before sending them through YARP
 *
 * - \c bin \c 1000 \n
 *   specifies the time-window inwhich we consider the events to be co-occurent
 * 
 * - \c threshold \c 2 \n
 *   specifies the number of local event needed to compute the optical flow at a specific adress
 * 
 * - \c szSobel \c 3 \n 
 *   specifies the size of the sobel filers
 *
 * - \c tsValidity \c 150000 \n 
 *   specifies the duration forwhich an event is valid (given its timestamp)
 *
 * - \c save \n
 *   if this parameter is given, the computation is save in a txt file in the format x y vx vy ts
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *    
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /eventSnifferModule
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /tsOptFlow/evts:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /tsOptFlow/flow:o \n
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
 * None
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>--name optFlow --source dvs --type 8 --height 128 --width 128 --acc 10000 --alpha 0 --tsValidity 150000 --szSobel 3 --threshold 2 --bin 1000</tt>
 *
 * \author Charles Clercq
 *
 * Copyright (C) 2012 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/velocityExtractor/include/iCub/velocityExtractorModule.h
 * 
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdio>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Matrix.h>

#include "iCub/emorph/eventBuffer.h"

#include "tsOptFlowThread.hpp"
#include "sendVelBuf.hpp"

class tsOptFlow:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer>
{
public:
    tsOptFlow(uint&, uint&, std::string&, uint&, uint&, uint&, double&, uint&, uint&, uint&, double&, double&, int&, uint&, bool&, yarp::os::BufferedPort<VelocityBuffer>*);
    ~tsOptFlow();
    void onRead(emorph::ebuffer::eventBuffer&);
private:
    
    yarp::os::Semaphore *mutex;
    tsOptFlowThread *tsofThreadPos;
    tsOptFlowThread *tsofThreadNeg;
    sendVelBuf *sendvelbuf;
    yarp::sig::Matrix *vxMat;   
    yarp::sig::Matrix *vyMat;

    VelocityBuffer *velBuf;
};

#endif //TSOPTFLOW_HPP
