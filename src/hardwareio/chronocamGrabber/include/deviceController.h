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

#ifndef __VDEVCTRL__
#define __VDEVCTRL__

#include <yarp/os/Bottle.h>

#include <string>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>

#include "lib_atis.h"
#include "lib_atis_biases.h"
#include "lib_atis_instance.h"


class vDevCtrl
{
private:

    //PARAMETERS
    std::string deviceName;

    //INTERNAL VARIABLES
    Chronocam::CCamDevice *atis = nullptr;
    Chronocam::I_CCam *cam = nullptr;
    AtisBiases *biases = nullptr;
    Chronocam::I_EventsStream * stream = nullptr;
    
    yarp::os::Bottle bias;




    //WRAPPERS?
    bool configureRegisters(); //new initDevice



public:

    //REQUIRE: devicefilename, chiptype (eg DVS/ATIS), chipFPGAaddress (eg LEFT or RIGHT)
    vDevCtrl(std::string deviceName = "");

    //SET/GET CONFIGURATION
    bool setBias(std::string biasName, unsigned int biasValue);
    bool setBias(yarp::os::Bottle bias);
    unsigned int getBias(std::string biasName);
    
    //Share Atis instance between Controller and D2Y
    Chronocam::I_EventsStream & getStream();

    //CONNECTION
    bool connect(void);
    bool configure(bool verbose = false);
    void disconnect(bool andturnoff = true);
    bool activate(bool active = true);
    bool suspend(void); //wraps activate(false);

    //COMMANDS
    bool configureBiases();

    //DEBUG OUTPUTS
    void printConfiguration(void); // bias file, void dumpRegisterValues();

};

#endif
