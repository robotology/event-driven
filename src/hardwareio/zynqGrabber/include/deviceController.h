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

#include <deviceRegisters.h>
#include <yarp/os/Bottle.h>

#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>

typedef struct fpgaStatus {
    bool crcErr;
    bool biasDone;
    bool i2cTimeout;
    bool apsFifoFull;
    bool tdFifoFull;
} fpgaStatus_t;

class vDevCtrl
{
private:

    //PARAMETERS
    std::string deviceName;
    unsigned char I2CAddress;

    //INTERNAL VARIABLES
    int fd;
    yarp::os::Bottle bias;
    fpgaStatus_t fpgaStat;
    bool iBias;


    //INTERNAL FUNCTIONS
    int i2cRead(unsigned char reg, unsigned char *data, unsigned int size);
    int i2cWrite(unsigned char reg, unsigned char *data, unsigned int size);

    //WRAPPERS?
    bool configureRegisters(); //new initDevice

    bool setLatchAtEnd(bool Enable);
    bool setShiftCount(uint8_t shiftCount);

    int getFpgaStatus();
    bool clearFpgaStatus(std::string clr);

public:

    //REQUIRE: devicefilename, chiptype (eg DVS/ATIS), chipFPGAaddress (eg LEFT or RIGHT)
    vDevCtrl(std::string deviceName = "", unsigned char i2cAddress = 0);

    //SET/GET CONFIGURATION
    bool setBias(std::string biasName, unsigned int biasValue);
    bool setBias(yarp::os::Bottle bias);
    unsigned int getBias(std::string biasName);
    void useCurrentBias(bool flag = true);

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
