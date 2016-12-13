/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: chiara.bartolozzi@iit.it, arren.glover@iit.it
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

#include <string>
#include <yarp/os/Bottle.h>
#include <fstream>
#include <iCub/vsCtrl.h>

class vDevCtrl
{
private:

    //PARAMETERS
    std::string deviceName;
    std::string chipName;
    unsigned char I2CAddress;

    //INTERNAL VARIABLES
    int fd;
    yarp::os::Bottle bias;
    fpgaStatus_t fpgaStat;


    //INTERNAL FUNCTIONS
    int i2cRead(unsigned char reg, unsigned char *data, unsigned int size);
    int i2cWrite(unsigned char reg, unsigned char *data, unsigned int size);

    //int writeRegConfig(unsigned char regAddr, std::vector<uint8_t> regConfig); // do we need this?

    //std::vector<unsigned int> prepareBiases(); //do we need this?

    //WRAPPERS?
    bool configureRegisters(); //new initDevice


    bool setLatchAtEnd(bool Enable);
    bool setShiftCount(uint8_t shiftCount);

    int getFpgaStatus();
    bool clearFpgaStatus(std::string clr);

public:

    //REQUIRE: devicefilename, chiptype (eg DVS/ATIS), chipFPGAaddress (eg LEFT or RIGHT)
    vDevCtrl(std::string deviceName = "", std::string chipName = "", unsigned char i2cAddress = 0);

    //SET/GET CONFIGURATION
    bool setBias(std::string biasName, unsigned int biasValue);
    bool setBias(yarp::os::Bottle bias);
    unsigned int getBias(std::string biasName);

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
