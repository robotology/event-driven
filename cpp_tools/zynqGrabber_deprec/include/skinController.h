/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VSKINCTRL__
#define __VSKINCTRL__

#include <deviceRegisters.h>
#include <yarp/os/Bottle.h>

#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>

class vSkinCtrl
{
private:

    //PARAMETERS
    std::string deviceName;
    unsigned char I2CAddress;

    //INTERNAL VARIABLES
    int fd;

    //INTERNAL FUNCTIONS
    int i2cRead(unsigned char reg, unsigned char *data, unsigned int size);
    int i2cWrite(unsigned char reg, unsigned char *data, unsigned int size);
    int i2cWrite(unsigned char reg, unsigned int data);

    //WRAPPERS?
    bool setDefaultRegisterValues(); //new initDevice
    bool select_generator(int type, int neural_mask = 0);
    bool config_generator(int type, uint32_t p1, uint32_t p2, uint32_t p3, uint32_t p4);


public:

    //REQUIRE: devicefilename, chiptype (eg DVS/ATIS), chipFPGAaddress (eg LEFT or RIGHT)
    vSkinCtrl(std::string deviceName = "", unsigned char i2cAddress = 0);

    //CONNECTION
    bool connect(void);
    bool configure();
    bool calibrate();
    //void disconnect(bool andturnoff = true);
    void disconnect();

    //DEBUG OUTPUTS
    void printConfiguration(void); // bias file, void dumpRegisterValues();
    int printFpgaStatus();
    bool configureRegisters(yarp::os::Bottle cnfgReg);
    bool setRegister(int byte, unsigned int mask, unsigned char regAddr, bool regVal);
    bool setRegister(unsigned char regAddr, double regVal);


};

#endif
