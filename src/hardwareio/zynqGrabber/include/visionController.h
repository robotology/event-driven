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

#pragma once

#include <deviceRegisters.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>

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


class visCtrlInterface
{
public:
    enum channel_name {LEFT = 0, RIGHT = 1};
    enum cam_type {DVS = 1, ATIS1 = 2, ATIS3 = 3}; //correspond to FPGA values

protected:

    static const int AUTO_INCREMENT = 0x80;
    static const int I2C_LEFT = 0x10;
    static const int I2C_RIGHT = 0x11;
    static const int VCTRL_INFO = 0x00;

    int fd{-1};
    channel_name channel{LEFT};

    static int extractCamType(int reg_value);
    static int channelSelect(int fd, channel_name name);
    static int i2cRead(int fd, unsigned char reg, unsigned char *data, 
                       unsigned int size);
    static int i2cWrite(int fd, unsigned char reg, unsigned char *data, 
                        unsigned int size);
    static void printConfiguration(int fd, channel_name name);

    static bool checkBiasDone(int fd)
    {
        unsigned char val;
        int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
        return val & ST_BIAS_DONE_MSK > 0;
    }
    
    static bool checkFifoFull(int fd)
    {
        unsigned char val;
        int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
        return val & ST_TD_FIFO_FULL_MSK > 0;
    }

    static bool checkAPSFifoFull(int fd)
    {
        unsigned char val;
        int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
        return val & ST_TD_FIFO_FULL_MSK > 0;
    }

    static bool checki2cTimeout(int fd)
    {
        unsigned char val;
        int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
        return val & ST_I2C_TIMEOUT_MSK > 0;
    }

    static bool checkCRCError(int fd)
    {
        unsigned char val;
        int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
        return val & ST_CRC_ERR_MSK > 0;
    }

    static bool clearStatusReg(int fd) 
    {
        unsigned char r = 0xFF;
        if (i2cWrite(fd, VSCTRL_STATUS_ADDR, (unsigned char *)&r, sizeof(r)) == sizeof(r))
            return true;
        else
            return false;
    }

public:

    visCtrlInterface(int fd, channel_name channel);
    static int openI2Cdevice(std::string path);
    static void closeI2Cdevice(int fd);
    static int getChannelI2CAddress(int fd, channel_name channel)
    {
        return channelSelect(fd, channel);
    }
    static int readCameraType(int fd, channel_name name);

    virtual bool activate(bool activate = true);
    virtual bool configure(yarp::os::ResourceFinder rf) = 0;
    virtual bool printConfiguration()
    {
            printConfiguration(fd, channel);
    }
};

class autoVisionController
{
private:
    int fd;
    visCtrlInterface * controls[2];
    visCtrlInterface *createController(int fd, visCtrlInterface::channel_name channel);

public:

    autoVisionController();
    ~autoVisionController();
    void connect(std::string i2c_device);
    void configureAndActivate(yarp::os::ResourceFinder rf);
    void disconnect()
    {
        if(controls[0]) controls[0]->activate(false);
        if(controls[1]) controls[1]->activate(false);
    }
};

