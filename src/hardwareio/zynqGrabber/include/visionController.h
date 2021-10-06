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

class vVisionCtrl
{
private:

    //PARAMETERS
    std::string deviceName;
    std::string deviceType;
    unsigned char I2CAddress;

    //INTERNAL VARIABLES
    int fd;
    yarp::os::Bottle bias;
    fpgaStatus_t fpgaStat;
    bool iBias;
    bool aps;

    //INTERNAL FUNCTIONS
    int i2cRead(unsigned char reg, unsigned char *data, unsigned int size);
    int i2cWrite(unsigned char reg, unsigned char *data, unsigned int size);
    int ReadSisleyRegister(uint32_t sisley_reg_address, uint32_t *sisley_data);
    int WriteSisleyRegister(uint32_t sisley_reg_address, uint32_t sisley_data);
    bool EnablePower();
    bool SisleySetup();
    int SetROI(int start, int size, xory_t coord, tdorem_t type);

    bool configureBiaseseGen3();
    bool configureBiasesGen1();

    bool configureRegistersGen1();
    bool configureRegistersGen3();

    bool SetupVSCTRLinHSSAERmode();
    bool enableGTP();

    //WRAPPERS?
    bool configureRegisters(); //new initDevice

    bool setLatchAtEnd(bool Enable);
    bool setShiftCount(uint8_t shiftCount);

    int getFpgaStatus();
    bool clearFpgaStatus(std::string clr);

    bool checkBiasProg();

public:

    //REQUIRE: devicefilename, chiptype (eg DVS/ATIS), chipFPGAaddress (eg LEFT or RIGHT)
    vVisionCtrl(std::string deviceName = "", std::string deviceType = "", unsigned char i2cAddress = 0);

    //SET/GET CONFIGURATION
    bool setBias(std::string biasName, unsigned int biasValue);
    bool setBias(yarp::os::Bottle bias);
    unsigned int getBias(std::string biasName);
    void useCurrentBias(bool flag = true);
    void turnOnAPS(bool flag = true);
    bool activateAPSShutter();

    //CONNECTION
    bool connect(void);
    bool configure(bool verbose = false);
    void disconnect(bool andturnoff = false);
    bool activate(bool active = true);
    bool suspend(void); //wraps activate(false);

    //COMMANDS
    bool configureBiases();
    bool SisleyTDROIDefinition(int x, int y, int width, int height);

    //DEBUG OUTPUTS
    void printConfiguration(void); // bias file, void dumpRegisterValues();


};

