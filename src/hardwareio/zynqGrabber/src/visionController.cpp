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


#include "visionController.h"
#include "deviceRegisters.h"
#include "visCtrlATIS1.h"
#include "visCtrlATIS3.h"
#include <yarp/os/all.h>

#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>

// =================== visCtrlInterface =================== //

int visCtrlInterface::extractCamType(int reg_value) 
{
    return (reg_value & 0x0000E000) >> 13;
}

int visCtrlInterface::channelSelect(int fd, channel_name name) 
{
    switch (name) 
    {
        case (LEFT):
            ioctl(fd, I2C_SLAVE, I2C_LEFT);
            return I2C_LEFT;
            break;
        case (RIGHT): 
            ioctl(fd, I2C_SLAVE, I2C_RIGHT);
            return I2C_RIGHT;
            break;
    }
    return -1;
}

int visCtrlInterface::i2cRead(int fd, unsigned char reg, unsigned char *data, 
                              unsigned int size) 
{
    unsigned char addr = size > 1 ? reg | AUTO_INCREMENT : reg;
    int ret = write(fd, &addr, 1);
    if (ret < 0) return ret;
    return read(fd, data, size);
}

int visCtrlInterface::i2cWrite(int fd, unsigned char reg, unsigned char *data, 
                            unsigned int size)
{

    unsigned char *tmp = (unsigned char *) malloc ((size+1)*sizeof(unsigned char));

    tmp[0] = size > 1 ? reg|AUTOINCR : reg;

    for (unsigned int i = 0; i < size; i++)
        tmp[1+i]=data[i];

    int ret = write(fd, tmp, size+1);

    free(tmp);

    return (ret-1); // -1 because of one is the starting register

}

visCtrlInterface::visCtrlInterface(int fd, channel_name channel) 
{
    this->fd = fd;
    this->channel = channel;
}

int visCtrlInterface::openI2Cdevice(std::string path) 
{
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0)
        perror("Cannot open i2c device: ");
    return fd;
}

void visCtrlInterface::closeI2Cdevice(int fd) 
{
    close(fd);
}

int visCtrlInterface::readCameraType(int fd, channel_name name) 
{
    channelSelect(fd, name);
    int reg_value = 0;
    if(i2cRead(fd, VCTRL_INFO, (unsigned char *)&reg_value, sizeof(reg_value)) 
            != sizeof(reg_value))
        return -1;
    else
        return extractCamType(reg_value);
}

bool visCtrlInterface::activate(bool activate)
{
    yError() << "This controller doesn't have an activate function";
    return false;
}

bool visCtrlInterface::configure(yarp::os::ResourceFinder rf)
{
    yError() << "This controller doesn't have a configure function";
    return false;
}

bool visCtrlInterface::checkBiasDone(int fd) {
    unsigned char val;
    int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
    return (val & ST_BIAS_DONE_MSK) > 0;
}

bool visCtrlInterface::checkFifoFull(int fd) {
    unsigned char val;
    int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
    return (val & ST_TD_FIFO_FULL_MSK) > 0;
}

bool visCtrlInterface::checkAPSFifoFull(int fd) {
    unsigned char val;
    int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
    return (val & ST_TD_FIFO_FULL_MSK) > 0;
}

bool visCtrlInterface::checki2cTimeout(int fd) {
    unsigned char val;
    int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
    return (val & ST_I2C_TIMEOUT_MSK) > 0;
}

bool visCtrlInterface::checkCRCError(int fd) {
    unsigned char val;
    int ret = i2cRead(fd, VSCTRL_STATUS_ADDR, &val, sizeof(val));
    return (val & ST_CRC_ERR_MSK) > 0;
}

bool visCtrlInterface::clearStatusReg(int fd) {
    unsigned char r = 0xFF;
    if (i2cWrite(fd, VSCTRL_STATUS_ADDR, (unsigned char *)&r, sizeof(r)) == sizeof(r))
        return true;
    else
        return false;
}

int visCtrlInterface::getChannelI2CAddress(int fd, channel_name channel) {
    return channelSelect(fd, channel);
}

void visCtrlInterface::printConfiguration() {
    printConfiguration(fd, channel);
}

void visCtrlInterface::printConfiguration(int fd, channel_name name)
{

    channelSelect(fd, name);
    switch (name) 
    {
        case (LEFT): yInfo() <<  "== LEFT FPGA Register Values ==";  break;
        case (RIGHT): yInfo() << "== RIGHT FPGA Register Values =="; break;
    }

    unsigned int regval = 0;
    i2cRead(fd, VSCTRL_INFO_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Info: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_STATUS_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Status: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_SRC_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Config: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_SRC_DST_CTRL_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("DstCtrl: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_PAER_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("PEAR-config: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_HSSAER_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("HSSAER-config: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_GTP_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GTP-config: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-config: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_BG_PRESC_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-prescaler: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_BG_TIMINGS_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-timings: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_GPO_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GPO: 0x%08X\n", regval);
    i2cRead(fd, VSCTRL_GPI_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GPI: 0x%08X\n", regval);

}

// =================== autoVisionController =================== //
visCtrlInterface *autoVisionController::createController(int fd, 
                                       visCtrlInterface::channel_name channel) 
{
    visCtrlInterface *controller = nullptr;
    yInfo() << "Attempting to connect to i2c address" 
            << visCtrlInterface::getChannelI2CAddress(fd, channel);
    int cam_type = visCtrlInterface::readCameraType(fd, channel);
    switch (cam_type) {
        case (visCtrlInterface::DVS):
            yInfo() << "DVS camera found";
            yError() << "No controller for DVS implemented";
            break;
        case (visCtrlInterface::ATIS1):
            yInfo() << "ATIS1 camera found";
            controller = new visCtrlATIS1(fd, channel);
            break;
        case (visCtrlInterface::ATIS3):
            yInfo() << "ATIS3 camera found";
            controller = new visCtrlATIS3(fd, channel);
            break;
        default:
            yInfo() << "No camera found" << cam_type;
            break;
    }
    return controller;
}
autoVisionController::autoVisionController() 
{
    fd = -1;
    controls[0] = nullptr;
    controls[1] = nullptr;
};

autoVisionController::~autoVisionController() 
{
    for (auto c : controls)
        if (c) delete c;
    visCtrlInterface::closeI2Cdevice(fd);
};

void autoVisionController::connect(std::string i2c_device) 
{
    yInfo() << "";
    yInfo() << "===== Vision Controller =====";
    fd = visCtrlInterface::openI2Cdevice(i2c_device);
    if(fd < 0) 
    {
        yInfo() << "Could not open i2c device:" << i2c_device;
    }
    else 
    { 
        yInfo() << "Found and opened" << i2c_device;
        controls[0] = createController(fd, visCtrlInterface::LEFT);
        controls[1] = createController(fd, visCtrlInterface::RIGHT);
    }
}

void autoVisionController::configureAndActivate(yarp::os::ResourceFinder rf) 
{
    bool lefton = rf.check("visLeftOn") && 
        rf.check("visLeftOn", yarp::os::Value(true)).asBool();
    bool righton = rf.check("visRightOn") && 
        rf.check("visRightOn", yarp::os::Value(true)).asBool();

    if(controls[0]) {
        if (!lefton)
            yWarning() << "Left camera connected but not using";
        else
            yInfo() << "Configuring Left Camera";
        controls[0]->configure(rf);
        controls[0]->activate(lefton);
        if(lefton) controls[0]->printConfiguration();
        
    }

    if(controls[1]) {
        if (!righton)
            yWarning() << "Right camera connected but not using";
        else
            yInfo() << "Configuring Right Camera";
        controls[1]->configure(rf);
        controls[1]->activate(righton);
        if(righton)controls[1]->printConfiguration();
        
    }

}
