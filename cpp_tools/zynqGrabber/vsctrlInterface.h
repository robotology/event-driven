/*
 *   Copyright (C) 2023 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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

#ifndef __ZYNQ_VSCTRL_INTERFACE__
#define __ZYNQ_VSCTRL_INTERFACE__

#include <yarp/os/all.h>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "vsctrlDevice.h"


int readCameraTypes(std::string i2c_device)
{

    //open the i2c line
    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return -1;
    }
    
    //set to talk to the left camera
    ioctl(fd, I2C_SLAVE, I2C_LEFT);
    int reg_value = 0;
    i2cRead(fd, VCTRL_INFO, (unsigned char *)&reg_value, sizeof(reg_value));

    int version_major = (reg_value & 0x000000f0) >> 4;
    int version_minor = (reg_value & 0x0000000f);
    int camera_typel = (reg_value & 0x0000E000) >> 13;
    yInfo() << "LEFT: GEN" << camera_typel << " | VERSION: "<< version_major << "." << version_minor;

    //set to talk to right camera
    ioctl(fd, I2C_SLAVE, I2C_RIGHT);
    i2cRead(fd, VCTRL_INFO, (unsigned char *)&reg_value, sizeof(reg_value));
    version_major = (reg_value & 0x000000f0) >> 4;
    version_minor = (reg_value & 0x0000000f);
    int camera_typer = (reg_value & 0x0000E000) >> 13;
    yInfo() << "RIGHT: GEN" << camera_typer << " | VERSION: "<< version_major << "." << version_minor;

    close(fd);
    return camera_typel == camera_typer ? camera_typel : -1;
}

void resetFPGA(std::string i2c_device)
{
    yInfo() << "Resetting FGPA ...";
    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return;
    }

    unsigned char data = 0;

    //LEFT FPGA CONTROL REGISTERS
    ioctl(fd, I2C_SLAVE, 0x20);

    data = 0x01; //global reset
    i2cWrite(fd, 0x50, &data, sizeof(data));
    usleep(10000);
    data = 0x02; //reset clock device
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x00; //no reset
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x04; //reset phase shift
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x00; //no reset
    i2cWrite(fd, 0x50, &data, sizeof(data));

    //RIGHT FPGA CONTROL REGISTERS
    ioctl(fd, I2C_SLAVE, 0x21);

    data = 0x01;
    i2cWrite(fd, 0x50, &data, sizeof(data));
    usleep(10000);
    data = 0x02;
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x00;
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x04;
    i2cWrite(fd, 0x50, &data, sizeof(data));
    data = 0x00;
    i2cWrite(fd, 0x50, &data, sizeof(data));

    //usleep(3e6);
    close(fd);
    yInfo() << "Reset complete";
    

}

void atisLeftOff(std::string i2c_device)
{
    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return;
    }

    unsigned char data = 0x01;

    //LEFT FPGA CONTROL REGISTERS
    ioctl(fd, I2C_SLAVE, 0x20);
    i2cWrite(fd, 0x52, &data, sizeof(data));

    close(fd);

}
void atisRightOff(std::string i2c_device) 
{

    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return;
    }

    unsigned char data = 0x01;
    ioctl(fd, I2C_SLAVE, 0x21);
    i2cWrite(fd, 0x52, &data, sizeof(data));

    close(fd);
}

void atis3on(int fd, int sensitivity, int refractory)
{
    uint32_t reg_value = 0;

    //set VDD lines on chip clock
    reg_value = 0x00010155;
    i2cWrite(fd, VCTRL_SIS_LDO_RST, (unsigned char *)&reg_value, sizeof(reg_value));

    //turn on GTP and perform alignment
    reg_value = 0x00802448; //the extra 4 does a FIFO FLUSH
    i2cWrite(fd, VCTRL_SRCDST_CTRL, (unsigned char *)&reg_value, sizeof(reg_value));
    reg_value = 0x00000001;
    i2cWrite(fd, VCTRL_GTP_CNFG, (unsigned char *)&reg_value, sizeof(reg_value));
    usleep(0.1e6);
    reg_value = 0x00000000;
    i2cWrite(fd, VCTRL_GTP_CNFG, (unsigned char *)&reg_value, sizeof(reg_value));

    //at the chip level - set the global config
    writeSisleyRegister(fd, SISLEY_GLOBAL_CTRL_REG, 0x1A);
    usleep(0.1e6);
    
    //use the entire sensor - (you HAVE to use ROI)
    setROIfull(fd);

    //set decent biases
    sensitivity = sensitivity < 10 ? 10 : sensitivity;
    sensitivity = sensitivity > 90 ? 90 : sensitivity;
    setSensitivityBiases(fd, sensitivity);

    refractory = refractory < 1 ? 1 : refractory;
    refractory = refractory > 99 ? 99 : refractory;
    setRefractoryBias(fd, refractory);

}

void turnOnATIS3GTP(std::string i2c_device, int sensitivity = 65, int refractory = 1)
{

    //open the i2c line
    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return;
    }
    uint32_t reg_value = 0;
    
    yInfo() << "====LEFT ====";
    ioctl(fd, I2C_SLAVE, I2C_LEFT);
    atis3on(fd, sensitivity, refractory);
    yInfo() << allRegInfo(fd);
    
    
    yInfo() << "====RIGHT====";
    ioctl(fd, I2C_SLAVE, I2C_RIGHT);
    atis3on(fd, sensitivity, refractory);
    yInfo() << allRegInfo(fd);

    
    close(fd);
}


#endif