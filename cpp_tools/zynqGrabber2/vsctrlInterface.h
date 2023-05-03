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


void readCameraTypes(std::string i2c_device)
{

    //open the i2c line
    int fd = open(i2c_device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open i2c device: ");
        return;
    }
    
    //set to talk to the left camera
    ioctl(fd, I2C_SLAVE, I2C_LEFT);
    int reg_value = 0;
    i2cRead(fd, VCTRL_INFO, (unsigned char *)&reg_value, sizeof(reg_value));

    yInfo() << tohex(reg_value);
    int version_major = (reg_value & 0x000000f0) >> 4;
    int version_minor = (reg_value & 0x0000000f);
    int camera_type = (reg_value & 0x0000E000) >> 13;
    yInfo() << "LEFT: " << camera_type << version_major << "." << version_minor;

    ioctl(fd, I2C_SLAVE, I2C_RIGHT);
    i2cRead(fd, VCTRL_INFO, (unsigned char *)&reg_value, sizeof(reg_value));
    yInfo() << tohex(reg_value);
    version_major = (reg_value & 0x000000f0) >> 4;
    version_minor = (reg_value & 0x0000000f);
    camera_type = (reg_value & 0x0000E000) >> 13;
    yInfo() << "RIGHT: " << camera_type << version_major << "." << version_minor;

    yInfo() << allRegInfo(fd);
    yInfo() << desiredRegisterValues();

    close(fd);
}


#endif