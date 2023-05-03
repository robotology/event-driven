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

#ifndef __ZYNQ_VSCTRL_DEVICE__
#define __ZYNQ_VSCTRL_DEVICE__


#define I2C_LEFT 0x10
#define I2C_RIGHT 0x11

#define AUTO_INCREMENT      0x80

// --- addresses of the registers --- //
#define VCTRL_INFO         0x00
#define VCTRL_STATUS       0x04
#define VCTRL_GTP_STATS    0x08
#define VCTRL_SRC_CNFG     0x0C
#define VCTRL_SRCDST_CTRL  0x10
#define VCTRL_PAER_CNFG    0x14
#define VCTRL_HSSAER_CNFG  0x18
#define VCTRL_GTP_CNFG     0x1C
#define VCTRL_SIS_LDO_RST  0x20
#define VCTRL_SIS_ADDR     0x24
#define VCTRL_SIS_DATA     0x28

//#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
//#include <fcntl.h>
#include <sstream>
#include <iomanip>

std::string tohex(uint32_t value)
{
    std::stringstream ss; ss.str("");
    ss << "0x" << std::setw(8) << std::hex << std::setfill('0') << value;
    return ss.str();

}

int i2cRead(int fd, unsigned char reg, unsigned char *data, unsigned int size) 
{
    unsigned char addr = size > 1 ? reg | AUTO_INCREMENT : reg;
    int ret = write(fd, &addr, 1);
    if (ret < 0) return ret;
    return read(fd, data, size);
}

std::string allRegInfo(int fd)
{
    uint32_t data;
    std::stringstream ss; ss.str("");
    ss << "VISION CONTROLLER REGISTER DUMP" << std::endl;
    i2cRead(fd, VCTRL_INFO, (unsigned char *)&data, sizeof(data));
    ss << "0x00 VCTRL_INFO\t\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_STATUS, (unsigned char *)&data, sizeof(data));
    ss << "0x04 VCTRL_STATUS\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_GTP_STATS, (unsigned char *)&data, sizeof(data));
    ss << "0x08 VCTRL_GTP_STATS\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_SRC_CNFG, (unsigned char *)&data, sizeof(data));
    ss << "0x0C VCTRL_SRC_CNFG\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_SRCDST_CTRL, (unsigned char *)&data, sizeof(data));
    ss << "0x10 VCTRL_SRCDST_CTRL\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_PAER_CNFG, (unsigned char *)&data, sizeof(data));
    ss << "0x14 VCTRL_PAER_CNFG\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_HSSAER_CNFG, (unsigned char *)&data, sizeof(data));
    ss << "0x18 VCTRL_HSSAER_CNFG\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_GTP_CNFG, (unsigned char *)&data, sizeof(data));
    ss << "0x1C VCTRL_GTP_CNFG\t" << tohex(data) << std::endl;
    i2cRead(fd, VCTRL_SIS_LDO_RST, (unsigned char *)&data, sizeof(data));
    ss << "0x20 VCTRL_SIS_LDO_RST\t" << tohex(data) << std::endl;
    return ss.str();
}

std::string desiredRegisterValues()
{
    uint32_t data;
    std::stringstream ss; ss.str("");
    ss << "VISION CONTROLLER DESIRED" << std::endl;
    data = 0x00007441;
    ss << "0x00 VCTRL_INFO\t\t" << tohex(data) << std::endl;
    data = 0x00002408;
    ss << "0x10 VCTRL_SRCDST_CTRL\t" << tohex(data) << std::endl;

    data = 1;
    ss << "0x1C VCTRL_GTP_CNFG\t" << tohex(data) << std::endl;
    data = 0x00000000;
    ss << "0x20 VCTRL_SIS_LDO_RST\t" << tohex(data) << std::endl;
    return ss.str();
}

#endif