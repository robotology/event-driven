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

#define SISLEY_GLOBAL_CTRL_REG     0x00
#define SISLEY_ROI_CTRL_REG        0x04

typedef enum {
    TD,
    EM,
} tdorem_t;

typedef enum {
    X,
    Y,
} xory_t;

#define SISLEY_CAM_X_SIZE          640
#define SISLEY_CAM_Y_SIZE          480
#define SISLEY_TD_ROI_X_OFFSET	   0x200
#define SISLEY_TD_ROI_Y_OFFSET	   0x300
#define SISLEY_EM_ROI_X_OFFSET	   0x400
#define SISLEY_EM_ROI_Y_OFFSET	   0x500

// VSCTRL: Enable clock
// data8=0x01;
// if (i2cWrite(fd, 0x21, &data8, 1) != 1) return false;

// // Enable GTP
// data8 = 0x24;
// if (i2cWrite(fd, 0x11, &data8, 1) != 1) return false;

// //align GTP between eye and zynq FPGA
// data8 = 0x01;
// if (i2cWrite(fd, 0x1C, &data8, 1) != 1) return false;
// yarp::os::Time::delay(0.01);
// data8 = 0x00;
// if (i2cWrite(fd, 0x1C, &data8, 1) != 1) return false;

// // VSCTRL: Flush Fifo
// data8=0x40;
// if (i2cWrite(fd, 0x10, &data8, 1) != 1) return false;


// if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x02) != 4) return false;
// //printf("GEN3: Assert bgen_en\n");
// if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x12) != 4) return false;
// //printf("GEN3: Assert bgen_rstn\n");
// if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x1A) != 4) return false;
// usleep(1000);


// unsigned char value_8bit;

// // VSCTRL: Enable VDDA, VDDD and VDDC end keep out from reset MRRSTN
// if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// value_8bit |= VSCTRL_ENABLE_VDDA;
// // I repeat three times as suggested in sisley reference stuff...
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

// if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// value_8bit |= VSCTRL_DISABLE_TDRSTN;
// // I repeat three times as suggested in sisley reference stuff...
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
// if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

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

int i2cWrite(int fd, unsigned char reg, unsigned char *data, unsigned int size)
{
    unsigned char *tmp = (unsigned char *) malloc ((size+1)*sizeof(unsigned char));
    tmp[0] = size > 1 ? reg|AUTO_INCREMENT : reg;
    for (unsigned int i = 0; i < size; i++)
        tmp[1+i]=data[i];

    int ret = write(fd, tmp, size+1);
    free(tmp);
    return (ret-1); // -1 because of one is the starting register
}

int readSisleyRegister(int fd, uint32_t sisley_reg_address, uint32_t *sisley_data)
{
    //  #define VSCTRL_SISLEY_DATA_REG     0x28
    //  #define VSCTRL_SISLEY_ADDRESS_REG  0x24
    sisley_reg_address &= 0x00FFFFFF;
    if(4 != i2cWrite(fd, VCTRL_SIS_ADDR, (unsigned char *)&sisley_reg_address, 4)) {
        yError() << "SisleyRead: Could not write the read command";
        return -1;
    }

    if(4 != i2cRead(fd, VCTRL_SIS_DATA, (unsigned char *)sisley_data, 4)) {
        yError() << "SisleyRead: Could not read the data";
        return -1;
    }

    return 0; 
}

int writeSisleyRegister(int fd, uint32_t sisley_reg_address, uint32_t sisley_data)
{
    if(4 != i2cWrite(fd, VCTRL_SIS_DATA, (unsigned char *)&sisley_data, 4)) {
        yError() << "SisleyWrite: Could not write the data";
        return -1;
    }

    sisley_reg_address &= 0x00FFFFFF;
    sisley_reg_address |= 0x80000000;
    if(4 != i2cWrite(fd, VCTRL_SIS_ADDR, (unsigned char *)&sisley_reg_address, 4)) {
        yError() << "SisleyWrite: Could not write the write command";
        return -1;
    }

    return 4;
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

void setROIfull(int fd)
{
    for(int i = 0; i < 640 / 32; i++) 
        writeSisleyRegister(fd, (i * 4) + SISLEY_TD_ROI_X_OFFSET, 0xFFFFFFFF);
    for(int i = 0; i < 480/32; i++) 
        writeSisleyRegister(fd, (i * 4) + SISLEY_TD_ROI_Y_OFFSET, 0xFFFFFFFF);
    
    writeSisleyRegister(fd, SISLEY_ROI_CTRL_REG, 0x02E);
    writeSisleyRegister(fd, SISLEY_ROI_CTRL_REG, 0x00E);
}

void setSensitivityBiases(int fd, int sensitivity)
{
    uint32_t diff_on{0};
    uint32_t diff_off{0};
    uint32_t diff{0};

    //set the constant diff value
    static const int diff_val = 43; 
    readSisleyRegister(fd, 0x154, &diff);
    diff &= 0xFF300000;
    diff += diff_val;
    writeSisleyRegister(fd, 0x154, diff);

    if(sensitivity < 0) sensitivity = 0;
    if(sensitivity > 100) sensitivity = 100;
    double s = 10 + 13 * (1.0 - sensitivity *0.01);
        
    readSisleyRegister(fd, 0x14C, &diff_off);
    diff_off &= 0xFF300000;
    diff_off += diff_val - s;
    writeSisleyRegister(fd, 0x14C, diff_off);

    readSisleyRegister(fd, 0x150, &diff_on);
    diff_on &= 0xFF300000;
    diff_on += diff_val + s;
    writeSisleyRegister(fd, 0x150, diff_on);

}

void setRefractoryBias(int fd, int period) 
{
    if(period < 0) period = 0;
    if(period > 100) period = 100;
    uint32_t bias_val = 0;
    readSisleyRegister(fd, 0x128, &bias_val);
    bias_val &= 0xFF300000;
    bias_val += 245 - 0.5 * (100-period);
    writeSisleyRegister(fd, 0x128, bias_val);
}

#endif