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

#include "skinController.h"
#include "deviceRegisters.h"

#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>

vSkinCtrl::vSkinCtrl(std::string deviceName, unsigned char i2cAddress)
{

    fd = -1;
    this->deviceName = deviceName;
    this->I2CAddress = i2cAddress;

    fpgaStat.biasDone      = false;
    fpgaStat.tdFifoFull    = false;
    fpgaStat.apsFifoFull   = false;
    fpgaStat.i2cTimeout    = false;
    fpgaStat.crcErr        = false;

    iBias = false;

}

bool vSkinCtrl::connect()
{

    std::cout << "Connecting to " << deviceName << " for " << (int)I2CAddress << " device configuration" << std::endl;
    fd = open(deviceName.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open device: ");
        return false;
    }

    if(!clearFpgaStatus("all")) {
        std::cout << "Cannot write to " << deviceName << ":" << (int)I2CAddress << std::endl;
        std::cout << "Check sensor has correct I2CAddress and is physically connected" << std::endl;
        close(fd);
        return false;
    }

    return true;
}

void vSkinCtrl::disconnect(bool andturnoff)
{
    if(fd > 0) {
        close(fd);
    }
}

int vSkinCtrl::i2cWrite(unsigned char reg, unsigned int data)
{
    int ret = ioctl(fd, I2C_SLAVE, I2CAddress);
    int n_bits = sizeof (unsigned int) + sizeof(unsigned char);
    int n_bytes = n_bits/sizeof(unsigned char);
    unsigned char *tmp = (unsigned char *) malloc (n_bits);

    tmp[0]= reg|AUTOINCR;
    tmp[1]=data & 0xFF;
    tmp[2]=data>>8 & 0xFF;
    tmp[3]=data>>16 & 0xFF;
    tmp[4]=data>>24 & 0xFF;

    ret = write (fd, tmp, n_bytes);

    free(tmp);

    return (ret-1); // -1 because of one is the starting register

}

int vSkinCtrl::i2cWrite(unsigned char reg, unsigned char *data, unsigned int size)
{

    int ret = ioctl(fd, I2C_SLAVE, I2CAddress);

    unsigned char *tmp = (unsigned char *) malloc ((size+1)*sizeof(unsigned char));

    tmp[0]= size>1 ? reg|AUTOINCR : reg;

    for (unsigned int i = 0; i < size; i++)
        tmp[1+i]=data[i];

    ret = write (fd, tmp, size+1);

    free(tmp);

    return (ret-1); // -1 because of one is the starting register

}

int vSkinCtrl::i2cRead(unsigned char reg, unsigned char *data, unsigned int size)
{

    int ret = ioctl(fd, I2C_SLAVE, I2CAddress);
    unsigned char addr =  size>1 ? reg|AUTOINCR : reg;

    ret = write(fd, &addr, 1);
    if (ret<0)
        return ret;

    ret = read(fd, data, size);

    return ret;

}

bool vSkinCtrl::configure(bool verbose)
{
    if(!configureRegisters())
        return false;
    std::cout << deviceName << ":" << (int)I2CAddress << " registers configured." << std::endl;
    if(verbose)
        printConfiguration();
    return true;
}

bool vSkinCtrl::configureRegisters()
{

    unsigned char valReg[4];

    // --- configure SKCTRL_EN_ADDR --- //
    valReg[0] = I2C_ACQ_EN|FORCE_CALIB_EN;
    // I2C acquisition enable | force calibration | Dummy generators general enable | Current dummy generator select
    valReg[1] = 0;  // Disable dummy generator (e.g. which dummy generator you want to enable, from 0 to 7)
    valReg[2] = EVGEN_NTHR_EN|PREPROC_SAMPLES|PREPROC_EVGEN;
    // Event Generator configuration
    // Enable and configure events and Samples
    unsigned char rshift = (SAMPLES_RSHIFT_DEFAULT << SAMPLES_RSHIFT_SHIFT)&SAMPLES_RSHIFT;
    valReg[3] = rshift|SAMPLES_SEL|AUX_TX_EN|SAMPLES_TX_EN|EVENTS_TX_EN;
    if(i2cWrite(SKCTRL_EN_ADDR, valReg, 4) < 0) return false;

    // --- configure SKCTRL_DUMMY_PERIOD_ADDR --- //
    if(i2cWrite(SKCTRL_DUMMY_PERIOD_ADDR, DUMMY_PERIOD_DEFAULT) < 0) return false;

    // --- configure SKCTRL_DUMMY_CFG_ADDR --- //
    valReg[0] = LOW8(DUMMY_CALIB_DEFAULT);                   //
    valReg[1] = HIGH8(DUMMY_CALIB_DEFAULT);                    //
    valReg[2] = LOW8(DUMMY_ADDR_DEFAULT);                    //
    valReg[3] = HIGH8(DUMMY_ADDR_DEFAULT);                     //
    if(i2cWrite(SKCTRL_DUMMY_CFG_ADDR, valReg, 4) < 0) return false;

    // --- configure SKCTRL_DUMMY_BOUND_ADDR --- //
    valReg[0] = LOW8(DUMMY_UP_BOUND_DEFAULT);                //
    valReg[1] = HIGH8(DUMMY_UP_BOUND_DEFAULT);                 //
    valReg[2] = LOW8(DUMMY_LOW_BOUND_DEFAULT);               //
    valReg[3] = HIGH8(DUMMY_LOW_BOUND_DEFAULT);                //
    if(i2cWrite(SKCTRL_DUMMY_BOUND_ADDR, valReg, 4) < 0) return false;


    // --- configure SKCTRL_DUMMY_INC_ADDR --- //
    valReg[0] = LOW8(DUMMY_INC_DEFAULT);  //
    valReg[1] = HIGH8(DUMMY_INC_DEFAULT);  //
    valReg[2] = LOW8(DUMMY_DECR_DEFAULT);  //
    valReg[3] = HIGH8(DUMMY_DECR_DEFAULT);  //
    if(i2cWrite(SKCTRL_DUMMY_INC_ADDR, valReg, 4) < 0) return false;

    // --- configure SKCTRL_RES_TO_ADDR --- //
    if(i2cWrite(SKCTRL_RES_TO_ADDR, RESAMPLING_TIMEOUT_DEFAULT) < 0) return false;

    // --- configure SKCTRL_EG_UPTHR_ADDR --- //
    if(i2cWrite(SKCTRL_EG_UPTHR_ADDR, FIXED_UINT(EG_UP_THR_DEFAULT)) < 0) return false;

    // --- configure SKCTRL_EG_DWTHR_ADDR --- //
    if(i2cWrite(SKCTRL_EG_DWTHR_ADDR, FIXED_UINT(EG_DWN_THR_DEFAULT)) < 0) return false;

    // --- configure SKCTRL_EG_NOISE_RISE_THR_ADDR --- //
    if(i2cWrite(SKCTRL_EG_NOISE_RISE_THR_ADDR, FIXED_UINT(EG_NOISE_RISE_THR_DEFAULT)) < 0) return false;

    // --- configure SKCTRL_EG_NOISE_FALL_THR_ADDR --- //
    if(i2cWrite(SKCTRL_EG_NOISE_FALL_THR_ADDR, FIXED_UINT(EG_NOISE_FALL_THR_DEFAULT)) < 0) return false;

    // --- configure SKCTRL_I2C_ACQ_SOFT_RST_ADDR --- //
    if(i2cWrite(SKCTRL_I2C_ACQ_SOFT_RST_ADDR, I2C_ACQ_SOFT_RST_DEFAULT) < 0) return false;

    return true;
}

int vSkinCtrl::getFpgaStatus()
{

    unsigned char val;
    int ret = i2cRead(VSCTRL_STATUS_ADDR, &val, sizeof(val));

    fpgaStat.biasDone      = val & ST_BIAS_DONE_MSK;
    fpgaStat.tdFifoFull    = val & ST_TD_FIFO_FULL_MSK;
    fpgaStat.apsFifoFull   = val & ST_APS_FIFO_FULL_MSK;
    fpgaStat.i2cTimeout    = val & ST_I2C_TIMEOUT_MSK;
    fpgaStat.crcErr        = val & ST_CRC_ERR_MSK;

    return ret;
}

bool vSkinCtrl::clearFpgaStatus(std::string clr)
{
    unsigned char clrStatus;
    // --- to clear the value of one bit, write "1" to the bit --- //
    if (clr == "biasDone") {
        clrStatus = ST_BIAS_DONE_MSK;
    } else if (clr == "tdFifoFull") {
        clrStatus = ST_TD_FIFO_FULL_MSK;
    } else if (clr == "apsFifoFull"){
        clrStatus = ST_APS_FIFO_FULL_MSK;
    } else if (clr == "i2cTimeOut") {
        clrStatus = ST_I2C_TIMEOUT_MSK;
    } else if (clr == "crcErr"){
        clrStatus = ST_CRC_ERR_MSK;
    } else { // clear all
        clrStatus = 0xFF;
    }
// to be changed
    if(i2cWrite(SKCTRL_STATUS_ADDR, (unsigned char *)&clrStatus, sizeof(clrStatus)) < 0)
        return false;

    return true;
}

void vSkinCtrl::printConfiguration()
{

    std::cout << "Configuration for control device: " << (unsigned int)I2CAddress << std::endl;
// to be updated
//    std::cout << "== FPGA Register Values ==" << std::endl;
//    unsigned int regval = 0;
//    i2cRead(SKCTRL_INFO_ADDR, (unsigned char *)&regval, sizeof(regval));
//    printf("Info: 0x%08X\n", regval);
//    i2cRead(SKCTRL_STATUS_ADDR, (unsigned char *)&regval, sizeof(regval));
//    printf("Status: 0x%08X\n", regval);

}




