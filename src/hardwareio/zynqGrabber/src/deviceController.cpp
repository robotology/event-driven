/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
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

#include "deviceController.h"
#include "deviceRegisters.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

vDevCtrl::vDevCtrl(std::string deviceName, unsigned char i2cAddress)
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

bool vDevCtrl::connect()
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

void vDevCtrl::disconnect(bool andturnoff)
{
    if(fd > 0) {
        if(andturnoff) suspend();
        close(fd);
    }
}

int vDevCtrl::i2cWrite(unsigned char reg, unsigned char *data, unsigned int size)
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

int vDevCtrl::i2cRead(unsigned char reg, unsigned char *data, unsigned int size)
{

    int ret = ioctl(fd, I2C_SLAVE, I2CAddress);
    unsigned char addr =  size>1 ? reg|AUTOINCR : reg;

    ret = write(fd, &addr, 1);
    if (ret<0)
        return ret;

    ret = read(fd, data, size);

    return ret;

}

bool vDevCtrl::configure(bool verbose)
{
    if(!configureRegisters())
        return false;
    std::cout << deviceName << ":" << (int)I2CAddress << " registers configured." << std::endl;
    if(!configureBiases())
        return false;
    std::cout << deviceName << ":" << (int)I2CAddress << " biases configured." << std::endl;
    if(verbose)
        printConfiguration();
    return true;
}

bool vDevCtrl::configureRegisters()
{

    unsigned char valReg[4];

    // --- configure BG Timings --- //
    valReg[0] = BG_LAT;  // Latch Active Time
    valReg[1] = BG_LS;  // Latch Setup
    valReg[2] = BG_CAT;  // Clock Active Time
    valReg[3] = BG_SHT;  // Setup Hold Time
    if(i2cWrite(VSCTRL_BG_TIMINGS_ADDR, valReg, 4) < 0) return false;

    // --- configure BG Levels --- //
    valReg[0] = BG_CNFG;   // BGtype = 1 (ATIS), BG overwrite = 1, CK active level = 1, LATCH active level = 1
    valReg[1] = 0x00;   // reserved
    valReg[2] = BG_LATEND_SHCNT;   // LatchOut@end = 1, ShiftCount = 32
    valReg[3] = BG_ROI;   // Choose if setting ROI or setting BG (0 -> BG, 1 -> ROI)
    if(i2cWrite(VSCTRL_BG_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure BG Prescaler --- //
    for (int i = 0; i < 4; i++)
        valReg[i]  = (BG_PRESC >> (i*8)) & 0xFF;
    if(i2cWrite(VSCTRL_BG_PRESC_ADDR, valReg, 4) < 0) return false;

    // --- configure Source Config --- //
    valReg[0] = AER_LVL;   // AER Ack and Req Levels (Ack active low, Req active high)
    valReg[1] = ACK_SET_DEL;   // Ack Set Delay 20 ns
    valReg[2] = ACK_SAM_DEL;   // Ack Sample Delay 30 ns
    valReg[3] = ACK_REL_DEL;   // Ack Release Delay 50ns
    if(i2cWrite(VSCTRL_SRC_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure Source Destination Control --- //
    valReg[0] =  TD_APS_CTRL;  // TD loopback = 0, TD EN =1, APS loppback = 0, APS EN = 1, flush fifo = 0, ignore FIFO Full = 0
    valReg[1] = SRC_CTRL;   // Flush FIFOs = 0, Ignore FIFO Full = 0, PAER En = 0, SAER En = 1, GTP En = 0, Sel DEST = 01 (HSSAER)
    valReg[2] = 0;      // reserved
    valReg[3] = 0;      // reserved
    if(i2cWrite(VSCTRL_SRC_DST_CTRL_ADDR, valReg, 4) < 0) return false;

    // --- configure HSSAER --- //
    // --- this should be done only if we use HSSAER (with ATIS, SKIN, but not with SpiNNaker nor DVS)
    valReg[0] =  CH_SAER_EN;     // enable ch0, ch1, ch2
    valReg[1] = 0;          // reserved
    valReg[2] = 0;          // reserved
    valReg[3] = 0;          // reserved
    if(i2cWrite(VSCTRL_HSSAER_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure GPO register --- //
    valReg[0] = 0x2;
    valReg[1] = 0x4;
    valReg[2] = 0x0;
    valReg[3] = 0x0;
    if(i2cWrite(VSCTRL_GPO_ADDR, valReg, 4) < 0) return false;

    return true;
}

bool vDevCtrl::setBias(yarp::os::Bottle bias)
{
    if(bias.isNull())
        return false;

    this->bias = bias;
    return true;
}

// --- change the value of a single bias --- //
bool vDevCtrl::setBias(std::string biasName, unsigned int biasValue)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return false;
    vals.pop(); //remove the old value
    vals.addInt(biasValue);
    return true;
}

unsigned int vDevCtrl::getBias(std::string biasName)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return -1;
    return vals.get(3).asInt();
}

void vDevCtrl::useCurrentBias(bool flag)
{
    iBias = flag;
}

bool vDevCtrl::configureBiases(){

    clearFpgaStatus("biasDone");

    suspend();

    // send the first 4 bits (disabling the Latch)
    if(!setLatchAtEnd(false)) return false;
    if(!setShiftCount(ATIS_PDSHIFT)) return false;

    unsigned int pds = ATIS_PDSTRENGTH; //paddrivestrength
    if(i2cWrite(VSCTRL_BG_DATA_ADDR, (unsigned char *)&pds, sizeof(pds)) < 0)
        return false;

    // set the number of bits in each bias (ATIS is 32, DVS is 24)
    if(!setShiftCount(ATIS_BIASSHIFT))
        return false;

    std::cout << "Programming " << bias.size() << " biases:" << std::endl;
    double vref, voltage;
    int header;
    int i;
    for(i = 1; i < bias.size() - 1; i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        unsigned int biasVal = 0;
        if(iBias)
            biasVal = voltage;
        else
            biasVal = 255 * (voltage / vref);
        biasVal += header << 21;
        //std::cout << biasdata->get(0).asString() << " " << biasVal << std::endl;
        if(i2cWrite(VSCTRL_BG_DATA_ADDR, (unsigned char *)&biasVal, sizeof(biasVal)) != sizeof(biasVal))
            return false;
    }
    //set the latch true for the last bias
    //i = bias.size()
    if(!setLatchAtEnd(true)) return false;
    yarp::os::Bottle *biasdata = bias.get(i).asList();
    vref = biasdata->get(1).asInt();
    header = biasdata->get(2).asInt();
    voltage = biasdata->get(3).asInt();
    unsigned int biasVal = 0;
    if(iBias)
        biasVal = voltage;
    else
        biasVal = 255 * (voltage / vref);
    biasVal += header << 21;
    //std::cout << biasdata->get(0).asString() << " " << biasVal << std::endl;
    if(i2cWrite(VSCTRL_BG_DATA_ADDR, (unsigned char *)&biasVal, sizeof(biasVal)) != sizeof(biasVal))
        return false;

    // --- checks --- //

    getFpgaStatus();
    int count = 0;
    while (!fpgaStat.biasDone & (count <= 10000)){
        count++;
        getFpgaStatus();
        if (fpgaStat.crcErr){
            std::cout << "Bias write: failed programming, CRC Error " << fpgaStat.crcErr << std::endl;
            clearFpgaStatus("crcErr");
            return false;
        }

    }
    if (count > 10000) {
        std::cout << "Bias write: failed programming, Timeout "  << std::endl;
        return false;
    }

    clearFpgaStatus("biasDone");
    activate();

    return true;

}

bool vDevCtrl::setShiftCount(uint8_t shiftCount){

    unsigned int val;

    if(i2cRead(VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) < 0)
        return false;

    val = (val & ~BG_SHIFT_COUNT_MSK) | ((shiftCount << 16) & BG_SHIFT_COUNT_MSK);
    if(i2cWrite(VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(int)) < 0)
        return false;

    return true;
}

bool vDevCtrl::setLatchAtEnd(bool enable){

    unsigned int val;

    if(i2cRead(VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) < 0) return false;

    if (enable == true) {
        val |= BG_LATOUTEND_MSK;
    } else {
        val &= ~BG_LATOUTEND_MSK;
    }

    if(i2cWrite(VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(int)) < 0)
        return false;

    return true;
}

bool vDevCtrl::suspend()
{
    return activate(false);
}


bool vDevCtrl::activate(bool active)
{

    unsigned int val;

    //get current config state
    if(i2cRead(VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) != sizeof(val))
        return false;

    //alter the correct bit
    if(active)
        val &= ~BG_PWRDWN_MSK;
    else
        val |= BG_PWRDWN_MSK;

    //rewrite the new config status
    return i2cWrite(VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(unsigned int));
}


int vDevCtrl::getFpgaStatus()
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

bool vDevCtrl::clearFpgaStatus(std::string clr)
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

    if(i2cWrite(VSCTRL_STATUS_ADDR, (unsigned char *)&clrStatus, sizeof(clrStatus)) < 0)
        return false;

    return true;
}

void vDevCtrl::printConfiguration()
{

    std::cout << "Configuration for control device: " << (unsigned int)I2CAddress << std::endl;

    std::cout << "== Bias Values ==" << std::endl;
    std::cout << bias.toString() << std::endl;

    std::cout << "== Bias Hex Stream ==" << std::endl;
    double vref, voltage;
    int header;
    int i;
    printf("0x%02X\n", 3);
    for(i = 1; i < bias.size(); i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        unsigned int biasVal = 255 * (voltage / vref);
        biasVal += header << 21;
        printf("0x%08X\n", biasVal);
    }

    std::cout << "== FPGA Register Values ==" << std::endl;
    unsigned int regval = 0;
    i2cRead(VSCTRL_INFO_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Info: 0x%08X\n", regval);
    i2cRead(VSCTRL_STATUS_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Status: 0x%08X\n", regval);
    i2cRead(VSCTRL_SRC_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("Config: 0x%08X\n", regval);
    i2cRead(VSCTRL_SRC_DST_CTRL_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("DstCtrl: 0x%08X\n", regval);
    i2cRead(VSCTRL_PAER_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("PEAR-config: 0x%08X\n", regval);
    i2cRead(VSCTRL_HSSAER_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("HSSAER-config: 0x%08X\n", regval);
    i2cRead(VSCTRL_GTP_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GTP-config: 0x%08X\n", regval);
    i2cRead(VSCTRL_BG_CNFG_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-config: 0x%08X\n", regval);
    i2cRead(VSCTRL_BG_PRESC_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-prescaler: 0x%08X\n", regval);
    i2cRead(VSCTRL_BG_TIMINGS_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("BG-timings: 0x%08X\n", regval);
    i2cRead(VSCTRL_GPO_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GPO: 0x%08X\n", regval);
    i2cRead(VSCTRL_GPI_ADDR, (unsigned char *)&regval, sizeof(regval));
    printf("GPI: 0x%08X\n", regval);

}




