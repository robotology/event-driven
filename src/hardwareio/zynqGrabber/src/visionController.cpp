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

#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>

vVisionCtrl::vVisionCtrl(std::string deviceName, std::string deviceType, unsigned char i2cAddress)
{

    fd = -1;
    this->deviceType = deviceType;
    this->deviceName = deviceName;
    this->I2CAddress = i2cAddress;

    fpgaStat.biasDone      = false;
    fpgaStat.tdFifoFull    = false;
    fpgaStat.apsFifoFull   = false;
    fpgaStat.i2cTimeout    = false;
    fpgaStat.crcErr        = false;

    iBias = false;
    aps = false;

}

bool vVisionCtrl::connect()
{

    std::cout << "Connecting to " << deviceName << " for " << (int)I2CAddress << " device configuration" << std::endl;
    fd = open(deviceName.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Cannot open device: ");
        return false;
    }

	if (ioctl(fd, I2C_SLAVE, I2CAddress) < 0)
	{
		printf("Failed to acquire bus access and/or talk to i2c slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
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

void vVisionCtrl::disconnect(bool andturnoff)
{
    if(fd > 0) {
        if(andturnoff) suspend();
        close(fd);
    }
}

int vVisionCtrl::i2cWrite(unsigned char reg, unsigned char *data, unsigned int size)
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

int vVisionCtrl::i2cRead(unsigned char reg, unsigned char *data, unsigned int size)
{

    int ret = ioctl(fd, I2C_SLAVE, I2CAddress);
    unsigned char addr =  size>1 ? reg|AUTOINCR : reg;

    ret = write(fd, &addr, 1);
    if (ret<0)
        return ret;

    ret = read(fd, data, size);

    return ret;

}

int vVisionCtrl::ReadSisleyRegister(uint32_t sisley_reg_address, uint32_t *sisley_data)
{
    uint32_t data32;
    char i2cdata[5];
    uint8_t *tmp;

    // Data to be written in SISLEY_ADDRESS_REG
    data32=READ_SISLEY_REG | (0xFFFFFF&sisley_reg_address);
    tmp = (uint8_t *)(&data32);
    i2cdata[0] = I2C_AUTOTINCRREGS | VSCTRL_SISLEY_ADDRESS_REG;
    i2cdata[1] = tmp[0];
    i2cdata[2] = tmp[1];
    i2cdata[3] = tmp[2];
    i2cdata[4] = tmp[3];
    write(fd, i2cdata, 5);

    // Now the SISLEY_DATA_REG can be read
    i2cdata[0] = I2C_AUTOTINCRREGS | VSCTRL_SISLEY_DATA_REG;
    write (fd, i2cdata, 1);

    if (read(fd, (char *)sisley_data, 4)!=4)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
        exit(1);
    }
    else
    {
        //printf("Data read: %02X%02X%02X%02X\n", (*sisley_data&0xFF000000)>>24, (*sisley_data&0xFF0000)>>16, (*sisley_data&0xFF00)>>8, (*sisley_data&0xFF)>>0);
    }

    return (0);
}

int vVisionCtrl::WriteSisleyRegister(uint32_t sisley_reg_address, uint32_t sisley_data)
{
    uint32_t data32;
    char i2cdata[5];
    uint8_t *tmp;

    // Data to be written in SISLEY_DATA_REG
    data32=sisley_data;
    tmp = (uint8_t *)(&data32);
    i2cdata[0] = I2C_AUTOTINCRREGS | VSCTRL_SISLEY_DATA_REG;
    i2cdata[1] = tmp[0];
    i2cdata[2] = tmp[1];
    i2cdata[3] = tmp[2];
    i2cdata[4] = tmp[3];
    write(fd, i2cdata, 5);

    // Data to be written in SISLEY_ADDRESS_REG
    data32=WRITE_SISLEY_REG | (0xFFFFFF&sisley_reg_address);
    tmp = (uint8_t *)(&data32);
    i2cdata[0] = I2C_AUTOTINCRREGS | VSCTRL_SISLEY_ADDRESS_REG;
    i2cdata[1] = tmp[0];
    i2cdata[2] = tmp[1];
    i2cdata[3] = tmp[2];
    i2cdata[4] = tmp[3];
    return write(fd, i2cdata, 5) - 1 ; // -1 because of one is the starting register


}

bool vVisionCtrl::configureBiaseseGen3() {
    return true; //checkBiasProg();
    // TODO Check how to configure biases. At the moment they are set by the sensor itself

    clearFpgaStatus("biasDone");
    double vref, voltage;
    std::string biasName;
    int header;
    size_t i;
    for (i = 1; i < bias.size() - 1; i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        biasName = biasdata->get(0).asString();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        uint32_t biasVal = 0;
        if (iBias) {
            biasVal = voltage;
        }
        else {
            biasVal = 511 * (voltage / vref);
        }
        biasVal += header << 24;
        std::cout << iBias << " " << biasName << " = " << biasVal <<std::endl;
	if (WriteSisleyRegister(BIAS_LATCHOUT_OR_PU + i * 4, biasVal) != sizeof(biasVal)) {
            return false;
        }
    }
}


bool vVisionCtrl::EnablePower()
{	uint8_t value_8bit;

    // VSCTRL: Enable VDDA, VDDD and VDDC end keep out from reset MRRSTN
    if (i2cRead(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit=value_8bit|(VSCTRL_ENABLE_VDDA);
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    if (i2cRead(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit=value_8bit|(VSCTRL_DISABLE_TDRSTN);
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    return true;
}

bool vVisionCtrl::SisleySetup() {
    printf("GEN3: Enable analog\n");
    if (WriteSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x02) != 4) return false;
    printf("GEN3: Assert bgen_en\n");
    if (WriteSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x12) != 4) return false;
    printf("GEN3: Assert bgen_rstn\n");
    if (WriteSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x1A) != 4) return false;
    usleep(1000);
    return true;
}

int vVisionCtrl::SetROI(int start, int size, xory_t coord, tdorem_t type)
{
	int limit;
	int addr_offset;
	int address;
	int pixel;
	int low, high;
	int temp;
	int i;

	if (coord==X) {
		if (size+start>SISLEY_CAM_X_SIZE)
			return(-1);

		limit=SISLEY_CAM_X_SIZE;
		if (type==TD)
			addr_offset=SISLEY_TD_ROI_X_OFFSET;
		else
			addr_offset=SISLEY_EM_ROI_X_OFFSET;
	} else {
		if (size+start>SISLEY_CAM_Y_SIZE)
			return(-1);

		limit=SISLEY_CAM_Y_SIZE;
		if (type==TD)
			addr_offset=SISLEY_TD_ROI_Y_OFFSET;
		else
			addr_offset=SISLEY_EM_ROI_Y_OFFSET;
	}

	address=-1;
    for (pixel=0;pixel<limit;pixel+=32)
    {	address++;
        //printf("@%03X ", address*4+addr_offset);
		low=pixel;
		high=pixel+31;
		//printf("Low: %d High:%d ", low, high);
		if (start>high) {
			//printf ("0x00000000\n");
			WriteSisleyRegister(address*4+addr_offset, 0x00000000);

			continue;
		}
		if ( (start>=low) && (start+size-1<=high) ) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i>=(start-low) && (i<=(start+size-1-low)))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			WriteSisleyRegister(address*4+addr_offset, temp);
			continue;
		}
		if ( (start>=low) && (start+size-1>=high) ) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i>=(start-low))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			WriteSisleyRegister(address*4+addr_offset, temp);

			continue;
		}
		if ( (start+size-1) > high ) {
			//printf("0xFFFFFFFF\n");
			WriteSisleyRegister(address*4+addr_offset, 0xFFFFFFFF);
		}
		if ( (start+size-1)<low ) {
			//printf ("0x00000000\n");
			WriteSisleyRegister(address*4+addr_offset, 0x00000000);
			continue;
		}
		if ( (start+size-1)< high) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i<=(start+size-low-1))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			WriteSisleyRegister(address*4+addr_offset, temp);
			continue;
		}
    }
    return (0);
}

bool vVisionCtrl::configure(bool verbose)
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

bool vVisionCtrl::configureRegisters()
{
    if (deviceType == "ATIS")
        return configureRegistersGen1();
    else if (deviceType == "ATISGen3")
        return configureRegistersGen3();


    return false;
}

bool vVisionCtrl::configureRegistersGen1() {
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
    if(aps) {
        std::cout << "APS events enabled";
        valReg[0] |= APS_CTRL;
    }
    valReg[1] = SRC_CTRL;   // Flush FIFOs = 0, Ignore FIFO Full = 0, PAER En = 0, SAER En = 1, GTP En = 0, Sel DEST = 01 (HSSAER)
    valReg[2] = 0;      // reserved
    valReg[3] = 0;      // reserved
    if(i2cWrite(VSCTRL_SRC_DST_CTRL_ADDR, valReg, 4) < 0) return false;

    // --- configure HSSAER --- //
// --- this should be done only if we use HSSAER (with ATIS, SKIN, but not with SpiNNaker nor DVS)
    valReg[0] =  VSCTRL_ENABLE_ALLCHANNELS;     // enable ch0, ch1, ch2
    valReg[1] = 0;          // reserved
    valReg[2] = 0;          // reserved
    valReg[3] = 0;          // reserved
    if(i2cWrite(VSCTRL_HSSAER_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure GPO register --- //
    valReg[0] = 0x00;// 0x2;
    valReg[1] = 0x00; //0x4;
    valReg[2] = 0x00;
    valReg[3] = 0x00;
    if(i2cWrite(VSCTRL_GPO_ADDR, valReg, 4) < 0) return false;

    return true;
}

bool vVisionCtrl::configureRegistersGen3() {
    if (!SetupVSCTRLinHSSAERmode()) return false;
    if (!SisleySetup()) return false;
    if (!EnablePower()) return false;
    // Enable data transmission from VSCTRL
    unsigned int data32=VSCTRL_ENABLE_GEN3;
    if (i2cWrite(VSCTRL_SRC_DST_CTRL_ADDR, (uint8_t *) &data32, 1) != 1) return false;
    std::cout << "ENABLED" << std::endl;
    return true;
}

bool vVisionCtrl::SisleyTDROIDefinition(int x, int y, int width, int height) {
 // Set TD ROI of 400x400 starting from pixel (220,140)
    if (SetROI(x, width, X, TD)) {
		printf("Error in defining TD X ROI\n");
		return false;
	}
    if (SetROI(y, height, Y, TD)) {
		printf("Error in defining TD Y ROI\n");
		return false;
	}
	// Enable TD ROI and trigger trasnfer values to the shadow registers
 	WriteSisleyRegister(SISLEY_ROI_CTRL_REG, 0x0000002E);
	WriteSisleyRegister(SISLEY_ROI_CTRL_REG, 0x0000000E);
}

bool vVisionCtrl::SetupVSCTRLinHSSAERmode() {
    uint32_t data32;
	// VSCTRL: Enable clock
	data32=VSCTRL_ENABLE_CLK;
	if (i2cWrite(VSCTRL_SISLEY_LDO_RSTN_REG1, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Enable 4 HSSAER channels
	data32=VSCTRL_ENABLE_ALLCHANNELS;
	if (i2cWrite(VSCTRL_HSSAER_CNFG_ADDR, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Select HSSAER as destination and Enable HSSAER
	data32=(VSCTRL_DESTINATION_HSSAER<<4) | (VSCTRL_ENABLETXON_HSSAER<<0);
	if (i2cWrite(VSCTRL_DSTCTRL_REG, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Flush Fifo
	data32=VSCTRL_FLUSH_FIFO;
	if (i2cWrite(VSCTRL_SRC_DST_CTRL_ADDR, (uint8_t *)&data32, 1) != 1) return false;
	return true;
}

bool vVisionCtrl::setBias(yarp::os::Bottle bias)
{
    if(bias.isNull())
        return false;

    this->bias = bias;
    return true;
}

// --- change the value of a single bias --- //
bool vVisionCtrl::setBias(std::string biasName, unsigned int biasValue)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return false;
    vals.pop(); //remove the old value
    vals.addInt(biasValue);
    return true;
}

unsigned int vVisionCtrl::getBias(std::string biasName)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return -1;
    return vals.get(3).asInt();
}

void vVisionCtrl::useCurrentBias(bool flag)
{
    iBias = flag;
}

void vVisionCtrl::turnOnAPS(bool flag)
{
    aps = flag;
}

bool vVisionCtrl::activateAPSShutter()
{
    unsigned char valReg[4];
    valReg[0] = 0x00;
    valReg[1] = 0x20;
    valReg[2] = 0x00;
    valReg[3] = 0x00;
    if(i2cWrite(VSCTRL_GPO_ADDR, valReg, 4) < 0)
        return false;
    return true;
}

bool vVisionCtrl::configureBiases(){
    if (this->deviceType == "ATIS")
        return configureBiasesGen1();
    else if (this->deviceType == "ATISGen3"){
	std::cout << "CONFIGURING BIASES GEN 3";
        return configureBiaseseGen3();}
}

bool vVisionCtrl::configureBiasesGen1() {
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

    std::cout << "Programming " << bias.size() - 1 << " biases:" << std::endl;
    double vref, voltage;
    int header;
    size_t i;
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
    return checkBiasProg();
}

bool vVisionCtrl::checkBiasProg() {
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

bool vVisionCtrl::setShiftCount(uint8_t shiftCount){

    unsigned int val;

    if(i2cRead(VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) < 0)
        return false;

    val = (val & ~BG_SHIFT_COUNT_MSK) | ((shiftCount << 16) & BG_SHIFT_COUNT_MSK);
    if(i2cWrite(VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(int)) < 0)
        return false;

    return true;
}

bool vVisionCtrl::setLatchAtEnd(bool enable){

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

bool vVisionCtrl::suspend()
{
    return activate(false);
}


bool vVisionCtrl::activate(bool active)
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


int vVisionCtrl::getFpgaStatus()
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

bool vVisionCtrl::clearFpgaStatus(std::string clr)
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

void vVisionCtrl::printConfiguration()
{

    std::cout << "Configuration for control device: " << (unsigned int)I2CAddress << std::endl;

    std::cout << "== Bias Values ==" << std::endl;
    std::cout << bias.toString() << std::endl;

    std::cout << "== Bias Hex Stream ==" << std::endl;
    double vref, voltage;
    int header;
    printf("0x%02X\n", 3);
    for(size_t i = 1; i < bias.size(); i++) {
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







