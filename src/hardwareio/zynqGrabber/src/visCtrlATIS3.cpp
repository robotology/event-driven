#include "visCtrlATIS3.h"
#include "visionController.h"
#include <unistd.h>
#include <yarp/os/all.h>

int visCtrlATIS3::readSisleyRegister(uint32_t sisley_reg_address, uint32_t *sisley_data)
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
    read(fd, i2cdata, 5);

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

int visCtrlATIS3::writeSisleyRegister(uint32_t sisley_reg_address, uint32_t sisley_data)
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

bool visCtrlATIS3::updateBiases(yarp::os::Bottle &bias) {
    
    
    return true; //checkBiasProg();
    // TODO Check how to configure biases. At the moment they are set by the sensor itself
    
    clearStatusReg(fd);
    double vref, voltage;
    std::string biasName;
    int header;
    size_t i;
    for (i = 1; i < bias.size(); i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        biasName = biasdata->get(0).asString();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        uint32_t biasVal = 0;
        biasVal = 511 * (voltage / vref);

        biasVal += header << 24;
        yInfo() << biasName << " = " << biasVal;
	if (writeSisleyRegister(BIAS_LATCHOUT_OR_PU + i * 4, biasVal) != sizeof(biasVal)) {
            return false;
        }
    }
}

bool visCtrlATIS3::activate(bool activate)
{	
    unsigned char value_8bit;

    // VSCTRL: Enable VDDA, VDDD and VDDC end keep out from reset MRRSTN
    if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit=value_8bit|(VSCTRL_ENABLE_VDDA);
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit=value_8bit|(VSCTRL_DISABLE_TDRSTN);
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    return true;
}

bool visCtrlATIS3::sisleySetup() {
    printf("GEN3: Enable analog\n");
    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x02) != 4) return false;
    printf("GEN3: Assert bgen_en\n");
    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x12) != 4) return false;
    printf("GEN3: Assert bgen_rstn\n");
    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x1A) != 4) return false;
    usleep(1000);
    return true;
}

int visCtrlATIS3::setROIAXIS(int start, int size, xory_t coord, tdorem_t type)
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
			writeSisleyRegister(address*4+addr_offset, 0x00000000);

			continue;
		}
		if ( (start>=low) && (start+size-1<=high) ) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i>=(start-low) && (i<=(start+size-1-low)))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			writeSisleyRegister(address*4+addr_offset, temp);
			continue;
		}
		if ( (start>=low) && (start+size-1>=high) ) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i>=(start-low))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			writeSisleyRegister(address*4+addr_offset, temp);

			continue;
		}
		if ( (start+size-1) > high ) {
			//printf("0xFFFFFFFF\n");
			writeSisleyRegister(address*4+addr_offset, 0xFFFFFFFF);
		}
		if ( (start+size-1)<low ) {
			//printf ("0x00000000\n");
			writeSisleyRegister(address*4+addr_offset, 0x00000000);
			continue;
		}
		if ( (start+size-1)< high) {
			temp=0;
			for (i=0;i<32;i++) {
				if (i<=(start+size-low-1))
					temp=temp+(1<<i);
			}
			//printf ("0x%08X\n", temp);
			writeSisleyRegister(address*4+addr_offset, temp);
			continue;
		}
    }
    return (0);
}

bool visCtrlATIS3::configure(yarp::os::ResourceFinder rf) 
{
    yarp::os::Time::delay(0.01);
    if (!enableGTP()) return false;
    yarp::os::Time::delay(0.01);
    if (!sisleySetup()) return false;
    yarp::os::Time::delay(0.01);
    if (!activate()) return false;
    yarp::os::Time::delay(0.01);
    // Enable data transmission from VSCTRL
    unsigned int data32=VSCTRL_ENABLE_GEN3;
    if (i2cWrite(fd, VSCTRL_SRC_DST_CTRL_ADDR, (uint8_t *) &data32, 1) != 1) return false;
    yInfo() << "ENABLED";
    
    //setup biases?
    //region of interest?
    //other options?

    return true;
}

bool visCtrlATIS3::setROI(int x, int y, int width, int height) {
 // Set TD ROI of 400x400 starting from pixel (220,140)
    if (setROIAXIS(x, width, X, TD)) {
		printf("Error in defining TD X ROI\n");
		return false;
	}
    if (setROIAXIS(y, height, Y, TD)) {
		printf("Error in defining TD Y ROI\n");
		return false;
	}
	// Enable TD ROI and trigger trasnfer values to the shadow registers
 	writeSisleyRegister(SISLEY_ROI_CTRL_REG, 0x0000002E);
	writeSisleyRegister(SISLEY_ROI_CTRL_REG, 0x0000000E);
}

bool visCtrlATIS3::enableGTP()
{
    uint32_t data32;
	// VSCTRL: Enable clock
	data32=VSCTRL_ENABLE_CLK;
	if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG1, (uint8_t *)&data32, 1) != 1) return false;

    // Enable GTP
    data32 = 0x24;
	if (i2cWrite(fd, VSCTRL_DSTCTRL_REG, (uint8_t *)&data32, 1) != 1) return false;

    //align GTP between eye and zynq FPGA
    data32 = 0x01;
    if (i2cWrite(fd, VSCTRL_GTP_CNFG_ADDR, (uint8_t *)&data32, 1) != 1) return false;
    yarp::os::Time::delay(0.01);
    data32 = 0x00;
    if (i2cWrite(fd, VSCTRL_GTP_CNFG_ADDR, (uint8_t *)&data32, 1) != 1) return false;

    // VSCTRL: Flush Fifo
	data32=VSCTRL_FLUSH_FIFO;
	if (i2cWrite(fd, VSCTRL_SRC_DST_CTRL_ADDR, (uint8_t *)&data32, 1) != 1) return false;

}

bool visCtrlATIS3::enableHSSAER() {
    uint32_t data32;
	// VSCTRL: Enable clock
	data32=VSCTRL_ENABLE_CLK;
	if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG1, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Enable 4 HSSAER channels
	data32=VSCTRL_ENABLE_ALLCHANNELS;
	if (i2cWrite(fd, VSCTRL_HSSAER_CNFG_ADDR, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Select HSSAER as destination and Enable HSSAER
	data32=(VSCTRL_DESTINATION_HSSAER<<4) | (VSCTRL_ENABLETXON_HSSAER<<0);
	if (i2cWrite(fd, VSCTRL_DSTCTRL_REG, (uint8_t *)&data32, 1) != 1) return false;
	// VSCTRL: Flush Fifo
	data32=VSCTRL_FLUSH_FIFO;
	if (i2cWrite(fd, VSCTRL_SRC_DST_CTRL_ADDR, (uint8_t *)&data32, 1) != 1) return false;
	return true;
}