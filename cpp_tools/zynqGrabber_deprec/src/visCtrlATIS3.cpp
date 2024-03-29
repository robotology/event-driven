#include "visCtrlATIS3.h"
#include "visionController.h"
#include <unistd.h>
#include <yarp/os/all.h>

bool visCtrlATIS3::configure(yarp::os::ResourceFinder rf) 
{

    //these print statements seems necessary to have the code work. Inserting
    //delays doesn't work either. Typically it means there is a memory bug
    //somewhere and the prints change the way the code is compiled "avoiding"
    //the bug by chance.
    channelSelect(fd, channel);

    uint32_t data32;
    uint8_t  data8;
    
	// VSCTRL: Enable clock
	data8=0x01;
	if (i2cWrite(fd, 0x21, &data8, 1) != 1) return false;

    // Enable GTP
    data8 = 0x24;
	if (i2cWrite(fd, 0x11, &data8, 1) != 1) return false;

    //align GTP between eye and zynq FPGA
    data8 = 0x01;
    if (i2cWrite(fd, 0x1C, &data8, 1) != 1) return false;
    yarp::os::Time::delay(0.01);
    data8 = 0x00;
    if (i2cWrite(fd, 0x1C, &data8, 1) != 1) return false;

    // VSCTRL: Flush Fifo
	data8=0x40;
	if (i2cWrite(fd, 0x10, &data8, 1) != 1) return false;


    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x02) != 4) return false;
    //printf("GEN3: Assert bgen_en\n");
    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x12) != 4) return false;
    //printf("GEN3: Assert bgen_rstn\n");
    if (writeSisleyRegister(SISLEY_GLOBAL_CTRL_REG, 0x1A) != 4) return false;
    usleep(1000);


    unsigned char value_8bit;

    // VSCTRL: Enable VDDA, VDDD and VDDC end keep out from reset MRRSTN
    if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit |= VSCTRL_ENABLE_VDDA;
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    if (i2cRead(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    value_8bit |= VSCTRL_DISABLE_TDRSTN;
    // I repeat three times as suggested in sisley reference stuff...
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;
    if (i2cWrite(fd, VSCTRL_SISLEY_LDO_RSTN_REG, &value_8bit, 1) != 1) return false;

    // Enable data transmission from VSCTRL
    
    // data8=0x08;
    // if (i2cWrite(fd, 0x10, &data8, 1) != 1) return false;
    yInfo() << "\t...configured registers.";

    yarp::os::Bottle &roi_definition = rf.findGroup("ATIS_ROI");
    if(!roi_definition.isNull()) {
        int x = roi_definition.find("x").asInt32();
        int y = roi_definition.find("y").asInt32();
        int width = roi_definition.find("width").asInt32();
        int height = roi_definition.find("height").asInt32();
        yInfo() << "\tROI definition found [" << x << y << width << height << "]";
        setROI(x, y, width, height);
    }

    if(rf.check("sensitivity")) {
        int sensitivity = rf.find("sensitivity").asInt32();
        yInfo() << "\tSetting sensitivity to " << sensitivity;
        setSensitivityBiases(sensitivity);
    }
    printSensitivyBiases();

    if(rf.check("refractory")) {
        int ref_period = rf.find("refractory").asInt32();
        yInfo() << "\tSetting refractory period to " << ref_period;
        setRefractoryBias(ref_period);  
    }
    printRefractoryBias();

    //other options?

    return true;
}

int visCtrlATIS3::readSisleyRegister(uint32_t sisley_reg_address, uint32_t *sisley_data)
{
    //  #define VSCTRL_SISLEY_DATA_REG     0x28
    //  #define VSCTRL_SISLEY_ADDRESS_REG  0x24
    sisley_reg_address &= 0x00FFFFFF;
    if(4 != i2cWrite(fd, VSCTRL_SISLEY_ADDRESS_REG, (unsigned char *)&sisley_reg_address, 4)) {
        yError() << "SisleyRead: Could not write the read command";
        return -1;
    }

    if(4 != i2cRead(fd, VSCTRL_SISLEY_DATA_REG, (unsigned char *)sisley_data, 4)) {
        yError() << "SisleyRead: Could not read the data";
        return -1;
    }

    return 0; 
}

int visCtrlATIS3::writeSisleyRegister(uint32_t sisley_reg_address, uint32_t sisley_data)
{
    if(4 != i2cWrite(fd, VSCTRL_SISLEY_DATA_REG, (unsigned char *)&sisley_data, 4)) {
        yError() << "SisleyWrite: Could not write the data";
        return -1;
    }

    sisley_reg_address &= 0x00FFFFFF;
    sisley_reg_address |= 0x80000000;
    if(4 != i2cWrite(fd, VSCTRL_SISLEY_ADDRESS_REG, (unsigned char *)&sisley_reg_address, 4)) {
        yError() << "SisleyWrite: Could not write the write command";
        return -1;
    }

    return 4;
}

//BIAS REGISTER INFORMATION
//pad_enable | bias_enable | bias_polarity | cascode | bias_type (31:27)
//internal buffer (26:21)
//current_based_bias (20:8)
//voltage_based_bias (7:0)
//bias_diff_off = bias19 = reg 0x14C = F900011F = 0b11111 001000 0...01 00011111 
//bias_diff_on = bias20 = reg 0x150 = E9000136 =  0b11101 001000 0...01 00110110
//bias_refr = bias10 = reg 0x128 = D90001D3 

void visCtrlATIS3::printRefractoryBias() 
{
    uint32_t bias_val{0};
    readSisleyRegister(0x128, &bias_val);
    bias_val &= 0xFFF;
    yInfo() << "\tBiases:" << bias_val << "[period]";
}

void visCtrlATIS3::setRefractoryBias(int period) 
{
    if(period < 0) period = 0;
    if(period > 100) period = 100;
    uint32_t bias_val{0};
    readSisleyRegister(0x128, &bias_val);
    bias_val &= 0xFF300000;
    bias_val += 245 - 0.5 * (100-period);
    writeSisleyRegister(0x128, bias_val);
}

void visCtrlATIS3::setSensitivityBiases(int sensitivity)
{
    uint32_t diff_on{0};
    uint32_t diff_off{0};
    uint32_t diff{0};

    //set the constant diff value
    static const int diff_val = 43; 
    readSisleyRegister(0x154, &diff);
    diff &= 0xFF300000;
    diff += diff_val;
    writeSisleyRegister(0x154, diff);

    if(sensitivity < 0) sensitivity = 0;
    if(sensitivity > 100) sensitivity = 100;
    double s = 10 + 13 * (1.0 - sensitivity *0.01);
        
    readSisleyRegister(0x14C, &diff_off);
    diff_off &= 0xFF300000;
    diff_off += diff_val - s;
    writeSisleyRegister(0x14C, diff_off);

    readSisleyRegister(0x150, &diff_on);
    diff_on &= 0xFF300000;
    diff_on += diff_val + s;
    writeSisleyRegister(0x150, diff_on);

}

void visCtrlATIS3::printSensitivyBiases() 
{

    uint32_t diff_off{0};
    readSisleyRegister(0x14C, &diff_off);
    diff_off &= 0xFFF;
    uint32_t diff_on{0};
    readSisleyRegister(0x150, &diff_on);
    diff_on &= 0xFFF;
    uint32_t diff{0};
    readSisleyRegister(0x154, &diff);
    diff &= 0xFFF;
    yInfo() << "\tBiases:" << diff_off << diff << diff_on << "[diff_off diff diff_on]";

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
        vref = biasdata->get(1).asInt32();
        header = biasdata->get(2).asInt32();
        voltage = biasdata->get(3).asInt32();
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
    channelSelect(fd, channel);
    uint8_t data8 = activate ? 0x08 : 0x00;
    i2cWrite(fd, 0x10, &data8, 1);
    return true;
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



