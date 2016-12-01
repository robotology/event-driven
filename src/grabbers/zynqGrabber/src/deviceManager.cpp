/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: chiara.bartolozzi@iit.it
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

#include "iCub/deviceManager.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>



// costruttore

deviceManager::deviceManager(bool bufferedRead, unsigned int maxBufferSize){

    //this->deviceName = deviceName;
    this->maxBufferSize = maxBufferSize;
    this->bufferedRead = bufferedRead;
    readCount = 0;
    bufferedreadwaiting = false;

    //allocate memory for reading
    buffer1.resize(maxBufferSize);
    readBuffer = &buffer1;
    accessBuffer = &buffer1;

    if(bufferedRead) {

        //allocate extra memory if buffered (constant) reading
        buffer2.resize(maxBufferSize);
        accessBuffer = &buffer2;
    }

#ifdef DEBUG
    writeDump.open("/tmp/writeDump.txt");
    readDump.open("/tmp/readDump.txt");
#endif

}


/*-----------------------------------------------------------
 functions for device open/close
 --------------------------------------------------------------*/

bool deviceManager::openDevice(){

    //opening the device
    std::cout << "name of the device: " << deviceName << std::endl;
    devDesc = ::open(deviceName.c_str(), O_RDWR);
    if (devDesc < 0) {
        std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
        perror("");
        return false;
    }



    //start the reading thread
    if(bufferedRead) start();

    return true;
}

void deviceManager::closeDevice()
{

    signal.post();
    //stop the read thread
    if(!stop())
        std::cerr << "Thread did not stop correctly" << std::endl;
    else
        std::cout << "Thread stopped. Continuing shutdown." << std::endl;

    ::close(devDesc);
    std::cout <<  "closing device " << deviceName << std::endl;

#ifdef DEBUG
    writeDump.close();
    readDump.close();
#endif

}

/*-----------------------------------------------------------
 functions for device read/write
 --------------------------------------------------------------*/

unsigned int deviceManager::writeDevice(std::vector<unsigned int> &deviceData){
    /*
     if(writeFifoAFull()){
     std::cout<<"Y2D write: warning fifo almost full"<<std::endl;
     }

     if(writeFifoFull()){
     std::cout<<"Y2D write: error fifo full"<<std::endl;
     }
     */
    unsigned int written =0;
    char* buff = (char *)deviceData.data();
    unsigned int len = deviceData.size()*sizeof(unsigned int);

    //send the vector to the device, read how many bytes have been written and if smaller than the full vector send the vector again from the location pointed by buff + written
    while (written < len) {

        int ret = ::write(devDesc, buff + written, len - written);

        if(ret > 0) {
            //std::cout << written << std::endl;
            written += ret;
        } else if(ret < 0 && errno != EAGAIN) {
            perror("Error writing because: ");
            break;
        }
    }

#ifdef DEBUG

    writeDump << deviceData << std::endl;

#endif

    return written/sizeof(unsigned int);

}



const std::vector<char>& deviceManager::readDevice(int &nBytesRead)
{
    if(bufferedRead) {

        //safely copy the data into the accessBuffer and reset the readCount
        //signal.post(); //tell the other thread we are ready (no wait)
        bufferedreadwaiting = true;
        //std::cout << "signal++" << std::endl;
        safety.wait();

        bufferedreadwaiting = false;

        //switch the buffer the read into
        std::vector<char> * temp = readBuffer;
        readBuffer = accessBuffer;
        accessBuffer = temp;

        //reset the filling position
        nBytesRead = readCount;
        readCount = 0;
        safety.post();
        signal.check();
        signal.post(); //tell the other thread we are done

    } else {

        //std::cout << "Direct Read" << std::endl;
        nBytesRead = ::read(devDesc, readBuffer->data(), 1024);
        if(nBytesRead < 0 && errno != EAGAIN) perror("perror: ");

    }

    //and return the accessBuffer
    return *accessBuffer;

}


void deviceManager::run(void)
{

    //int ntimesr = 0;
    //int ntimesr0 = 0;
    //double ytime = yarp::os::Time::now();
    signal.check();

    while(!isStopping()) {

        safety.wait();

        //std::cout << "Reading events from FIFO" << std::endl;
        //aerDevManager * tempcast = dynamic_cast<aerDevManager *>(this);
        //if(!tempcast) std::cout << "casting not working" << std::endl;
        //else {
        //    if(tempcast->readFifoFull())
        //	std::cout << "Fifo Full" << std::endl;
        //}

        //read SHOULD be a blocking call
        int r = ::read(devDesc, readBuffer->data() + readCount,
                       std::min(maxBufferSize - readCount, (unsigned int)256));
        //int r = ::read(devDesc, readBuffer->data() + readCount,
        //               maxBufferSize - readCount);


        //std::cout << readBuffer << " " <<  readCount << " " << maxBufferSize << std::endl;

        if(r > 0) {
            //std::cout << "Successful Read" << std::endl;
            readCount += r;
            //ntimesr++;
        } else if(r < 0 && errno != EAGAIN) {
            std::cerr << "Error reading from " << deviceName << " with error: " << r << std::endl;
            perror("perror: ");
            std::cerr << "readCount: " << readCount << "MaxBuffer: "
            << maxBufferSize << std::endl << std::endl;
            //ntimesr0++;
        }// else {
        //    ntimesr0++;
        //}

        if(readCount >= maxBufferSize) {
            std::cerr << "We reached maximum buffer! " << readCount << "/"
            << maxBufferSize << std::endl;
        }

        //std::cout << "Leaving safe zone" << std::endl;
        safety.post();
        //yarp::os::Time::delay(10);
        //if(signal.check()) {
        if(bufferedreadwaiting) {
            //the other thread is read to read
            //std::cout << "Read called with " << readCount << std::endl;
            signal.wait(); //wait for it to do the read
            //std::cout << ntimesr << " " << ntimesr0 << " " << (yarp::os::Time::now() - ytime)*1000 << std::endl;
            //ntimesr = 0;
            //ntimesr0 = 0;
            //ytime = yarp::os::Time::now();
        }


        //std::cout << "Done with FIFO" << std::endl;
    }
}
/* --------------------------------------------------------------------------------

 vsctrlDevManager -- to send biases and write registers -- specific to sensors

 ---------------------------------------------------------------------------------- */



vsctrlDevManager::vsctrlDevManager(std::string chip, unsigned char i2cAddress) : deviceManager(false, VSCTRL_MAX_BUF_SIZE) {

    this->deviceName = "/dev/i2c-2";
    this->I2CAddress = i2cAddress;

    fpgaStat.biasDone      = false;
    fpgaStat.tdFifoFull    = false;
    fpgaStat.apsFifoFull   = false;
    fpgaStat.i2cTimeout    = false;
    fpgaStat.crcErr        = false;

    bufferedRead = false;

    // default biases
    //this -> channel = channel;
    this -> chipName = chip;

};

bool vsctrlDevManager::openDevice(){

    //open the device
    bool ret1 = deviceManager::openDevice();
    // clear fpga registers
    bool ret2 = clearFpgaStatus("all");
    // set static configuration registers
    bool ret3 = initDevice();
    // program chip biases
    
    dumpRegisterValues();
    
    bool ret4 = programBiases();
    //bool ret4 = true;
    
    //return false;

    return ret1 & ret2 & ret3 & ret4;
}

void vsctrlDevManager::closeDevice(){

    chipPowerDown();
    deviceManager::closeDevice();

}

// --- write biases to device --- //
/*
 bool vsctrlDevManager::programBiases(){

 std::vector<unsigned int> vBiases = prepareBiases();

 //clearFpgaStatus("biasDone");
 //chipPowerDown();

 if (deviceManager::writeDevice(vBiases) <= 0) {
 std::cout << "Bias write: error writing to device" << std::endl;
 return false;
 }

 //getFpgaStatus();
 int count = 0;
 while (!fpgaStat.biasDone & (count <= 10000)){
 count++;
 //  getFpgaStatus();
 if (fpgaStat.crcErr){
 std::cout << "Bias write: failed programming, CRC Error " << fpgaStat.crcErr << std::endl;
 //    clearFpgaStatus("crcErr");
 return false;
 }

 }
 if (count > 10000) {
 std::cout << "Bias write: failed programming, Timeout "  << std::endl;
 return false;
 }

 //    clearFpgaStatus("biasDone");
 std::cout << "Biases correctly programmed" << std::endl;
 //  chipPowerUp();

 return true;

 }
 */
bool vsctrlDevManager::setBias(yarp::os::Bottle bias)
{
    if(bias.isNull())
        return false;

    this->bias = bias;
    std::cout << this->bias.toString() << std::endl;
    return true;

}

// --- change the value of a single bias --- //
bool vsctrlDevManager::setBias(std::string biasName, unsigned int biasValue) {

    // add header to bias Value
    int biasCurr;
    biasCurr = bias.find(biasName).asInt() & ~BG_VAL_MSK; // put zero to the bits from 0 to 21

    // assign the new value to the 21 LSB of the int that will be written to the chip
    bias.find(biasName) = biasCurr | (yarp::os::Value((int)biasValue).asInt() & BG_VAL_MSK);

    //bias.find(biasName) = yarp::os::Value((int)biasValue).asInt();
    if((unsigned int)bias.find(biasName).asInt() != (biasValue & BG_VAL_MSK)) {
        std::cerr << "Could not find " << biasName << " bias" << std::endl;
        return false;
    }
    return true;
}

// --- set the full vector that needs to be written to the device to program the chip --- //
std::vector<unsigned int> vsctrlDevManager::prepareBiases(){

    std::vector<unsigned int> vBiases;

    for(int i = 1; i < bias.size(); i++)
        vBiases.push_back(bias.get(i).asList()->get(1).asInt());

    return vBiases;

}

/****************************************************************

* i2c_write

****************************************************************/

int i2c_write(int handle, unsigned char reg, unsigned char *data, unsigned char eye, unsigned int size)

{

int ret;

unsigned char *tmp;

int i;

 

       ret = ioctl(handle, I2C_SLAVE, eye);

       tmp=(unsigned char *) malloc ((size+1)*sizeof(unsigned char));

       tmp[0]= size>1 ? reg|AUTOINCR : reg;

       for (i=0; i<size; i++)

             tmp[1+i]=data[i];

       ret = write (handle, tmp, size+1);

       free(tmp);

return (ret-1); // -1 because of one is the starting register

}

bool vsctrlDevManager::programBiases(){

    clearFpgaStatus("biasDone");

    std::vector<unsigned int> vBiases = prepareBiases();
    
    chipPowerDown();

    std::vector<uint8_t> valReg(4);
    int ret;
    uint8_t shiftCount;

    // send the first 4 bits (disabling the Latch)
    ret = setLatchAtEnd(false);
    shiftCount = 4; 
    ret = setShiftCount(shiftCount);
    unsigned int paddrivestrength = 3; //paddrivestrength
    i2c_write(devDesc, VSCTRL_BG_DATA_ADDR, (unsigned char *)&paddrivestrength, I2CAddress, sizeof(unsigned int));
    
    //ret = writeDevice(VSCTRL_BG_DATA_ADDR, 0);
    //ret = writeDevice(VSCTRL_BG_DATA_ADDR+3, 0); // write register from i2c

    // set the number of bits in each bias (ATIS is 32, DVS is 24)
    shiftCount = 32; // bisogna farlo configurabile!!!!!!!
    ret = setShiftCount(shiftCount);

	std::cout << "Programming " << vBiases.size() << " biases" << std::endl;
    for(int bias = 0; bias < vBiases.size(); bias++)
    {
		


        if(bias == vBiases.size()-1){
            // set the latch at the end of transmission
            ret = setLatchAtEnd(true);
            std::cout << "setting the latch true for bias number: " << bias << std::endl;
        }
        
        //std::cout << vBiases[bias] << std::endl;
		//printf("BG Config: 0x%08X\n", readDevice(VSCTRL_BG_CNFG_ADDR));

        int biasVal = vBiases[bias];

        //for (int i = 0; i < 4; i++){
        //    valReg[i]  = (biasVal >> (i*8)) & 0xFF;
        //}

        //ret = writeRegConfig(VSCTRL_BG_DATA_ADDR, valReg);
        ret = i2c_write(devDesc, VSCTRL_BG_DATA_ADDR, (unsigned char *)&biasVal, I2CAddress, sizeof(unsigned int));

    }
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
    std::cout << "Biases correctly programmed" << std::endl;
    chipPowerUp();
    dumpRegisterValues();

    return true;

}

unsigned int vsctrlDevManager::getBias(std::string biasName) {

    return bias.find(biasName).asInt();

}

void vsctrlDevManager::printBiases(){

    std::cout << bias.toString() << std::endl;

}




unsigned int vsctrlDevManager::readDevice(unsigned char reg)
{
    int ret;
    unsigned char buf[4];

    ret = ioctl(devDesc, I2C_SLAVE, I2CAddress);
    if( ret < 0) {
		std::cerr << "i2c read failed: ioctl(devDesc, I2C_SLAVE, I2CAddress) error " << errno << std::endl;
		return ret;
	}
    
    reg |= AUTOINCR;

	ret = write(devDesc, &reg, 1);
	if(ret < 0) {
		std::cerr << "write failed" << errno << std::endl;
		return ret;
	}
	ret = read(devDesc, buf, 4);
	if(ret < 0) {
		std::cerr << "read failed" << errno << std::endl;
		return ret;
	}

    return *((unsigned int *)buf);
}

int vsctrlDevManager::writeDevice(unsigned char reg, unsigned char data){
    int ret;
    unsigned char buf[2];

    ret = ioctl(devDesc, I2C_SLAVE, I2CAddress);
    if (ret == -1) {
        std::cerr << "i2c write failed: ioctl(devDesc, I2C_SLAVE, I2C_ADDRESS) error " << errno << std::endl;
        return ret;
    } else {

        buf[0] = reg;
        buf[1] = data;

        ret = write(devDesc,buf,2);

        if (ret == -1) {
            std::cerr << "i2c write failed: ioctl error " << errno << std::endl;
            return ret;
        } else {
            std::cout << "i2c write successful " << chipName << std::endl;
            return ret;
        }
    }
}

int vsctrlDevManager::writeRegConfig(unsigned char regAddr, std::vector<uint8_t> regConfig){

    int ret;

    for (int i = 0; i < 4; i++){

        ret = writeDevice(regAddr+i, regConfig[i]);

        if (ret == -1) {
            std::cerr << "write register " << regAddr << "failed: " << errno << std::endl;
        }
    }
    return ret;
}

int vsctrlDevManager::setShiftCount(uint8_t shiftCount){

    int ret;
    unsigned int val;

    val = readDevice(VSCTRL_BG_CNFG_ADDR);
    //printf("BiasConfigRegister: 0x%08X\n", val);

	val = (val & ~BG_SHIFT_COUNT_MSK) | ((shiftCount << 16) & BG_SHIFT_COUNT_MSK);
    //ret = writeDevice(VSCTRL_BG_CNFG_ADDR, (val & ~VSCTRL_BG_CNFG_ADDR) | (shiftCount & BG_SHIFT_COUNT_MSK));
    i2c_write(devDesc, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), I2CAddress, sizeof(int));

    if (ret == -1) {
        std::cerr << "write shift count failed: i2c write error " << errno << std::endl;
    }

    return ret;
}

int vsctrlDevManager::setLatchAtEnd(bool Enable){

    int ret = 0;
    unsigned int val;

    val = readDevice(VSCTRL_BG_CNFG_ADDR);

    if (Enable == true){
        val |= BG_LATOUTEND_MSK;
        i2c_write(devDesc, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), I2CAddress, sizeof(int));
    } else{
        val &= ~BG_LATOUTEND_MSK;
        i2c_write(devDesc, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), I2CAddress, sizeof(int));
    }
    if (ret == -1) {
        std::cerr << "write shift count failed: i2c write error " << errno << std::endl;
    }

    return ret;
}

int vsctrlDevManager::chipPowerDown(){

    int ret = 0;
    unsigned int val;

    val = readDevice(VSCTRL_BG_CNFG_ADDR) | BG_PWRDWN_MSK;
	i2c_write(devDesc, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), I2CAddress, sizeof(int));
	
	
    //ret = writeDevice(VSCTRL_BG_CNFG_ADDR, val | (BG_PWRDWN_MSK));
    if (ret == -1) {
        std::cerr << "write power down failed: i2c write error " << errno << std::endl;
    } else {
        std::cout << "Chip Powered Off" << std::endl;
    }

    return ret;
}


int vsctrlDevManager::chipPowerUp(){

    int ret = 0;
    unsigned int val;

	val = readDevice(VSCTRL_BG_CNFG_ADDR) & ~BG_PWRDWN_MSK;
    i2c_write(devDesc, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), I2CAddress, sizeof(int));

    //ret = writeDevice(VSCTRL_BG_CNFG_ADDR, val | (~BG_PWRDWN_MSK));
    if (ret == -1) {
        std::cerr << "write power on failed: i2c write error " << errno << std::endl;
    } else {
        std::cout << "Chip Powered On" << std::endl;
    }

    return ret;
}

int vsctrlDevManager::chipReset(){
    // --- AL MOMENTO NON C'E' IL REGISTRO! --- //
    //int ret = -1;
    // unsigned char val;
    // check register address!!!!!
    // val = readDevice(VSCTRL_BG_CNFG_ADDR);

    //      ret = writeDevice(VSCTRL_BG_CNFG_ADDR, val | (BG_PWRDWN_MSK));
    //if (ret == -1) {
    //    std::cerr << "write power down failed: i2c write error " << errno << std::endl;
    //    } else {
    //        std::cout << "Chip Reset" << std::endl;
    //    }
    //
    //    return ret;
    std::cout << "Warning: Chip Reset not implemented yet!" << std::endl;
    return 1;
}


int vsctrlDevManager::getFpgaStatus(){

    int ret = -1;
    unsigned char val;

    val = readDevice(VSCTRL_STATUS_ADDR);

    fpgaStat.biasDone      = val & ST_BIAS_DONE_MSK;
    fpgaStat.tdFifoFull    = val & ST_TD_FIFO_FULL_MSK;
    fpgaStat.apsFifoFull   = val & ST_APS_FIFO_FULL_MSK;
    fpgaStat.i2cTimeout    = val & ST_I2C_TIMEOUT_MSK;
    fpgaStat.crcErr        = val & ST_CRC_ERR_MSK;

    return ret;
}

bool vsctrlDevManager::clearFpgaStatus(std::string clr){

    int ret;

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

    ret = writeDevice(VSCTRL_STATUS_ADDR, clrStatus);

    if (ret == -1) {
        std::cerr << "clear status failed: error " << errno << std::endl;
        return false;
    } else {
        std::cout << "cleared field: " << clr << std::endl;
        return true;
    }
}

void vsctrlDevManager::dumpRegisterValues()
{
	printf("Info: 0x%08X\n", readDevice(VSCTRL_INFO_ADDR));
	printf("Status: 0x%08X\n", readDevice(VSCTRL_STATUS_ADDR));
	printf("Source Config: 0x%08X\n", readDevice(VSCTRL_SRC_CNFG_ADDR));
	printf("Source Dest. Cont.: 0x%08X\n", readDevice(VSCTRL_SRC_DST_CTRL_ADDR));
	printf("PAER Config: 0x%08X\n", readDevice(VSCTRL_PAER_CNFG_ADDR));
	printf("HSSAER Config: 0x%08X\n", readDevice(VSCTRL_HSSAER_CNFG_ADDR));
	printf("GTP Config: 0x%08X\n", readDevice(VSCTRL_GTP_CNFG_ADDR));
	printf("BG Config: 0x%08X\n", readDevice(VSCTRL_BG_CNFG_ADDR));
	printf("BG Prescaler: 0x%08X\n", readDevice(VSCTRL_BG_PRESC_ADDR));
	printf("BG Timing: 0x%08X\n", readDevice(VSCTRL_BG_TIMINGS_ADDR));
	printf("GPO Config: 0x%08X\n", readDevice(VSCTRL_GPO_ADDR));
	printf("GPI Config: 0x%08X\n", readDevice(VSCTRL_GPI_ADDR));
}

bool vsctrlDevManager::initDevice(){ // TODO sistemare i ret!

    bool retInit = true;
    int ret = -1;

    // --- configure BG Timings --- //
    std::vector<uint8_t> valReg(4);

    valReg[0] = BG_LAT;  // Latch Active Time
    valReg[1] = BG_LS;  // Latch Setup
    valReg[2] = BG_CAT;  // Clock Active Time
    valReg[3] = BG_SHT;  // Setup Hold Time


    ret = writeRegConfig(VSCTRL_BG_TIMINGS_ADDR, valReg);

    if (ret == -1) {
        std::cerr << "init error - BG timings" << errno << std::endl;
        retInit = false;
    } else {
        std::cout << "BG Timings configured " << std::endl;
        ret = -1;
    }

    // --- configure BG Levels --- //
    valReg[0] = BG_CNFG;   // BGtype = 1 (ATIS), BG overwrite = 1, CK active level = 1, LATCH active level = 1
    valReg[1] = 0x00;   // reserved
    valReg[2] = BG_LATEND_SHCNT;   // LatchOut@end = 1, ShiftCount = 32
    valReg[3] = BG_ROI;   // Choose if setting ROI or setting BG (0 -> BG, 1 -> ROI)

    ret = writeRegConfig(VSCTRL_BG_CNFG_ADDR, valReg);

    if (ret == -1) {
        std::cerr << "init error - BG levels " << errno << std::endl;
        retInit = false;
    } else {
        std::cout << "BG Levels configured " << std::endl;
        ret = -1;
    }
    // --- configure BG Prescaler --- //

    int prescaler = BG_PRESC; // prescaler value (times 10ns)

    for (int i = 0; i < 4; i++){
        valReg[i]  = (prescaler >> (i*8)) & 0xFF;
    }

    ret = writeRegConfig(VSCTRL_BG_PRESC_ADDR, valReg);

    if (ret == -1) {
        std::cerr << "init error - BG prescaler " << errno << std::endl;
        retInit = false;
    } else {
        std::cout << "BG prescaler configured " << std::endl;
        ret = -1;
    }
    // --- configure Source Config --- //

    valReg[0] = AER_LVL;   // AER Ack and Req Levels (Ack active low, Req active high)
    valReg[1] = ACK_SET_DEL;   // Ack Set Delay 20 ns
    valReg[2] = ACK_SAM_DEL;   // Ack Sample Delay 30 ns
    valReg[3] = ACK_REL_DEL;   // Ack Release Delay 50ns

    ret = writeRegConfig(VSCTRL_SRC_CNFG_ADDR, valReg);

    if (ret == -1) {
        std::cerr << "init error - Source config " << errno << std::endl;
        retInit = false;
    } else {
        std::cout << "Source configured " << std::endl;
        ret = -1;
    }
    // --- configure Source Destination Control --- //
    valReg[0] =  TD_APS_CTRL;  // TD loopback = 0, TD EN =1, APS loppback = 0, APS EN = 1, flush fifo = 0, ignore FIFO Full = 0
    valReg[1] = SRC_CTRL;   // Flush FIFOs = 0, Ignore FIFO Full = 0, PAER En = 0, SAER En = 1, GTP En = 0, Sel DEST = 01 (HSSAER)
    valReg[2] = 0;      // reserved
    valReg[3] = 0;      // reserved

    ret = writeRegConfig(VSCTRL_SRC_DST_CTRL_ADDR, valReg);

    if (ret == -1) {
        std::cerr << "init error - Source/Destination Control " << errno << std::endl;
        retInit = false;
    } else {
        std::cout << "Source/Destination Control configured " << std::endl;
        ret = -1;
    }
    // --- configure HSSAER --- //
    // --- this should be done only if we use HSSAER (with ATIS, SKIN, but not with SpiNNaker nor DVS)
    //if (chipName == "atis" || chipName == "skin"){
        valReg[0] =  CH_SAER_EN;     // enable ch0, ch1, ch2
        valReg[1] = 0;          // reserved
        valReg[2] = 0;          // reserved
        valReg[3] = 0;          // reserved

        ret = writeRegConfig(VSCTRL_HSSAER_CNFG_ADDR, valReg);

        if (ret == -1) {
            std::cerr << "init error - HSSAER Channel Config " << errno << std::endl;
            retInit = false;
        } else {
            std::cout << "HSSAER Channels configured " << std::endl;
        }
    /*
    //} else if (chipName == "spinn") {
    // --- configure PAER Ack and Req levels --- //
        valReg[0] =  TX_PAER_CFG;     // enable ch0, ch1, ch2
        valReg[1] = 0;          // reserved
        valReg[2] = 0;          // reserved
        valReg[3] = 0;          // reserved

        ret = writeRegConfig(VSCTRL_PAER_CNFG_ADDR, valReg);

        if (ret == -1) {
            std::cerr << "init error - HSSAER Channel Config " << errno << std::endl;
            retInit = false;
        } else {
            std::cout << "HSSAER Channels configured " << std::endl;
        }
    //}
    */
    
    // --- configure GPO register --- //
    valReg[0] =  0; 
    valReg[1] = 0; 
    valReg[2] = 0; 
    valReg[3] = 0;     

    ret = writeRegConfig(VSCTRL_GPO_ADDR, valReg);
    
    
    
    
    
    return retInit;
}


/* -----------------------------------------------------------------
 aerDevManager -- to handle AER IO: read events from sensors and spinnaker and write events to spinnaker
 ----------------------------------------------------------------- */
aerDevManager::aerDevManager(std::string dev, int clockPeriod, std::string loopBack) : deviceManager(false, AER_MAX_BUF_SIZE) {

    this->tickToUs = 1000.0/clockPeriod; // to scale the timestamp to 1us temporal resolution
    this->usToTick = 1.0/tickToUs; // to scale the 1us temporal resolution to hw clock ticks
    this->loopBack = loopBack;

    if (dev == "zynq_spinn")
    {

        this->deviceName = "/dev/spinn2neu";
        //clockRes = 1;
    } else if (dev == "zynq_sens")
    {

        this->deviceName = "/dev/iit-hpu0";
        //clockRes = 1;
    } else {

        std::cout << "aer device error: unrecognised device" << std::endl;

    }
}

bool aerDevManager::openDevice(){

    bool ret = deviceManager::openDevice();

    if (ret!=false){
        //initialization for writing to device
        unsigned long version;
        unsigned char hw_major,hw_minor;
        char          stringa[4];
        int i;
        unsigned int  tmp_reg;

        ioctl(devDesc, AER_VERSION, &version);

        hw_major = (version & 0xF0) >> 4;
        hw_minor = (version & 0x0F);
        stringa[3]=0;

        for (i=0; i<3; i++) {
            stringa[i] = (version&0xFF000000) >> 24;
            version = version << 8;
        }

        std::cout << "Identified: " << stringa << " version " << static_cast<unsigned>(hw_major) << "." << static_cast<unsigned>(hw_minor) << std::endl;
        // Write the WrapTimeStamp register with any value if you want to clear it
        //aerWriteGenericReg(devDesc,STMP_REG,0);

        // Set mask for enabling interrupts
        aerWriteGenericReg(devDesc, MASK_REG, MSK_RX_PAER_ERR);
        std::cout << "Enabled FIFO Full interrupt " << std::endl;

        tmp_reg = aerReadGenericReg(devDesc, CTRL_REG);

        // Flush FIFOs
        aerWriteGenericReg(devDesc, CTRL_REG, tmp_reg | CTRL_FLUSHFIFO | (CTRL_ENABLEINTERRUPT));
        std::cout << "FIFO Flushed and Interrupt enabled " << std::endl;

    }
    return ret;
}

void aerDevManager::closeDevice(){

    deviceManager::closeDevice();
    //unsigned int tmp_reg = aerReadGenericReg(devDesc, CTRL_REG);
    //aerWriteGenericReg(devDesc, CTRL_REG, tmp_reg & ~(CTRL_ENABLEINTERRUPT));

}

bool aerDevManager::readFifoFull(){
    int devData=aerReadGenericReg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::readFifoEmpty(){
    int devData=aerReadGenericReg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_EMPTY)==1)
    {
        fprintf(stdout,"EMPTY RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoAFull(){
    int devData=aerReadGenericReg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_AFULL)==1)
    {
        fprintf(stdout,"Almost FULL TX FIFO!!!!  \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoFull(){
    int devData=aerReadGenericReg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoEmpty(){
    int devData=aerReadGenericReg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_EMPTY)==0)
    {
        fprintf(stdout,"EMPTY TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

int aerDevManager::timeWrapCount(){
    int time;

    time = aerReadGenericReg(devDesc,STMP_REG);
    fprintf (stdout,"Times wrapping counter: %d\n",time);

    return time;
}

void aerDevManager::aerWriteGenericReg (int devDesc, unsigned int offset, unsigned int data) {
    aerGenReg_t reg;

    reg.rw = 1;
    reg.data = data;
    reg.offset = offset;
    ioctl(devDesc, AER_GEN_REG, &reg);
}

// clean up the code!! either use ioctl or this wrapper everywhere
// here we do not really need to pass devDesc to the function, as it is already in the deviceManager
unsigned int aerDevManager::aerReadGenericReg (int devDesc, unsigned int offset) {
    aerGenReg_t reg;

    reg.rw = 0;
    reg.offset = offset;
    ioctl(devDesc, AER_GEN_REG, &reg);

    return reg.data;
}


void aerDevManager::usage (void) {
    std::cerr << __FILE__ << "<even number of data to transfer>\n" << std::endl;

}

/* -------------------------------------------------------
 aerfx2_0DevManager -- handles AER IO for "legacy" iHead board and aerfx2_0 device -- the bias programming does not work for this device: use aexGrabber for this functionality
 ------------------------------------------------------- */

aerfx2_0DevManager::aerfx2_0DevManager() : deviceManager(false, AER_MAX_BUF_SIZE) {

    deviceName = "/dev/aerfx2_0";
}

bool aerfx2_0DevManager::openDevice(){

    std::cout << "name of the device: " << deviceName << std::endl;
    devDesc = ::open(deviceName.c_str(), O_RDWR | O_NONBLOCK | O_SYNC);
    if (devDesc < 0) {
        std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
        perror("");
        return false;
    }

    if(bufferedRead) this->start();

    return true;
}


