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
    //stop the read thread
    if(!stop())
        std::cerr << "Thread did not stop correctly" << std::endl;

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

int deviceManager::writeDevice(std::vector<unsigned int> &deviceData){
    /*
     if(writeFifoAFull()){
     std::cout<<"Y2D write: warning fifo almost full"<<std::endl;
     }
     
     if(writeFifoFull()){
     std::cout<<"Y2D write: error fifo full"<<std::endl;
     }
     */
    int written =0;
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

        std::cout << "Buffered Read" << std::endl;
        //safely copy the data into the accessBuffer and reset the readCount
        safety.wait();

        //switch the buffer the read into
        std::vector<char> * temp = readBuffer;
        readBuffer = accessBuffer;
        accessBuffer = temp;

        //reset the filling position
        nBytesRead = readCount;
        readCount = 0;
        safety.post();

    } else {

        std::cout << "Direct Read" << std::endl;
        nBytesRead = ::read(devDesc, readBuffer->data(), maxBufferSize);
        if(nBytesRead < 0 && errno != EAGAIN) perror("perror: ");

    }

    //and return the accessBuffer
    return *accessBuffer;

}


void deviceManager::run(void)
{

    while(!isStopping()) {

        safety.wait();

        //std::cout << "Reading events from FIFO" << std::endl;

        //read SHOULD be a blocking call
        int r = ::read(devDesc, readBuffer->data() + readCount,
                       maxBufferSize - readCount);

        //std::cout << readBuffer << " " <<  readCount << " " << maxBufferSize << std::endl;

        if(r > 0) {
            //std::cout << "Successful Read" << std::endl;
            readCount += r;
        } else if(r < 0 && errno != EAGAIN) {
            std::cerr << "Error reading from " << deviceName << std::endl;
            perror("perror: ");
            std::cerr << "readCount: " << readCount << "MaxBuffer: "
                      << maxBufferSize << std::endl;
        }

        if(readCount >= maxBufferSize) {
            std::cerr << "We reached maximum buffer! " << readCount << "/"
                      << maxBufferSize << std::endl;
        }

        //std::cout << "Leaving safe zone" << std::endl;
        safety.post();
        //yarp::os::Time::delay(0.00001);


        //std::cout << "Done with FIFO" << std::endl;
    }
}
/* --------------------------------------------------------------------------------
 
    vsctrlDevManager -- to send biases and write registers -- specific to sensors
 
 ---------------------------------------------------------------------------------- */



vsctrlDevManager::vsctrlDevManager(std::string channel, std::string chip) : deviceManager(false, VSCTRL_MAX_BUF_SIZE) {

    if (channel == "left"){
      
        this->deviceName = "/dev/iit_vsctrl_l";
        
    } else if (channel == "right"){
        
        this->deviceName = "/dev/iit_vsctrl_r";
        
    } else {
    
        std::cout << "vsctrl error: unrecognised channel" << std::endl;
    
    }

    fpgaStat.biasDone      = false;
    fpgaStat.tdFifoFull    = false;
    fpgaStat.apsFifoFull   = false;
    fpgaStat.i2cTimeout    = false;
    fpgaStat.crcErr        = false;
    
    
    // default biases
    this -> channel = channel;
    this -> chipName = chip;

};

bool vsctrlDevManager::openDevice(){
    
    bufferedRead = false;
    //opening the device
    
    bool ret = deviceManager::openDevice();
    
    // clear fpga registers
    clearFpgaStatus("biasDone");
    clearFpgaStatus("tdFifoFull");
    clearFpgaStatus("apsFifoFull");
    clearFpgaStatus("i2cTimeout");
    clearFpgaStatus("crcErr");
    
    initDevice();
    programBiases();
    
    return ret;

}

void vsctrlDevManager::closeDevice(){
    
    chipPowerDown();
    deviceManager::closeDevice();
    
}

// --- write biases to device --- //

bool vsctrlDevManager::programBiases(){
    
    std::vector<unsigned int> vBiases = prepareBiases();
    
    clearFpgaStatus("biasDone");
    chipPowerDown();

    if (writeDevice(vBiases) <= 0) {
        std::cout << "Bias write: error writing to device" << std::endl;
        return false;
    }
    
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

    return true;
    
}

bool vsctrlDevManager::setBias(yarp::os::Bottle bias)
{
    if(bias.isNull())
        return false;

    this->bias = bias;
    return true;

}

// --- change the value of a single bias --- //
bool vsctrlDevManager::setBias(std::string biasName, unsigned int biasValue) {

    bias.find(biasName) = yarp::os::Value((int)biasValue).asInt();
    if(bias.find(biasName).asInt() != biasValue) {
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

unsigned int vsctrlDevManager::getBias(std::string biasName) {
    
    return bias.find(biasName).asInt();

}

void vsctrlDevManager::printBiases(){

    std::cout << bias.toString() << std::endl;
    
}

// -------------- ioctl for i2c device ---------------- //
int vsctrlDevManager::chipReset(){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_RESET_ARRAY, 0);
    if (ret == -1) {
        std::cerr << "reset chip not done: ioctl error " << errno << std::endl;
        return ret;
    }
    sleep(1); // sec
    ret = ioctl(devDesc, VSCTRL_RESET_ARRAY, 1);
    if (ret == -1) {
        std::cerr << "reset release not done: ioctl error " << errno << std::endl;
        return ret;
    }
    else {
        
        std::cout << "reset " << chipName << channel << std::endl;
    }
    
    return ret;
}

int vsctrlDevManager::chipPowerDown(){
    int ret;
    ret = ioctl(devDesc, VSCTRL_SET_PWRDWN, 1);
    if (ret == -1) {
        std::cerr << "power off chip not done: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "power off " << chipName << channel << std::endl;
    }
    return ret;
}

int vsctrlDevManager::chipPowerUp(){
    int ret;
    
    ret = ioctl(devDesc, VSCTRL_SET_PWRDWN, 0);
    if (ret == -1) {
        std::cerr << "power on chip not done: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "power on " << chipName << channel << std::endl;
    }
    return ret;
}

int vsctrlDevManager::getFpgaRel(){
    int ret;
    unsigned int fpga_rel;

    ret = ioctl(devDesc, VSCTRL_GET_FPGAREL, &fpga_rel);
    
    if (ret == -1) {
        std::cerr << "read release failed: ioctl error " << errno << std::endl;
    } else {
        std::cout << "FPGA_REL    = " << (uint8_t)fpga_rel << std::endl;

    }
    return ret;
}


int vsctrlDevManager::getFpgaStatus(){
    int ret;
    unsigned int fpga_stat;

    ret = ioctl(devDesc, VSCTRL_GET_STATUS, &fpga_stat);
    
    if (ret == -1) {
        std::cerr << "read status failed: ioctl error " << errno << std::endl;
    } else {
        std::cout << "FPGA_STAT    = " << fpga_stat << std::endl;

        fpgaStat.biasDone      = fpga_stat & 0x00000020;
        fpgaStat.tdFifoFull    = fpga_stat & 0x00000001;
        fpgaStat.apsFifoFull   = fpga_stat & 0x00000002;
        fpgaStat.i2cTimeout    = fpga_stat & 0x00000080;
        fpgaStat.crcErr        = fpga_stat & 0x00000010;
        
    }
    return ret;
}

int vsctrlDevManager::clearFpgaStatus(std::string clr){
    int ret;
    unsigned int clrStatus;
    
    if (clr == "biasDone") {
        clrStatus = 0x00000020;
    } else if (clr == "tdFifoFull") {
        clrStatus = 0x00000001;
    } else if (clr == "apsFifoFull"){
        clrStatus = 0x00000002;
    } else if (clr == "i2cTimeOut") {
        clrStatus = 0x00000080;
    } else if (clr == "crcErr"){
        clrStatus = 0x00000010;
    }
    
    ret = ioctl(devDesc, VSCTRL_CLR_STATUS, &clrStatus);
    
    if (ret == -1) {
        std::cerr << "clear status failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "cleared field" << std::endl;
    }
    return ret;
}


int vsctrlDevManager::getFpgaInfo(){
    int ret;
    unsigned int fpga_info;
    
    ret = ioctl(devDesc, VSCTRL_GET_INFO, &fpga_info);
    
    if (ret == -1) {
        std::cerr << "read info failed: ioctl error " << errno << std::endl;
    } else {
        std::cout << "FPGA_INFO    = " << (uint8_t)fpga_info << std::endl;
        
    }
    return ret;
}

int vsctrlDevManager::writeAerTimings(uint8_t ack_rel, uint8_t sample, uint8_t ack_set){
    
    int ret;
    iocVsctrlArg.aer_timings.cfg_ack_rel_delay = ack_rel;
    iocVsctrlArg.aer_timings.cfg_sample_delay  = sample;
    iocVsctrlArg.aer_timings.cfg_ack_set_delay = ack_set;
    ret = ioctl(devDesc, VSCTRL_SET_AER_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "write AER timings failed: ioctl error " << errno << std::endl;
    }
    
    return ret;
}


int vsctrlDevManager::writeBgTimings(uint8_t prescaler, uint8_t hold, uint8_t ck_active, uint8_t latch_setup, uint8_t latch_active){
    
    int ret;
    iocVsctrlArg.bg_timings.prescaler_value   = prescaler;
    iocVsctrlArg.bg_timings.setup_hold_time   = hold;
    iocVsctrlArg.bg_timings.clock_active_time = ck_active;
    iocVsctrlArg.bg_timings.latch_setup_time  = latch_setup;
    iocVsctrlArg.bg_timings.latch_active_time = latch_active;
    ret = ioctl(devDesc, VSCTRL_SET_BG_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "write AER timings failed: ioctl error " << errno << std::endl;
    }
    
    return ret;
}


int vsctrlDevManager::getBgTimings(){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_GET_BG_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "read Bg timings failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "Bg Timings:" << std::endl << "Prescaler = " <<  iocVsctrlArg.bg_timings.prescaler_value  << std::endl << "Setup/Hold Time = " <<  iocVsctrlArg.bg_timings.setup_hold_time  << std::endl << "Clock Active Time = " <<  iocVsctrlArg.bg_timings.clock_active_time << std::endl << "Latch Setup Time = " <<  iocVsctrlArg.bg_timings.latch_setup_time  << std::endl << "Latch Active Time = " << iocVsctrlArg.bg_timings.latch_active_time << std::endl;
        
    }
    
    
    return ret;
}

int vsctrlDevManager::getAerTimings(){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_GET_AER_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "read AER timings failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "AER Timings:" << std::endl << "Ack Release Delay = " <<  iocVsctrlArg.aer_timings.cfg_ack_rel_delay  << std::endl << "Sample Delay = " <<  iocVsctrlArg.aer_timings.cfg_sample_delay << std::endl << "Ack Set Delay = " <<  iocVsctrlArg.aer_timings.cfg_ack_set_delay << std::endl;
        
    }
    
    
    return ret;
}


int vsctrlDevManager::initDevice(){
    
    int ret = -1;

    if (chipName == "ATIS"){
        ret = ioctl(devDesc, VSCTRL_INIT_DEV, CHIP_ATIS);
        if (ret == -1) {
            std::cerr << "init device failed: ioctl error " << errno << std::endl;
        } else {
            
            std::cout << "Initializing device as ATIS" << std::endl;
            
        }
        
    } else if (chipName == "DVS"){
        ret = ioctl(devDesc, VSCTRL_INIT_DEV, CHIP_DVS);
        if (ret == -1) {
            std::cerr << "init device failed: ioctl error " << errno << std::endl;
        } else {
            
            std::cout << "Initializing device as DVS" << std::endl;
            
        }
        
    } else {
        std::cout << "Error: " << chipName << "unsupported" << std::endl;
        
    }
    
    return ret;
}


int vsctrlDevManager::writeGPORegister(uint32_t data){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_SET_GPO, &data);
    if (ret == -1) {
        std::cerr << "read GPO register failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "GPO register = " << iocVsctrlArg.regs.data << std::endl;
        
    }
    return ret;
    
}

int vsctrlDevManager::readGPORegister(){
    int ret;

    iocVsctrlArg.regs.addr = VSCTRL_GPO_ADDR;
    iocVsctrlArg.regs.rw   = VSCTRL_IOC_READ;
    iocVsctrlArg.regs.data = 0x00; // erase any previous content
    ret = ioctl(devDesc, VSCTRL_GEN_REG_ACCESS, &iocVsctrlArg); // read from register
    if (ret == -1) {
        std::cerr << "read GPO register failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "GPO register = " << iocVsctrlArg.regs.data << std::endl;
        
    }
    
    return ret;
    
}

/* -----------------------------------------------------------------
    aerDevManager -- to handle AER IO: read events from sensors and spinnaker and write events to spinnaker
----------------------------------------------------------------- */
aerDevManager::aerDevManager(std::string dev) : deviceManager(true, AER_MAX_BUF_SIZE) {
    
    if (dev == "zynq_spinn")
        {
        
            this->deviceName = "/dev/iit_sp2neu";
    
        } else if (dev == "zynq_sens")
        {
            
            this->deviceName = "/dev/iit_hpucore";
        
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
        
        ioctl(devDesc, SP2NEU_VERSION, &version);
        
        hw_major = (version & 0xF0) >> 4;
        hw_minor = (version & 0x0F);
        stringa[3]=0;
        
        for (i=0; i<3; i++) {
            stringa[i] = (version&0xFF000000) >> 24;
            version = version << 8;
        }
        fprintf(stderr, "Identified: %s version %d.%d\r\n\r\n", stringa, hw_major, hw_minor);
        
        // Write the WrapTimeStamp register with any value if you want to clear it
        //write_generic_sp2neu_reg(fp,STMP_REG,0);
        fprintf(stderr, "Times wrapping counter: %d\n", read_generic_sp2neu_reg(devDesc, STMP_REG));
        
        // Enable Time wrapping interrupt
        //write_generic_sp2neu_reg(devDesc, MASK_REG, MSK_TIMEWRAPPING | MSK_TX_DUMPMODE | MSK_RX_PAR_ERR | MSK_RX_MOD_ERR);
        write_generic_sp2neu_reg(devDesc, MASK_REG, MSK_TIMEWRAPPING | MSK_RX_PAR_ERR);
        
        // Flush FIFOs
        tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
        write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_FLUSHFIFO); // | CTRL_ENABLEIP);
        
        // Start IP in LoopBack
        tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
        write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | (CTRL_ENABLEINTERRUPT));// | CTRL_ENABLE_FAR_LBCK));
        
        ioctl(devDesc, SP2NEU_SET_LOC_LBCK, 0);
        ioctl(devDesc, SP2NEU_SET_FAR_LBCK, 0);
        ioctl(devDesc, SP2NEU_SET_REM_LBCK, 0);
    }
    return ret;
}

void aerDevManager::closeDevice(){

    deviceManager::closeDevice();
    unsigned int tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg & ~(CTRL_ENABLEINTERRUPT));

    
}

bool aerDevManager::readFifoFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::readFifoEmpty(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_EMPTY)==1)
    {
        fprintf(stdout,"EMPTY RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoAFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_AFULL)==1)
    {
        fprintf(stdout,"Almost FULL TX FIFO!!!!  \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool aerDevManager::writeFifoEmpty(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_EMPTY)==0)
    {
        fprintf(stdout,"EMPTY TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

int aerDevManager::timeWrapCount(){
    int time;
    
    time = read_generic_sp2neu_reg(devDesc,STMP_REG);
    fprintf (stdout,"Times wrapping counter: %d\n",time);
    
    return time;
}

void aerDevManager::write_generic_sp2neu_reg (int devDesc, unsigned int offset, unsigned int data) {
    sp2neu_gen_reg_t reg;
    
    reg.rw = 1;
    reg.data = data;
    reg.offset = offset;
    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
}


unsigned int aerDevManager::read_generic_sp2neu_reg (int devDesc, unsigned int offset) {
    sp2neu_gen_reg_t reg;
    
    reg.rw = 0;
    reg.offset = offset;
    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
    
    return reg.data;
}


void aerDevManager::usage (void) {
    std::cerr << __FILE__ << "<even number of data to transfer>\n" << std::endl;
    
}

/* -------------------------------------------------------
    aerfx2_0DevManager -- handles AER IO for "legacy" iHead board and aerfx2_0 device -- the bias programming does not work for this device: use aexGrabber for this functionality
------------------------------------------------------- */

aerfx2_0DevManager::aerfx2_0DevManager() : deviceManager(true, AER_MAX_BUF_SIZE) {
    
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

    return true;
}


