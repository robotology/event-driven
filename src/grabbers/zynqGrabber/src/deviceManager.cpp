//
//  deviceManager.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 30/07/15.
//
//

#include "iCub/deviceManager.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>



// costruttore

deviceManager::deviceManager(std::string deviceName, bool bufferedRead, unsigned int maxBufferSize){
    
    this->deviceName = deviceName;
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
    
    fpgaStat = new fpgaStatus_t;
    
    fpgaStat->biasDone      = false;
    fpgaStat->tdFifoFull    = false;
    fpgaStat->apsFifoFull   = false;
    fpgaStat->i2cTimeout    = false;
    fpgaStat->crcErr        = false;
    
}

//----------------------------------------------------------------------------------------------------
// functions for device opening
//----------------------------------------------------------------------------------------------------
//void deviceManager::setDeviceName(string deviceName) {
//    printf("saving portDevice \n");
//    portDeviceName=deviceName;
//}

bool deviceManager::openDevice(){

    if(deviceName == "/dev/aerfx2_0") {
        //opening the device
        std::cout << "name of the device: " << deviceName << std::endl;
        devDesc = ::open(deviceName.c_str(), O_RDWR | O_NONBLOCK | O_SYNC);
        if (devDesc < 0) {
            std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
            perror("");
            return false;
        }
    } else if(deviceName == "/dev/iit_hpucore") {

        //opening the device
        std::cout << "name of the device: " << deviceName << std::endl;
        devDesc = ::open(deviceName.c_str(), O_RDWR);
        if (devDesc < 0) {
            std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
            perror("");
            return false;
        }

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
    } else if(deviceName == "/dev/iit_vsctrl_l" || deviceName == "/dev/iit_vsctrl_r") {
	bufferedRead = false;
        //opening the device
        std::cout << "name of the device: " << deviceName << std::endl;
        devDesc = ::open(deviceName.c_str(), O_RDWR);
        if (devDesc < 0) {
            std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
            perror("");
            return false;
        }
    }	else {
        std::cerr << "Device Unknown to deviceManager" << std::endl;
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

    //close the device
    if(deviceName == "/dev/iit_hpucore") {
        unsigned int tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
        write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg & ~(CTRL_ENABLEINTERRUPT));
    }

    ::close(devDesc);
    std::cout <<  "closing device " << deviceName << std::endl;
    
#ifdef DEBUG
    writeDump.close();
    readDump.close();
#endif
    
}



bool deviceManager::readFifoFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::readFifoEmpty(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_EMPTY)==1)
    {
        fprintf(stdout,"EMPTY RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoAFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_AFULL)==1)
    {
        fprintf(stdout,"Almost FULL TX FIFO!!!!  \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoFull(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoEmpty(){
    int devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_EMPTY)==0)
    {
        fprintf(stdout,"EMPTY TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

int deviceManager::timeWrapCount(){
    int time;

    time = read_generic_sp2neu_reg(devDesc,STMP_REG);
    fprintf (stdout,"Times wrapping counter: %d\n",time);

    return time;
}

void deviceManager::write_generic_sp2neu_reg (int devDesc, unsigned int offset, unsigned int data) {
    sp2neu_gen_reg_t reg;
    
    reg.rw = 1;
    reg.data = data;
    reg.offset = offset;
    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
}


unsigned int deviceManager::read_generic_sp2neu_reg (int devDesc, unsigned int offset) {
    sp2neu_gen_reg_t reg;
    
    reg.rw = 0;
    reg.offset = offset;
    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
    
    return reg.data;
}


void deviceManager::usage (void) {
    std::cerr << __FILE__ << "<even number of data to transfer>\n" << std::endl;

}

//READING AND WRITING TO THE DEVICE

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


// -------------- ioctl for i2c device ---------------- //
int deviceManager::chipReset(){
    
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
    return ret;
}

int deviceManager::chipPowerDown(){
    int ret;
    ret = ioctl(devDesc, VSCTRL_SET_PWRDWN, 1);
    if (ret == -1) {
        std::cerr << "power off chip not done: ioctl error " << errno << std::endl;
    }
    return ret;
}

int deviceManager::chipPowerUp(){
    int ret;
    
    ret = ioctl(devDesc, VSCTRL_SET_PWRDWN, 0);
    if (ret == -1) {
        std::cerr << "power up chip not done: ioctl error " << errno << std::endl;
    }
    return ret;
}

int deviceManager::getFpgaRel(){
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


int deviceManager::getFpgaStatus(){
    int ret;
    unsigned int fpga_stat;

    ret = ioctl(devDesc, VSCTRL_GET_STATUS, &fpga_stat);
    
    if (ret == -1) {
        std::cerr << "read status failed: ioctl error " << errno << std::endl;
    } else {
        std::cout << "FPGA_STAT    = " << (uint8_t)fpga_stat << std::endl;
     
        fpgaStat->biasDone      = fpga_stat & 0x00000020;
        fpgaStat->tdFifoFull    = fpga_stat & 0x00000001;
        fpgaStat->apsFifoFull   = fpga_stat & 0x00000002;
        fpgaStat->i2cTimeout    = fpga_stat & 0x00000080;
        fpgaStat->crcErr        = fpga_stat & 0x00000010;
        
    }
    return ret;
}

int deviceManager::clearFpgaStatus(std::string clr){
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


int deviceManager::getFpgaInfo(){
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

int deviceManager::writeAerTimings(uint8_t ack_rel, uint8_t sample, uint8_t ack_set){
    
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


int deviceManager::writeBgTimings(uint8_t prescaler, uint8_t hold, uint8_t ck_active, uint8_t latch_setup, uint8_t latch_active){
    
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


int deviceManager::getBgTimings(){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_GET_BG_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "read Bg timings failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "Bg Timings:" << std::endl << "Prescaler = " <<  iocVsctrlArg.bg_timings.prescaler_value  << std::endl << "Setup/Hold Time = " <<  iocVsctrlArg.bg_timings.setup_hold_time  << std::endl << "Clock Active Time = " <<  iocVsctrlArg.bg_timings.clock_active_time << std::endl << "Latch Setup Time = " <<  iocVsctrlArg.bg_timings.latch_setup_time  << std::endl << "Latch Active Time = " << iocVsctrlArg.bg_timings.latch_active_time << std::endl;
        
    }
    
    
    return ret;
}

int deviceManager::getAerTimings(){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_GET_AER_TIMINGS, &iocVsctrlArg);
    
    
    if (ret == -1) {
        std::cerr << "read AER timings failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "AER Timings:" << std::endl << "Ack Release Delay = " <<  iocVsctrlArg.aer_timings.cfg_ack_rel_delay  << std::endl << "Sample Delay = " <<  iocVsctrlArg.aer_timings.cfg_sample_delay << std::endl << "Ack Set Delay = " <<  iocVsctrlArg.aer_timings.cfg_ack_set_delay << std::endl;
        
    }
    
    
    return ret;
}


int deviceManager::initDevice(std::string chipName){
    
    int ret = 0;

    if (chipName == "atis"){
        ret = ioctl(devDesc, VSCTRL_INIT_DEV, CHIP_ATIS);
        if (ret == -1) {
            std::cerr << "init device failed: ioctl error " << errno << std::endl;
        } else {
            
            std::cout << "Initializing device as ATIS" << std::endl;
            
        }
        
    } else if (chipName == "dvs"){
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


int deviceManager::writeGPORegister(uint32_t data){
    
    int ret;
    ret = ioctl(devDesc, VSCTRL_SET_GPO, &data);
    if (ret == -1) {
        std::cerr << "read GPO register failed: ioctl error " << errno << std::endl;
    } else {
        
        std::cout << "GPO register = " << iocVsctrlArg.regs.data << std::endl;
        
    }
    return ret;
    
}

int deviceManager::readGPORegister(){
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



