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

//using namespace yarp::os;
//using namespace yarp::sig;
using namespace std;


#define MAGIC_NUM 100
#define SP2NEU_VERSION         _IOR (MAGIC_NUM,  7, void *)
#define SP2NEU_TIMESTAMP       _IOR (MAGIC_NUM,  8, void *)
#define SP2NEU_GEN_REG         _IOWR(MAGIC_NUM,  6, void *)
#define SP2NEU_SET_LOC_LBCK    _IOW (MAGIC_NUM, 10, void *)
#define SP2NEU_SET_REM_LBCK    _IOW (MAGIC_NUM, 11, void *)
#define SP2NEU_SET_FAR_LBCK    _IOW (MAGIC_NUM, 12, void *)


#define CTRL_REG     0x00
#define RXDATA_REG   0x08
#define RXTIME_REG   0x0C
#define TXDATA_REG   0x10
#define DMA_REG      0x14
#define RAWI_REG     0x18
#define IRQ_REG      0x1C
#define MASK_REG     0x20
#define STMP_REG     0x28
#define ID_REG       0x5c

// CTRL register bit field
//#define CTRL_ENABLEIP 0x00000001
#define CTRL_ENABLEINTERRUPT 0x00000004
#define CTRL_FLUSHFIFO       0x00000010
#define CTRL_ENABLE_REM_LBCK 0x01000000
#define CTRL_ENABLE_LOC_LBCK 0x02000000
#define CTRL_ENABLE_FAR_LBCK 0x04000000

// INterrupt Mask register bit field
#define MSK_RXBUF_EMPTY  0x00000001
#define MSK_RXBUF_AEMPTY 0x00000002
#define MSK_RXBUF_FULL   0x00000004
#define MSK_TXBUF_EMPTY  0x00000008
#define MSK_TXBUF_AFULL  0x00000010
#define MSK_TXBUF_FULL   0x00000020
#define MSK_TIMEWRAPPING 0x00000080
#define MSK_RXBUF_READY  0x00000100
#define MSK_RX_NOT_EMPTY 0x00000200
#define MSK_TX_DUMPMODE  0x00001000
#define MSK_RX_PAR_ERR   0x00002000
#define MSK_RX_MOD_ERR   0x00004000
//#define MASK_RX_EMPTY    0x01
//#define MASK_RX_FULL     0x04


// costruttore

deviceManager::deviceManager(string deviceName){
    
    this->deviceName = deviceName;
    
}

//----------------------------------------------------------------------------------------------------
// functions for device opening
//----------------------------------------------------------------------------------------------------
//void deviceManager::setDeviceName(string deviceName) {
//    printf("saving portDevice \n");
//    portDeviceName=deviceName;
//}

bool deviceManager::readFifoFull(){
    devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::readFifoEmpty(){
    devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_RXBUF_EMPTY)==0)
    {
        fprintf(stdout,"EMPTY RX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoAFull(){
    devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_AFULL)==1)
    {
        fprintf(stdout,"Almost FULL TX FIFO!!!!  \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoFull(){
    devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
    if((devData & MSK_TXBUF_FULL)==1)
    {
        fprintf(stdout,"FULL TX FIFO!!!!   \n");
        return true;
    }
    return false;
}

bool deviceManager::writeFifoEmpty(){
    devData=read_generic_sp2neu_reg(devDesc,RAWI_REG);
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

int deviceManager::writeDevice(std::vector<unsigned int> &deviceData){
    int devData = ::write(devDesc, (char *)deviceData.data(), 2*deviceData.size()*sizeof(unsigned int));
    return devData;
    
}

int deviceManager::readDevice(std::vector<unsigned int> &deviceData){
    int devData = ::read(devDesc, (char *)(deviceData.data()), deviceData.size()*sizeof(unsigned int));
    if (devData < 0){
        fprintf(stdout,"error reading from device\n");
        //if (errno != EAGAIN) {
        //    printf("error reading from spinn2neu: %d\n", (int)errno);
        //    perror("perror:");
        //}
        //if errno == EAGAIN ther is just no data to read just now
        // we are using a non-blocking call so we need to return and wait for
        // the thread to run again.
        return devData;
    } else if(devData == 0) {
        // everything ok, no data available, just call the run again later
        return devData;
    }
return devData;
    
}

void deviceManager::closeDevice(){
    ::close(devDesc);
    fprintf(stdout, "closing device %s \n",deviceName.c_str());
}

bool deviceManager::openDevice(){
    //opening the device
    fprintf(stdout,"name of the file buffer: %s\n", deviceName.c_str());
    devDesc = open(deviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (devDesc < 0) {
        printf("Cannot open device file: %s \n",deviceName.c_str());
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
    fprintf(stderr, "\r\nIdentified: %s version %d.%d\r\n\r\n", stringa, hw_major, hw_minor);
    
    // Write the WrapTimeStamp register with any value if you want to clear it
    //write_generic_sp2neu_reg(fp,STMP_REG,0);
    fprintf(stderr, "Times wrapping counter: %d\n", read_generic_sp2neu_reg(devDesc, STMP_REG));
    
    // Enable Time wrapping interrupt
    write_generic_sp2neu_reg(devDesc, MASK_REG, MSK_TIMEWRAPPING | MSK_TX_DUMPMODE | MSK_RX_PAR_ERR | MSK_RX_MOD_ERR);
    
    // Flush FIFOs
    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_FLUSHFIFO); // | CTRL_ENABLEIP);
    
    // Start IP in LoopBack
    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | (CTRL_ENABLEINTERRUPT | CTRL_ENABLE_FAR_LBCK));
    //    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_ENABLE_FAR_LBCK);
    
    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_ENABLE_FAR_LBCK);
    
    return true;
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
    fprintf (stderr, "%s <even number of data to transfer>\n", __FILE__);
    //exit(1);
}

