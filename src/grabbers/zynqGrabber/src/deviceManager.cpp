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
#include <fstream>



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

deviceManager::deviceManager(std::string deviceName, unsigned int maxBufferSize){
    
    this->deviceName = deviceName;
    this->maxBufferSize = maxBufferSize;
    readCount = 0;

    //allocate the memory for the readBuffer
    readBuffer.resize(maxBufferSize);
    //put the accessBuffer into a memory location which has the potential to
    //expand to the maximum size
    accessBuffer.reserve(maxBufferSize);
#ifdef DEBUG
    writeDump.open("/tmp/writeDump.txt");
    readDump.open("/tmp/readDump.txt");
#endif
    
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
        devDesc = ::open(deviceName.c_str(), O_RDWR | O_NONBLOCK);
        if (devDesc < 0) {
            std::cerr << "Cannot open device file: " << deviceName << " " << devDesc << " : ";
            perror("");
            return false;
        }
    }

    if(deviceName == "/dev/spinn2neu") {

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
    }

    //start the reading thread
    start();

    return true;
}

void deviceManager::closeDevice()
{
    //stop the read thread
    if(!stop())
        std::cerr << "Thread did not stop correctly" << std::endl;

    //close the device
    if(deviceName == "/dev/spinn2neu") {
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
        if (ret<0){
            break;
        }
        written += ret;
    }
    
#ifdef DEBUG
    
    writeDump << deviceData << std::endl;
    
#endif
    
    return written/sizeof(unsigned int);

}

std::vector<char> &deviceManager::readDevice()
{

#ifdef DEBUG
    for(int i = 0; i < readCount; i++) {
        for(int j = 0; j < 8; j++) {
            if( (1 << j) & readBuffer[i]) {
                readDump << "1";
            } else {
                readDump << "0";
            }
        }
        readDump << " ";
    }
    if(readCount) readDump << std::endl;
#endif
       
    //safely copy the data into the accessBuffer and reset the readCount
    safety.wait();
    accessBuffer.resize(readCount);
    for(int i = 0; i < readCount; i++) {
        accessBuffer[i] = readBuffer[i];
    }
    readCount = 0;
    safety.post();

    

    
    //and return the accessBuffer
    return accessBuffer;

}


void deviceManager::run(void)
{

    while(!isStopping()) {
        //read is a blocking call
        safety.wait();

        int r = ::read(devDesc, readBuffer.data() + readCount, 
                    maxBufferSize - readCount);

        if(r < 0) {
            //std::cerr << "Error reading from " << deviceName << std::endl;
            //perror("perror: ");
        } else {
            readCount += r;
        }

        safety.post();

        if(readCount >= maxBufferSize) {
            std::cerr << "We reached maximum buffer! " << readCount << "/"
                      << maxBufferSize << std::endl;
        }
    }
}

//int deviceManager::readDevice(std::vector<unsigned int> &deviceData){
//    int devData = ::read(devDesc, (char *)(deviceData.data()), deviceData.size()*sizeof(unsigned int));
//    if (devData < 0){
//        fprintf(stdout,"error reading from device\n");
//        if (errno != EAGAIN) {
//            printf("error reading from spinn2neu: %d\n", (int)errno);
//            perror("perror:");
//        }
//        //if errno == EAGAIN ther is just no data to read just now
//        // we are using a non-blocking call so we need to return and wait for
//        // the thread to run again.
//    } else if(devData == 0) {
//        // everything ok, no data available, just call the run again later
//    }

//return devData;

//}


