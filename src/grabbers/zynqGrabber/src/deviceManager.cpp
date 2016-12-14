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

	if(bufferedRead) {
		signal.post();
		//stop the read thread
		if(!stop())
			std::cerr << "Thread did not stop correctly" << std::endl;
		else
			std::cout << "Thread stopped. Continuing shutdown." << std::endl;
	}

    if(devDesc > 0)
		::close(devDesc);
		
    std::cout <<  "device closed" << deviceName << std::endl;

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
            std::cerr << "readCount: " << readCount << " MaxBuffer: "
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


/* -----------------------------------------------------------------
 aerDevManager -- to handle AER IO: read events from sensors and spinnaker and write events to spinnaker
 ----------------------------------------------------------------- */
aerDevManager::aerDevManager(std::string dev, int clockPeriod, std::string loopBack) : deviceManager(true, AER_MAX_BUF_SIZE) {

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
        //aerWriteGenericReg(devDesc, MASK_REG, MSK_RX_PAER_ERR);
        //std::cout << "Enabled FIFO Full interrupt " << std::endl;

        //tmp_reg = aerReadGenericReg(devDesc, CTRL_REG);

        // Flush FIFOs
        //aerWriteGenericReg(devDesc, CTRL_REG, tmp_reg | CTRL_FLUSHFIFO | (CTRL_ENABLEINTERRUPT));
        //std::cout << "FIFO Flushed and Interrupt enabled " << std::endl;

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


