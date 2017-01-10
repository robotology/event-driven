#include "iCub/yarpInterface.h"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/******************************************************************************/
//vDevReadBuffer
/******************************************************************************/
vDevReadBuffer::vDevReadBuffer()
{
    //parameters
    bufferSize = 800000;  //bytes
    readSize = 1024;      //bytes

    //internal variables/storage
    fd = -1;
    readCount = 0;
    lossCount = 0;

    bufferedreadwaiting = false;

}

bool vDevReadBuffer::initialise(std::string devicename,
                                unsigned int bufferSize,
                                unsigned int readSize)
{

    fd = open(devicename.c_str(), O_RDWR);
    if(fd < 0) return false;

    if(bufferSize > 0) this->bufferSize = bufferSize;
    if(readSize > 0) this->readSize = readSize;
    
	buffer1.resize(bufferSize);
	buffer2.resize(bufferSize);
	discardbuffer.resize(readSize);
    
    readBuffer = &buffer1;
    accessBuffer = &buffer2;

    return true;

}

void vDevReadBuffer::run()
{
	
	if(fd < 0) {
		std::cout << "Event Reading Device uninistialised. Please run "
			"initialisation before starting the thread" << std::endl;
		return;
	}

    signal.check();

    while(!isStopping()) {

        safety.wait();

        int r = 0;
        if(readCount >= bufferSize) {
            //we have reached maximum software buffer - read from the HW but
            //just discard the result.
            r = read(fd, discardbuffer.data(), readSize);
            if(r > 0) lossCount += r;
        } else {
            //we read and fill up the buffer
            r = read(fd, readBuffer->data() + readCount, std::min(bufferSize - readCount, readSize));
            if(r > 0) readCount += r;
        }

        if(r < 0 && errno != EAGAIN) {
            perror("Error reading events: ");
        }

        safety.post();
        if(bufferedreadwaiting) {
            //the other thread is read to read
            signal.wait(); //wait for it to do the read
        }

    }


}

void vDevReadBuffer::threadRelease()
{
    if(fd) close(fd);
}

std::vector<unsigned char>& vDevReadBuffer::getBuffer(unsigned int &nBytesRead, unsigned int &nBytesLost)
{

    //safely copy the data into the accessBuffer and reset the readCount
    bufferedreadwaiting = true;
    safety.wait();
    bufferedreadwaiting = false;

    //switch the buffer the read into
    std::vector<unsigned char> *temp;
    temp = readBuffer;
    readBuffer = accessBuffer;
    accessBuffer = temp;   

    //reset the filling position
    nBytesRead = readCount;
    nBytesLost = lossCount;
    readCount = 0;
    lossCount = 0;
    
    //send the correct signals to restart the grabbing thread
    safety.post();
    signal.check();
    signal.post(); //tell the other thread we are done

    return *accessBuffer;

}

/******************************************************************************/
//device2yarp
/******************************************************************************/

device2yarp::device2yarp() : RateThread(THRATE) {
    countAEs = 0;
    countLoss = 0;
    strict = false;
}

bool device2yarp::initialise(std::string moduleName, bool strict,
                             std::string deviceName, unsigned int bufferSize,
                             unsigned int readSize) {

    if(!deviceReader.initialise(deviceName, bufferSize, readSize))
        return false;

    this->strict = strict;
    if(strict) {
        std::cout << "D2Y: setting output port to strict" << std::endl;
        portvBottle.setStrict();
    } else {
        std::cout << "D2Y: setting output port to not-strict" << std::endl;
    }

    return portvBottle.open("/" + moduleName + "/vBottle:o");

}

void device2yarp::afterStart(bool success)
{
    if(success) deviceReader.start();
}

void  device2yarp::run() {

    //display an output to let everyone know we are still working.
    if(yarp::os::Time::now() - prevTS > 5.0) {
        std::cout << "Event grabber running happily: ";
        std::cout << (int)((countAEs - prevAEs) / 5.0) << " v/s" << std::endl;
        std::cout << "                         Lost: ";
        std::cout << (int)(countLoss / 5.0) << " v/s" << std::endl;
        countLoss = 0;
        prevTS = yarp::os::Time::now();
        prevAEs = countAEs;
    }

    //get the data from the device read thread
	unsigned int nBytesRead, nBytesLost;
    std::vector<unsigned char> &data = deviceReader.getBuffer(nBytesRead, nBytesLost);
    countAEs += nBytesRead / 8;
    countLoss += nBytesLost / 8;
    if (nBytesRead <= 0) return;
    

    bool dataError = false;

    //SMALL ERROR CHECKING BUT NOT FIXING
    if(nBytesRead % 8) {
        dataError = true;
        std::cout << "BUFFER NOT A MULTIPLE OF 8 BYTES: " <<  nBytesRead << std::endl;
    }

    
    if(portvBottle.getOutputCount() && nBytesRead > 8 && !dataError) {
        emorph::vBottleMimic &vbm = portvBottle.prepare();
        vbm.setdata((const char *)data.data(), nBytesRead);
        vStamp.update();
        portvBottle.setEnvelope(vStamp);
        if(strict) portvBottle.writeStrict();
        else portvBottle.write();
    }

}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;
    std::cout << "Closing device reader (Could be stuck in a read call!)"
              << std::endl;
    deviceReader.stop();

    portvBottle.close();

}

