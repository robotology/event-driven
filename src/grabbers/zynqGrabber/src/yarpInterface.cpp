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
    bufferSize = 5000;  //events
    readSize = 128;     //events

    //internal variables/storage
    fd = -1;
    readCount = 0;

    buffer1.resize(bufferSize);
    buffer2.resize(bufferSize);
    discardbuffer.resize(readSize);

    readBuffer = &buffer1;
    accessBuffer = &buffer2;

    bufferedreadwaiting = false;

}

bool vDevReadBuffer::initialise(std::string devicename,
                                unsigned int bufferSize,
                                unsigned int readSize,
                                unsigned int bytesperevent)
{

    fd = open(devicename.c_str(), O_RDWR);
    if(fd < 0) return false;

    if(bufferSize > 0) {
        this->bufferSize = bufferSize;
        buffer1.resize(bufferSize);
        buffer2.resize(bufferSize);
    }

    if(readSize > 0) {
        this->readSize = readSize;
        discardbuffer.resize(readSize);
    }
    this->bytestoread = readSize * bytesperevent;

    return true;

}

void vDevReadBuffer::run()
{

    signal.check();

    while(!isStopping()) {

        safety.wait();

        int r = 0;
        if(readCount >= bufferSize) {
            //we have reached maximum software buffer - read from the HW but
            //just discard the result.
            if(!msgflag) {
                std::cout << "Buffer Full - events discarded" << std::endl;
                msgflag = true;
            }
            r = read(fd, discardbuffer.data(), bytestoread);
        } else {
            //we read and fill up the buffer
            msgflag = false;
            r = read(fd, readBuffer->data() + readCount, std::min(bufferSize - readCount, bytestoread));
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

unsigned int vDevReadBuffer::getBuffer(std::vector<int32_t> *bufferpointer)
{

    //safely copy the data into the accessBuffer and reset the readCount
    bufferedreadwaiting = true;
    safety.wait();
    bufferedreadwaiting = false;

    //switch the buffer the read into
    bufferpointer = readBuffer;
    readBuffer = accessBuffer;
    accessBuffer = bufferpointer;

    //reset the filling position
    unsigned int nBytesRead = readCount;
    readCount = 0;
    safety.post();
    signal.check();
    signal.post(); //tell the other thread we are done

    return nBytesRead;

}

/******************************************************************************/
//device2yarp
/******************************************************************************/

device2yarp::device2yarp() : RateThread(THRATE) {
    countAEs = 0;
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

void device2yarp::afterStart()
{
    deviceReader.start();
}

void  device2yarp::run() {

    //display an output to let everyone know we are still working.
    if(yarp::os::Time::now() - prevTS > 5) {
        std::cout << "ZynqGrabber running happily: " << countAEs
                  << " events. ";
        std::cout << (countAEs - prevAEs) / 5.0 << "v / second" << std::endl;
        prevTS = yarp::os::Time::now();
        prevAEs = countAEs;
    }

    //get the data from the device read thread

    std::vector<int32_t> *data = 0;
    int nBytesRead = deviceReader.getBuffer(data);
    if (nBytesRead <= 0) return;

    bool dataError = false;

    //SMALL ERROR CHECKING BUT NOT FIXING
    if(nBytesRead % 8) {
        dataError = true;
        std::cout << "BUFFER NOT A MULTIPLE OF 8 BYTES: " <<  nBytesRead << std::endl;
    }

    countAEs += nBytesRead / 8;
    if(portvBottle.getOutputCount() && nBytesRead > 8 && !dataError) {
        emorph::vBottleMimic &vbm = portvBottle.prepare();
        vbm.setdata((const char *)(data->data()), nBytesRead);
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

