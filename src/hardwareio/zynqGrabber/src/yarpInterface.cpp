/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
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

#include "yarpInterface.h"
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

    fd = open(devicename.c_str(), O_RDONLY);
    if(fd < 0) {
        fd = open(devicename.c_str(), O_RDONLY | O_NONBLOCK);
        if(fd < 0)
            return false;
    }

    if(bufferSize > 0) this->bufferSize = bufferSize;
    if(readSize > 0) this->readSize = readSize;

    buffer1.resize(this->bufferSize);
    buffer2.resize(this->bufferSize);
    discardbuffer.resize(this->readSize);

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

device2yarp::device2yarp() {
    countAEs = 0;
    countLoss = 0;
    prevAEs = 0;
    strict = false;
    errorchecking = false;
    applyfilter = false;
    jumpcheck = false;
}

bool device2yarp::initialise(std::string moduleName, bool strict, bool check,
                             std::string deviceName, unsigned int bufferSize,
                             unsigned int readSize) {

    if(!deviceReader.initialise(deviceName, bufferSize, readSize))
        return false;

    this->errorchecking = check;

    this->strict = strict;
    if(strict) {
        std::cout << "D2Y: setting output port to strict" << std::endl;
        portvBottle.setStrict();
    } else {
        std::cout << "D2Y: setting output port to not-strict" << std::endl;
    }

    if(!portEventCount.open(moduleName + "/eventCount:o"))
        return false;

    return portvBottle.open(moduleName + "/vBottle:o");

}

void device2yarp::afterStart(bool success)
{
    if(success) deviceReader.start();
}

void device2yarp::tsjumpcheck(std::vector<unsigned char> &data, int nBytesRead)
{
    int pTS = *((int *)(data.data() + 0)) & 0x7FFFFFFF;
    for(int i = 0; i < nBytesRead; i+=8) {
        int TS =  *((int *)(data.data() + i)) & 0x7FFFFFFF;
        int dt = TS - pTS;
        if(dt < 0) {
            yError() << "stamp jump" << pTS << " " << TS;
        }
        pTS = TS;
    }
}

int device2yarp::applysaltandpepperfilter(std::vector<unsigned char> &data, int nBytesRead)
{
    int k = 0;
    for(int i = 0; i < nBytesRead; i+=8) {
        int *TS =  (int *)(data.data() + i);
        int *AE =  (int *)(data.data() + i + 4);

        int p = (*AE)&0x01;
        int x = ((*AE)>>1)&0x1FF;
        int y = ((*AE)>>10)&0xFF;
        int c = ((*AE)>>20)&0x01;
        int ts = (*TS) & 0x00FFFFFF;

        if(vfilter.check(x, y, p, c, ts)) {
            for(int j = i; j < i+8; j++) {
                data[k++] = data[j];
            }
        }

    }

    return k;

}

void  device2yarp::run() {

    while(!isStopping()) {

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
        if (nBytesRead <= 0) continue;


        bool dataError = false;

        //SMALL ERROR CHECKING BUT NOT FIXING
        if(nBytesRead % 8) {
            dataError = true;
            std::cout << "BUFFER NOT A MULTIPLE OF 8 BYTES: " <<  nBytesRead << std::endl;
        }

        if(applyfilter)
            nBytesRead = applysaltandpepperfilter(data, nBytesRead);
            
	    if(jumpcheck)
	        tsjumpcheck(data, nBytesRead);

        if(portEventCount.getOutputCount() && nBytesRead) {
            yarp::os::Bottle &ecb = portEventCount.prepare();
            ecb.clear();
            ecb.addInt(nBytesRead / 8);
            portEventCount.write();
        }

        //if we don't want or have nothing to send or there is an error finish here.
        if(!portvBottle.getOutputCount() || nBytesRead < 8)
            continue;

        //typical ZYNQ behaviour to skip error checking
        unsigned int chunksize = 80000, i = 0;
        if(!errorchecking && !dataError) {

            while((i+1) * chunksize < nBytesRead) {

                ev::vBottleMimic &vbm = portvBottle.prepare();
                vbm.setdata((const char *)data.data() + i*chunksize, chunksize);
                vStamp.update();
                portvBottle.setEnvelope(vStamp);
                portvBottle.write(strict);
                portvBottle.waitForWrite();

                i++;
            }

            ev::vBottleMimic &vbm = portvBottle.prepare();
            vbm.setdata((const char *)data.data() + i*chunksize, nBytesRead - i*chunksize);
            vStamp.update();
            portvBottle.setEnvelope(vStamp);
            portvBottle.write(strict);
            portvBottle.waitForWrite();

            continue;						//return here.
        }

        //or go through data and check for consistency
        int bstart = 0;
        int bend = 0;

        while(bend < (int)nBytesRead - 7) {

            //check validity
            int *TS =  (int *)(data.data() + bend);
            int *AE =  (int *)(data.data() + bend + 4);
            bool BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFBE00000);

            if(BITMISMATCH) {
                //send on what we have checked is not mismatched so far
                if(bend - bstart > 0) {
                    std::cerr << "BITMISMATCH in yarp2device" << std::endl;
                    std::cerr << *TS << " " << *AE << std::endl;

                    ev::vBottleMimic &vbm = portvBottle.prepare();
                    vbm.setdata((const char *)data.data()+bstart, bend-bstart);
                    countAEs += (bend - bstart) / 8;
                    vStamp.update();
                    portvBottle.setEnvelope(vStamp);
                    if(strict) portvBottle.writeStrict();
                    else portvBottle.write();
                }

                //then increment by 1 to find the next alignment
                bend++;
                bstart = bend;
            } else {
                //and then check the next two ints
                bend += 8;
            }
        }

        if(nBytesRead - bstart > 7) {
            ev::vBottleMimic &vbm = portvBottle.prepare();
            vbm.setdata((const char *)data.data()+bstart, 8*((nBytesRead-bstart)/8));
            countAEs += (nBytesRead - bstart) / 8;
            vStamp.update();
            portvBottle.setEnvelope(vStamp);
            if(strict) portvBottle.writeStrict();
            else portvBottle.write();
        }
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

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    devDesc = -1;
    flagStart = false;
    countAEs = 0;
    writtenAEs = 0;
    clockScale = 1;
}

bool yarp2device::initialise(std::string moduleName, std::string deviceName)
{

    devDesc = ::open(deviceName.c_str(), O_WRONLY);
    if(devDesc < 0)
        return false;


    fprintf(stdout,"opening port for receiving the events from yarp \n");
    this->useCallback();
    return yarp::os::BufferedPort<ev::vBottle>::open("/" + moduleName + "/vBottle:i");

}

void yarp2device::close()
{
    std::cout << "Y2D: received " << countAEs << " events from yarp" << std::endl;

    std::cout << "Y2D: written " << writtenAEs << " events to device"
              << std::endl;
    yarp::os::BufferedPort<ev::vBottle>::close();
}

void yarp2device::interrupt()
{
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

void yarp2device::onRead(ev::vBottle &bot)
{

    ev::vQueue q = bot.getAll();
    deviceData.resize(q.size()*2);  // deviceData has TS and ADDRESS, its size is double the number of events
    countAEs += q.size(); // counter for total number of events received from yarp port for the whole duration of the thread

    //std::cout<<"Y2D onRead - events queue size: "<<q.size()<<std::endl;
    //std::cout<<"Y2D onRead - deviceData size/2: "<<deviceData.size()/2<<std::endl;

    // write events on the vector of unsigned int
    // checks for empty or non valid queue????
    int i = 0;
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        ev::event<ev::AddressEvent> aep = ev::as_event<ev::AddressEvent>(*qi);
        if(!aep) continue;

        int channel = aep->getChannel();
        int polarity = aep->polarity;
        int x = aep->x;
        int y = aep->y;
        int ts = aep->stamp;

        // address
        int word0 = ((channel&0x01)<<15)|((y&0x7f)<<8)|((x&0x7f)<<1)|(polarity&0x01);

        // set intial timestamp to compute diff
        if (flagStart == false)
        {
            //std::cout<<"TS prev  :"<<tsPrev<<"us"<<std::endl;
            //std::cout<<"TS       :"<<ts<<"us"<<std::endl;
            std::cout<<"Delta TS :"<<ts - tsPrev<<"us"<<std::endl;

            //std::cout<<"Initial TS"<<ts<<"us"<<std::endl;
            tsPrev = ts;
            flagStart = true;
        }
        // timestamp difference
        int word1 = (ts - tsPrev);

        if (tsPrev > ts)
        {
            //std::cout<<"Wrap TS: ts      "<<ts<<"us"<<std::endl;
            //std::cout<<"Wrap TS: ts prev "<<tsPrev<<"us"<<std::endl;
            word1 += ev::vtsHelper::maxStamp();

            //std::cout<<"Wrap TS: max     "<<eventdriven::vtsHelper::maxStamp()<<"us"<<std::endl;
            std::cout<<"--------------- Wrap TS: Delta TS new "<<word1<<"us--------------------"<<std::endl;
            word1 = 0;

        }

        word1 = word1 * clockScale;

        deviceData[i] = word1;   //timestamp
        deviceData[i+1] = word0; //data

        i += 2;
        tsPrev = ts;

    }
    flagStart = false; // workaround for long delays due to large delta ts across bottles
    // write to the device

    unsigned int wroteData = ::write(devDesc, deviceData.data(), deviceData.size() * sizeof(unsigned int)); // wroteData is the number of data written to the FIFO (double the amount of events)

    //    std::cout<<"Y2D write: writing to device"<<deviceData.size()<< "elements"<<std::endl;
    //    std::cout<<"Y2D write: wrote to device"<<wroteData<< "elements"<<std::endl;
    if (!wroteData)
    {
        std::cout<<"Y2D write: error writing to device"<<std::endl;
        return;
    }
    else
    {
        writtenAEs += wroteData/2; // written events
        if (wroteData != deviceData.size()){
            std::cout<<"Y2D mismatch - sent events: "<<deviceData.size()<<" wrote events:"<<wroteData<<std::endl;

        } else {
            return;
        }
    }
}


