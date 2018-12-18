/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "yarpInterface.h"

#include "atis_events_stream.h"

#include "i_events_stream.h"



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
    readCount = 0;
    lossCount = 0;

    bufferedreadwaiting = false;

}



long vDevReadBuffer::getEventChunk(unsigned char* target)
{
    long n_TD = 0;
    uint64_t *target64 = (uint64_t*) target;
    uint32_t ts = 0;
    //std::cout << "n_events:" << n_events << std::endl;
    while (n_TD<512) {
        if (n_events == 0) {
            if (!stream->poll_buffer()) {
        //usleep(1);
                break;
            }
            ev_buffer = (uint32_t*)stream->decode_buffer(n_events);
        }
        EventBase * evbase = reinterpret_cast<EventBase*>(ev_buffer);
        if(static_cast<Event_Types_underlying>(Event_Types::EVT_TIME_HIGH) == evbase->type) {
            Event_EVT_TIME_HIGH ev = *reinterpret_cast<Event_EVT_TIME_HIGH*>(ev_buffer);
        last_timestamp = ((uint64_t) ev.timestamp)<<11;
        } else if(static_cast<Event_Types_underlying>(Event_Types::LEFT_TD_LOW) == evbase->type) {
            Event_EVENT2D ev = *reinterpret_cast<Event_EVENT2D*>(ev_buffer);
        ts = ((last_timestamp + (uint32_t) ev.timestamp)%(1<<24));
        uint64_t encoded = ((uint64_t) ts)|((uint64_t)(this->height-1-ev.y)<<42)|((uint64_t)(this->width-1-ev.x)<<33)|0UL<<32;
        *target64++ = encoded;
            ++n_TD;
        } else if(static_cast<Event_Types_underlying>(Event_Types::LEFT_TD_HIGH) == evbase->type) {
            Event_EVENT2D ev = *reinterpret_cast<Event_EVENT2D*>(ev_buffer);
        ts = ((last_timestamp + (uint32_t) ev.timestamp)%(1<<24));
        uint64_t encoded = ((uint64_t) ts)|((uint64_t)(this->height-1-ev.y)<<42)|((uint64_t)(this->width-1-ev.x)<<33)|1UL<<32;
        *target64++ = encoded;
            ++n_TD;
        }
    ev_buffer++;
    n_events--;
    }


    //std::cout << "sending " << n_TD << " events" << std::endl;
    return n_TD*8;
}


bool vDevReadBuffer::initialise(Chronocam::I_EventsStream &stream,
                                unsigned int bufferSize,
                                unsigned int readSize)
{


    this->stream = &stream;
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


        signal.check();

        while(!isStopping()) {

        safety.wait();

        int r = 0;
        if(readCount >= bufferSize) {
            //we have reached maximum software buffer - read from the HW but
            //just discard the result.
            //TODO: read events!
            //r = read(fd, discardbuffer.data(), readSize);
            r = getEventChunk(discardbuffer.data());
            //std::cout << "discarded " << r << " events" << std::endl;
            if(r > 0) lossCount += r;
        } else {
            //we read and fill up the buffer
            //TODO: read events!
            //r = read(fd, readBuffer->data() + readCount, std::min(bufferSize - readCount, readSize));
            r = getEventChunk(readBuffer->data() + readCount);
            //if (r>0) std::cout << "read " << r << " events" << std::endl;
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
        //close
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


bool device2yarp::initialise(Chronocam::I_EventsStream &stream,
                             std::string moduleName, bool check,
                             unsigned int bufferSize,
                             unsigned int readSize, unsigned int chunkSize)
{

    this->chunksize = chunkSize;
    if(!deviceReader.initialise(stream, bufferSize, readSize))
        return false;

    this->errorchecking = check;
    yInfo() << "yarp::os::Port used - which is always strict";


    if(!portvBottle.open(moduleName + "/eventCount:o"))
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

    ev::vGenPortInterface external_storage;
    external_storage.setHeader(ev::AE::tag);

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

    //std::cout << *(int*)data.data() << std::endl;
        //typical ZYNQ behaviour to skip error checking
        unsigned int i = 0;
        i = 0;
        if(!errorchecking && !dataError) {

            while((i+1) * chunksize < nBytesRead) {

                //ev::vBottleMimic &vbm = portvBottle.prepare();
                external_storage.setExternalData((const char *)data.data() + i*chunksize, chunksize);
                vStamp.update();
                portvBottle.setEnvelope(vStamp);
                portvBottle.write(external_storage);
                //portvBottle.write(strict);
                //portvBottle.waitForWrite();

                i++;
            }

            //ev::vBottleMimic &vbm = portvBottle.prepare();
            external_storage.setExternalData((const char *)data.data() + i*chunksize, nBytesRead - i*chunksize);
            vStamp.update();
            portvBottle.setEnvelope(vStamp);
            portvBottle.write(external_storage);
            //portvBottle.write(strict);
            //portvBottle.waitForWrite();

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

                    //ev::vBottleMimic &vbm = portvBottle.prepare();
                    external_storage.setExternalData((const char *)data.data()+bstart, bend-bstart);
                    countAEs += (bend - bstart) / 8;
                    vStamp.update();
                    portvBottle.setEnvelope(vStamp);
                    portvBottle.write(external_storage);
                    //if(strict) portvBottle.writeStrict();
                    //else portvBottle.write();
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
            //ev::vBottleMimic &vbm = portvBottle.prepare();
            external_storage.setExternalData((const char *)data.data()+bstart, 8*((nBytesRead-bstart)/8));
            countAEs += (nBytesRead - bstart) / 8;
            vStamp.update();
            portvBottle.setEnvelope(vStamp);
            portvBottle.write(external_storage);
            //if(strict) portvBottle.writeStrict();
            //else portvBottle.write();
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



