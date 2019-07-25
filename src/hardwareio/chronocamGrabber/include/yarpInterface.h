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

#ifndef __EVENTDRIVENYARPINTERFACE__
#define __EVENTDRIVENYARPINTERFACE__

#define THRATE 1


#include "i_events_stream.h"

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <string>

/******************************************************************************/
//vDevReadBuffer
/******************************************************************************/
class vDevReadBuffer : public yarp::os::Thread {

private:

    //parameters
    unsigned int bufferSize;
    unsigned int readSize;
    long n_events;

    unsigned int last_timestamp=0;
    const int width = 304;
    const int height = 240;

    //internal variables/storage
    unsigned int readCount;
    unsigned int lossCount;
    unsigned int bytesperevent;
    bool msgflag;

    std::vector<unsigned char> *readBuffer;
    std::vector<unsigned char> *accessBuffer;
    std::vector<unsigned char> buffer1;
    std::vector<unsigned char> buffer2;
    std::vector<unsigned char> discardbuffer;

    uint32_t *ev_buffer;

    yarp::os::Semaphore safety;
    yarp::os::Semaphore signal;
    bool bufferedreadwaiting;

    Chronocam::I_EventsStream * stream = nullptr;

public:

    vDevReadBuffer();

    bool initialise(Chronocam::I_EventsStream &stream,
                    unsigned int bufferSize = 0,
                    unsigned int readSize = 0);

    virtual long getEventChunk(unsigned char *target);
    //virtual bool threadInit();      //run before thread starts
    virtual void run();             //main function
    //virtual void onStop();          //run when stop() is called (first)
    virtual void threadRelease();   //run after thread stops (second)
    std::vector<unsigned char>& getBuffer(unsigned int &nBytesRead, unsigned int &nBytesLost);

};

/******************************************************************************/
//device2yarp
/******************************************************************************/
class device2yarp : public yarp::os::Thread {

private:

    // we need an instance which we can read events from here.

    //parameters
    bool strict;
    bool errorchecking;
    bool applyfilter;
    bool jumpcheck;
    unsigned int chunksize;

    //internal variables
    yarp::os::Port portvBottle;
    yarp::os::BufferedPort<yarp::os::Bottle> portEventCount;
    int countAEs;
    int countLoss;
    double rate;
    int prevAEs;
    double prevTS;
    yarp::os::Stamp vStamp;
    ev::vNoiseFilter vfilter;

    //data buffer thread
    vDevReadBuffer deviceReader;

    int applysaltandpepperfilter(std::vector<unsigned char> &data, int nBytesRead);
    void tsjumpcheck(std::vector<unsigned char> &data, int nBytesRead);


public:

    device2yarp();
    bool initialise(Chronocam::I_EventsStream &stream,
                    std::string moduleName = "", bool check = false,
                    unsigned int bufferSize = 800000,
                    unsigned int readSize = 1024, unsigned int chunkSize = 40960);
    void initialiseFilter(bool applyfilter, int width, int height, int temporalsize, int spatialSize)
    {
        this->applyfilter = applyfilter;
        vfilter.initialise(width, height);
        vfilter.use_spatial_filter(temporalsize, spatialSize);
        vfilter.use_temporal_filter(temporalsize);
    }

    void checkForTSJumps()
    {
        jumpcheck = true;
    }

    virtual void run();
    virtual void threadRelease();
    virtual void afterStart(bool success);


};





#endif
