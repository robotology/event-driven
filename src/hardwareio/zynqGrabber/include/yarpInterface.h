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


#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <string>
#include <vector>

using namespace yarp::os;
using namespace ev;
using std::string;
using std::vector;

/******************************************************************************/
//vDevReadBuffer
/******************************************************************************/
class vDevReadBuffer : public yarp::os::Thread {

private:

    //parameters
    unsigned int read_size;
    unsigned int buffer_size;


    //internal variables/storage
    int fd;
    unsigned int read_count;
    unsigned int loss_count;
    std::vector<unsigned char> buffer1;
    std::vector<unsigned char> buffer2;
    std::vector<unsigned char> discard_buffer;

    std::vector<unsigned char> *read_buffer;
    std::vector<unsigned char> *access_buffer;

    yarp::os::Semaphore safety;
    yarp::os::Semaphore signal;
    bool bufferedreadwaiting;

public:

    vDevReadBuffer(int fd, unsigned int read_size, unsigned int buffer_size);
    std::vector<unsigned char>& getBuffer(unsigned int &nBytesRead,
                                          unsigned int &nBytesLost);
    virtual void run();

};

/******************************************************************************/
//device2yarp
/******************************************************************************/
class device2yarp : public yarp::os::Thread {

private:

    //data buffer thread
    vDevReadBuffer *device_reader;
    yarp::os::Port output_port;

    //parameters
    unsigned int packet_size;
    bool direct_read;
    Stamp yarp_stamp;

    int countAEs;
    int countLoss;
    double rate;
    int prevAEs;
    double prevTS;

public:

    device2yarp();
    bool open(string module_name, int fd, unsigned int read_size,
              bool direct_read, unsigned int packet_size,
              unsigned int internal_storage_size);
    void setDirectRead(bool value = true);

    void run();
    void onStop();
    void threadRelease();
    void afterStart(bool success);

};

/******************************************************************************/
//yarp2device
/******************************************************************************/
class yarp2device : public Thread
{
protected:

    int fd;
    bool valid;
    BufferedPort<Bottle> input_port;
    vector<int> data_copy;

    unsigned int total_events;

public:

    yarp2device();
    bool open(string module_name, int fd);
    void run();
    void onStop();


};

/******************************************************************************/
//hpuInterface
/******************************************************************************/
class hpuInterface {

private:

    int fd;
    device2yarp D2Y;
    yarp2device Y2D;

    int pool_size;
    bool read_thread_open;
    bool write_thread_open;

public:

    hpuInterface();

    bool configureDevice(string device_name, bool spinnaker = false,
                         bool loopback = false);
    bool openReadPort(string module_name, bool direct_read,
                      unsigned int packet_size,
                      unsigned int maximum_internal_memory);
    bool openWritePort(string module_name);
    void start();
    void stop();

};

#endif
