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

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <string>
#include <vector>

using namespace yarp::os;
using namespace ev;
using std::string;
using std::vector;

/******************************************************************************/
//device2yarp
/******************************************************************************/
class device2yarp : public yarp::os::Thread {

private:

    //data buffer thread
    int fd;
    std::vector<unsigned char> data;
    yarp::os::Port output_port;
    Stamp yarp_stamp;

    //parameters
    unsigned int max_dma_pool_size;
    unsigned int max_packet_size;

public:

    device2yarp();
    bool open(string module_name, int fd, unsigned int pool_size,
              unsigned int packet_size);

    void run();
    void onStop();

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
    bool openReadPort(string module_name, unsigned int packet_size);
    bool openWritePort(string module_name);
    void start();
    void stop();

};

#endif
