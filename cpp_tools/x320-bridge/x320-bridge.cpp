/*
 *   Copyright (C) 2026 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <yarp/os/all.h>
#include <vector>
#include <mutex>
#include <condition_variable>

#include "event-driven/core.h"
#include "event-driven/vis.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>

class x320bridge : public yarp::os::RFModule {

private:

    //ports and devices
    yarp::os::Port output_port;
    int fd;

    //thread protection and synchronisation
    std::thread device_thread;
    std::thread port_thread;
    std::mutex m;
    std::condition_variable signal;
    bool port_free{false};
    bool buffer_switching{false};

    //data storage
    ev::packet<ev::AE> buffer1;
    ev::packet<ev::AE> buffer2;
    ev::packet<ev::AE> *fill_buffer;
    ev::packet<ev::AE> *send_buffer;
    static constexpr void switch_buffer(ev::packet<ev::AE>* &p1, ev::packet<ev::AE>* &p2) {auto t = p2; p2 = p1; p1 = t;};

    //other variables
    int counter_packets{0};
    int counter_events{0};
    static constexpr double period{0.2};

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {

            yInfo() << "Bridge to push x320 events to YARP";
            yInfo() << "--name <str>\t: internal port name prefix [/x320]";
            yInfo() << "--device <str>\t: x320 device [/dev/ttyACM0]";
            return false;
        }

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        setName(rf.check("name", yarp::os::Value("/x320")).asString().c_str());

        //open the camera
        std::string path = rf.check("--device", yarp::os::Value("/dev/ttyACM0")).asString();
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            yError() << "Could not open device " << path;
            yError() << "Do you have permission? sudo chmod 666 /dev/ttyACM0";
            return false;
        }
    
        // Set blocking reads
        ::fcntl(fd, F_SETFL, 0);

        struct termios tio{};
        if (::tcgetattr(fd, &tio) < 0) {
            yError() << "Could not get " << path << " attributes. Is it corrupted?";
            return false;
        }

        ::cfmakeraw(&tio);
        ::cfsetispeed(&tio, B115200);
        ::cfsetospeed(&tio, B115200);

        // Read returns as soon as ≥1 byte arrives, no timeout
        tio.c_cc[VMIN]  = 1;
        tio.c_cc[VTIME] = 0;

        if (::tcsetattr(fd, TCSANOW, &tio) < 0) {
            yError() << "Could not set " << path << " attributes. Is it read-only?";
            return false;
        }

        fill_buffer = &buffer1;
        send_buffer = &buffer2;


        if(!output_port.open(getName() + "/AE:o")) {
            yError() << "Could not open output port" << getName() + "/AE:o";
            return false;
        }

        device_thread = std::thread([this]{deviceToBuffer();});
        port_thread = std::thread([this]{bufferToPort();});
        
        return true;
    }

void deviceToBuffer()
    {
        bool should_notify{false};
        double toc = 0;
        while(fd > 0) {

            //pseduo read =
            double tic = yarp::os::Time::now();
            ev::AE ae;
            for (int i = 0; i < 100; i++) {
                ae.x = 320*(double)rand()/RAND_MAX; ae.y = 320*(double)rand()/RAND_MAX; ae.p = 1*(double)rand()/RAND_MAX;
                fill_buffer->push_back(ae);
            }
            double toc = yarp::os::Time::now();

            fill_buffer->duration(toc-tic + fill_buffer->duration());

            //we should only be here if the fd read failed or returned early. maybe with a timeout it is possible. check
            if(fill_buffer->size() == 0) break;

            {
                std::unique_lock<std::mutex> lk(m);
                if(port_free && !buffer_switching) {
                    switch_buffer(fill_buffer, send_buffer);
                    fill_buffer->clear();
                    buffer_switching = true;
                    should_notify = true;
                    
                }
                if(!port_free && buffer_switching) {
                    buffer_switching = false;
                    should_notify = true; 
                }
            }

            if(should_notify) {
                signal.notify_one();
                should_notify = false;
            }
         }

    }

    void bufferToPort()
    {
        yarp::os::Stamp yarpstamp;
        while(fd > 0) {

            {
                std::unique_lock<std::mutex> lk(m);
                port_free = true;
                signal.wait(lk, [this] { return buffer_switching || fd < 0;});
                port_free = false;
            }

            if(fd < 0) break;

            // send the data in the first buffer 
            yarpstamp.update();
            output_port.setEnvelope(yarpstamp);
            output_port.write(*send_buffer);

            counter_packets++;
            counter_events += send_buffer->size();

            {
                std::unique_lock<std::mutex> lk(m);
                signal.wait(lk, [this] { return !buffer_switching || fd < 0;});
            }
        }
    }

     double getPeriod() override
    {
        return period; //period of synchronous thread
    }

    bool interruptModule() override
    {
        {
            std::unique_lock<std::mutex> lk(m);
            ::close(fd);
            fd = -1;
        }
        yInfo() << getName() << "closed. Device released.";
        return true; 
    }

    bool close() override
    {   
        if(fd > 0) {
            {
                std::unique_lock<std::mutex> lk(m);
                ::close(fd);
                fd = -1;
            }
            yInfo() << getName() << "closed. Device released.";
        }
        
        device_thread.join();
        yInfo() << getName() << "device thread closed";
        port_thread.join();
        yInfo() << getName() << "prot thread closed";
        output_port.close();
        return true;
    }

    //synchronous thread
    bool updateModule() override
    {
        yInfo() << counter_packets << "packets and"
                << (counter_events * 0.001) << "k events sent per second";
        counter_packets = counter_events = 0;

        return fd > 0;
    }   
    
};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    x320bridge instance;
    return instance.runModule(rf);
}
