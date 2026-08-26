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
    int fd{-1};

    //thread protection and synchronisation
    std::thread device_thread;
    std::thread port_thread;
    std::mutex m;
    std::condition_variable signal;
    bool port_free{false};
    bool buffer_switching{false};

    //data storage
    ev::packet<ev::AE> buffer1, buffer2;
    ev::packet<ev::AE> *fill_buffer, *send_buffer;
    static constexpr void switch_buffer(ev::packet<ev::AE>* &p1, ev::packet<ev::AE>* &p2) {auto t = p2; p2 = p1; p1 = t;};

    //other variables
    int counter_packets{0};
    int counter_events{0};
    int counter_dcmifull{0};
    int counter_usbdrop{0};
    int counter_corrupt{0};
    int counter_misaligned{0};
    static constexpr double period{1};

    typedef struct {
        unsigned int y:9;
        unsigned int x:9;
        unsigned int p:1;
        unsigned int t:12;
        unsigned int sync:1;
    } microXAE;
    static const int EVENT_CODE_DCMI_OVERRUN=510;
    static const int EVENT_CODE_DROPPED_PACKET=509;

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
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 10;

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
        
        startX320();
        //synchX320();

        device_thread = std::thread([this]{deviceToBuffer();});
        port_thread = std::thread([this]{bufferToPort();});

        //device_thread = std::thread([this]{maxRead();});

        
        
        return true;
    }

    void startX320()
    {
        if(fd > 0) {
            auto n = ::write(fd, "+\r", 2);
            if(n != 2) yWarning() << "start command failed: " << n << " bits written [2]";
        } else {
            yWarning() << "start command failed: invalid file descriptor";
        }
    }

    void stopX320()
    {
        if(fd > 0) {
            auto n = ::write(fd, "-\r", 2);
            if(n != 2) yWarning() << "stop command failed: " << n << " bits written [2]";
        } else {
            yWarning() << "stop command failed: invalid file descriptor";
        }
    }

    void synchX320()
    {
        char x = 0;
        do {
            auto n = ::read(fd, &x, 1);
            if(n != 1) break;
        } while (!(x & 0x80));
        auto n = ::read(fd, &x, 1);
        n = ::read(fd, &x, 1);
        n = ::read(fd, &x, 1);

    }

    void maxRead()
    {
        const static int rb_size = 1024*4*32; //max seems to be 1024 (we'll make it bigger and always multiple of 4)
        std::array<uchar, rb_size> rb;

        while(fd > 0) {

            auto n = ::read(fd, rb.data(), rb_size);
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                yError() << "Error reading from device";
                interruptModule();
                break;
            }
            counter_packets++;
            counter_events += n / 4;

        }
    }

    void deviceToBuffer()
    {
        const static int rb_size = 1024*4; //max seems to be 1024 (we'll make it bigger and always multiple of 4)
        bool should_notify{false};
        std::array<uchar, rb_size> rb;
        //int toc = -1;
        ev::AE ae;
        microXAE mae;
        while(fd > 0) {

            //pseduo read =
            double tic = yarp::os::Time::now();

            //fill_buffer->fillFromDevice(fd, 128, 1024*4);

            auto n = ::read(fd, rb.data(), rb_size);
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                yError() << "Error reading from device";
                interruptModule();
                break;
            }

            // for (int i = 0; i < n; i+=4) {
            //     ae.x = 320*(double)rand()/RAND_MAX; ae.y = 320*(double)rand()/RAND_MAX; ae.p = 1*(double)rand()/RAND_MAX;
            //     fill_buffer->push_back(ae);
            // }

            auto i = 0;
            while(i < n-3) {
                if (rb[i] & 0x80) { // sync bit detected
                    uint32_t word =
                        (static_cast<uint32_t>(rb[i+0]) << 24) |
                        (static_cast<uint32_t>(rb[i+1]) << 16) |
                        (static_cast<uint32_t>(rb[i+2]) << 8) |
                         static_cast<uint32_t>(rb[i+3]);
                    ae.p = (word >> 18) & 0x1u;
                    ae.x = (word >> 9) & 0x1FFu;
                    ae.y = (word >> 0) & 0x1FFu;
                    if(ae.y == EVENT_CODE_DCMI_OVERRUN) {
                        counter_dcmifull++;
                        i += 4;
                    } else if(ae.y == EVENT_CODE_DROPPED_PACKET) {
                        counter_usbdrop++;
                        i += 4;
                    } else if(ae.x < 320 && ae.y < 320) {
                        fill_buffer->push_back(ae);
                        i += 4;
                    } else {
                        counter_corrupt++;
                        i += 1;
                    }
                } else {
                    counter_misaligned++;
                    i += 1;
                }
            }


            double toc = yarp::os::Time::now();

            fill_buffer->duration(toc-tic + fill_buffer->duration());
            if(fill_buffer->size() == 0) continue;

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

         yInfo() << "Device thread finished";

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
        yInfo() << "Port thread finished";
    }

     double getPeriod() override
    {
        return period; //period of synchronous thread
    }

    bool interruptModule() override
    {
        stopX320();
        ::close(fd);
        fd = -1;
        yInfo() << getName() << "closed. Device released.";
        signal.notify_all();
        return true; 
    }

    bool close() override
    {   
        yInfo() << "Finally thread cleanup...";
        signal.notify_all();
        if(fd > 0) {
            stopX320();
            ::close(fd);
            fd = -1;
            yInfo() << getName() << "closed. Device released.";
            signal.notify_all();
        }
        
        device_thread.join();
        yInfo() << getName() << "device thread closed";
        port_thread.join();
        yInfo() << getName() << "port thread closed";
        output_port.close();
        return true;
    }

    //synchronous thread
    bool updateModule() override
    {
        yInfo() << (counter_events * 0.001 / getPeriod()) << "k events/s in"
                << counter_packets << "YARP packets."
                << counter_dcmifull << "DCMI overruns and"
                << counter_usbdrop << "USB drops."
                << counter_corrupt << "corrupt and"
                << counter_misaligned << "misalignments";
        counter_packets = counter_events = counter_dcmifull = counter_usbdrop = counter_corrupt = counter_misaligned = 0;

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
