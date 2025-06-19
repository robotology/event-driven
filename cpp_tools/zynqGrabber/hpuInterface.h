/*
 *   Copyright (C) 2023 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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

#ifndef __ZYNQ_HPU_INTERFACE__
#define __ZYNQ_HPU_INTERFACE__

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "hpuDevice.h"

using yarp::os::Value;
using std::string;


/******************************************************************************/
//hpuInterface
/******************************************************************************/
class hpuInterface {

private:

    int fd{-1};
    ev::BufferedPort<ev::AE> y2d_port;
    ev::BufferedPort<ev::AE> d2y_port;
    ev::BufferedPort<ev::AE> d2y_port_2;
    ev::BufferedPort<ev::AE> d2y_port_skin;
    std::thread y2d_thread;
    std::thread d2y_thread;
    int y2d_eventcount{0};
    int d2y_eventcount{0};
    int d2y_packetcount{0};
    int d2y_filtered{0};

    //this thread runs constantly to write YARP data to the device (e.g. spinnaker)
    void y2d_run() 
    {        
        while(params.hpu_write)
        {
            if(y2d_port.isClosed()) {
                yarp::os::Time::delay(1.0);
                continue;
            }

            const ev::packet<ev::AE>* packet = y2d_port.read(true); //blocking read
            if(!packet) return; //when interrupt is called returns null

            int written = packet->pushToDevice(fd);
            y2d_eventcount += written / sizeof(ev::AE);
        }
    }


    void d2y_run_split()
    {
        int max_events_per_read = params.max_packet_size / sizeof(ev::AE) + (params.max_packet_size % sizeof(ev::AE) ? 1 : 0);
        int max_bytes_per_read = max_events_per_read * sizeof(ev::AE);
        std::vector<ev::AE> buffer(max_events_per_read);

        ev::packet<ev::AE>* packet_left = &d2y_port.prepare();
        packet_left->size(max_events_per_read);
        double tic_left = yarp::os::Time::now();
    
        ev::packet<ev::AE>* packet_right = &d2y_port_2.prepare();
        packet_right->size(max_events_per_read);
        double tic_right = yarp::os::Time::now();

        ev::packet<ev::AE>* packet_skin = &d2y_port_skin.prepare();
        double tic_skin = yarp::os::Time::now();

        ev::refractoryFilter refrac;
        if(params.filter > 0.0)
            refrac.initialise(params.roi_max_y, params.roi_max_x, params.filter);

        while(params.hpu_read) {

            //read data
            int r = -1;
            while (r < 0)
            {
                r = ::read(fd, (char *)buffer.data(), max_bytes_per_read);
                if (r < 0)
                    yInfo() << "[READ ]" << std::strerror(errno);
            }
            if (r % sizeof(ev::AE))
                yError() << "[READ ] partial read. bad fault. get help.";
            if(r == max_bytes_per_read) continue;

            //stats
            int events_read = r / sizeof(ev::AE);
            d2y_eventcount += events_read;
            d2y_packetcount++;

            double toc = yarp::os::Time::now();

            //sort the events
            if(params.filter) 
            {
                for(size_t i = 0; i < events_read; i++) {
                    ev::AE &event = buffer[i];
                    if(event.skin) {
                        //SKIN
                        packet_skin->push_back(event);
                    } else if(event.x >= params.roi_max_x || event.y >= params.roi_max_y) {
                        //yWarning() << "[" << event.x << "," << event.y << "]";
                    } else {
                        //VISION
                        if(!refrac.check(event, toc)) {
                            d2y_filtered++;
                            continue;
                        }
                        event.y = params.roi_max_y - 1 - event.y;
                        //event.x = params.roi_max_x - 1 - event.x;
                        if(event.channel == ev::CAMERA_LEFT)
                            packet_left->push_back(event);
                        else
                            packet_right->push_back(event);
                    }
                }
            } else {
                for(size_t i = 0; i < events_read; i++) {
                    ev::AE &event = buffer[i];
                    if(event.skin) {
                        //SKIN
                        packet_skin->push_back(event);
                    } else if(event.x >= params.roi_max_x || event.y >= params.roi_max_y) {
                        //yWarning() << "[" << event.x << "," << event.y << "]";
                    } else {
                        //VISION
                        event.y = params.roi_max_y - 1 - event.y;
                        //event.x = params.roi_max_x - 1 - event.x;
                        if(event.channel == ev::CAMERA_LEFT)
                            packet_left->push_back(event);
                        else
                            packet_right->push_back(event);
                    }
                }
            }

            if(d2y_port.isWriting() || d2y_port_2.isWriting() || d2y_port_skin.isWriting())
                continue;

            if(packet_left->size()) 
            {
                packet_left->duration(toc - tic_left);
                tic_left = toc;
                static int sequence_left = 0;
                packet_left->envelope() = {sequence_left++, toc};
                //if(packet_left->size() / packet_left->duration() < params.rate_limit) {
                    d2y_port.write();
                    packet_left = &d2y_port.prepare();
                //} else {
                //    packet_left->clear();
                //    yWarning() << "Dropped packet left";
                //} 
            }

            if(packet_right->size())
            {
                packet_right->duration(toc - tic_right);
                tic_right = toc;
                static int sequence_right = 0;
                packet_right->envelope() = {sequence_right++, toc};
                //if(packet_right->size() / packet_right->duration() < params.rate_limit) {
                    d2y_port_2.write();
                    packet_right = &d2y_port_2.prepare();
                //} else {
                //    packet_right->clear();
                //    yWarning() << "Dropped packet right";
                //}
            }

            if(packet_skin->size())
            {
                packet_skin->duration(toc - tic_skin);
                tic_skin = toc;
                static int sequence_skin = 0;
                packet_skin->envelope() = {sequence_skin++, toc};
                d2y_port_skin.write();
                packet_skin = &d2y_port_skin.prepare();
            }   
        }

        d2y_port.unprepare();
        d2y_port_2.unprepare();
        d2y_port_skin.unprepare();
    }

    //this thread runs constantly to read device (e.g. camera) data and send to YARP 
    void d2y_run() 
    {
        while(params.hpu_read) {

            //allocate the space in the packet within the output port
            ev::packet<ev::AE>& packet = d2y_port.prepare();
            packet.clear();
            packet.size(params.max_packet_size / sizeof(ev::encoded) + (params.max_packet_size % sizeof(ev::AE) ? 1 : 0));

            //keep filling the packet while the previous packet is still sending
            static double tic = yarp::os::Time::now();
            bool iswriting = true;
            while(iswriting) 
            {
                int r = packet.singleDeviceRead(fd);
                iswriting = d2y_port.isWriting();
                if(r == 0) break; //the packet is full, we cannot read anymore data!
            }

            //the temporal duration is measured
            double toc = yarp::os::Time::now();
            packet.duration(toc - tic);
            tic = toc;
            
            //an error check that shouldn't occur
            if(packet.size() == 0) {
                yError() << "0 size packet?";
                d2y_port.unprepare();
                continue;
            }

            //update stats for keeping tracking of event counts
            d2y_eventcount += packet.size();
            d2y_packetcount++;

            //send the packet of data (the port does in a second thread)
            static int sequence = 0;
            packet.envelope() = {sequence++, toc};
            d2y_port.write();
        }
    }

    void start()
    {
        if(params.hpu_write)
            y2d_thread = std::thread([this]{y2d_run();});
        if(params.hpu_read) {
            if(params.split)
                d2y_thread = std::thread([this]{d2y_run_split();});
            else
                d2y_thread = std::thread([this]{d2y_run();});
        }
    }

public:

    struct {
        string module{""};
        string device{""};
        bool hpu_read{false};
        bool hpu_write{false};
        bool gtp{true};
        bool spinnaker{false};
        bool spin_loopback{false};
        unsigned int max_packet_size{8*7500};
        bool split{false};
        double filter{0.0};
        int roi_max_x{640};
        int roi_max_y{480};
        double rate_limit{40e6};

    } params;

    bool configure()
    {
        yInfo() << "===== HPU Interface =====";
        yInfo() << params.module;
        yInfo() << params.device;
        if(params.hpu_read) yInfo() << "reading from device (d2y)";
        if(params.hpu_write) yInfo() << "writing to device (y2d)";
        if(params.gtp) yInfo() << "using gtp";
        if(params.spinnaker) yInfo() << "Using Spinnaker";
        if(params.spinnaker && params.spin_loopback) yWarning() << "Spinnaker in loopback mode";
        yInfo() << "Maximum " << params.max_packet_size / 8 << "AE in a packet";
        if(params.split) yInfo() << "Splitting stereo and skin (d2y)";
        if(params.filter > 0.0) yInfo() << "Artificial refractory period:" << params.filter << "seconds";

        // open the device
        fd = open(params.device.c_str(), O_RDWR);
        if (fd < 0) 
        {   fd = open(params.device.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0)
            { yError() << "Could not open" << params.device << " device"; return false; }
            else
            { yWarning() << params.device << "only opened in read-only, non-blocking mode"; }
        }

        //ts_flag is set through compiler options. i.e. set in 
        //cmake options
#if ENABLE_TS
        unsigned int ts_flag = 1;
        yInfo() << "ON: individual event timestamps";
#else
        unsigned int ts_flag = 0;
        yInfo() << "OFF: individual event timestamps";
#endif

        //try to configure DEVICE registers
        std::string msg;
        if(!configureHPURegisters(fd, params.gtp, ts_flag, msg))
            { yError() << msg; return false; }
        
        //configure spinnaker if needed
        if(params.spinnaker)
            configureSpinnakerMode(fd, params.spin_loopback);
        
        //output device information
        yInfo() << HPUDeviceInfo(fd);

        //start reading/writing threads.
        start();
        return true;
    }

    bool connectYARP()
    {

        if (!yarp::os::Network::checkNetwork(1.0)) {
            yInfo() << "HPU interface: YARP network not available";
            return true;
        }

        if(params.hpu_read && d2y_port.isClosed()) {
            std::string port_name = params.module + "/AE:o";
            if(params.split) port_name = params.module + "/left/AE:o";
            if(!d2y_port.open(port_name)) {
                yError() << "Could not open" << port_name;
                return false;
            }
        }

        if(params.hpu_read && params.split && d2y_port_2.isClosed()) {
            std::string port_name = params.module + "/right/AE:o";
            if(!d2y_port_2.open(port_name)) {
                yError() << "Could not open" << port_name;
                return false;
            }
        }

        if(params.hpu_read && params.split && d2y_port_skin.isClosed()) {
            std::string port_name = params.module + "/skin/AE:o";
            if(!d2y_port_skin.open(port_name)) {
                yError() << "Could not open" << port_name;
                return false;
            }
        }

        if(params.hpu_write && y2d_port.isClosed()) {
            std::string port_name = params.module + "/AE:i";
            if(!y2d_port.open(port_name)) {
                yError() << "Could not open" << port_name;
                return false;
            }
        }

        return true;
    }

    void stop()
    {
        if(params.hpu_read) 
            { params.hpu_read = false; d2y_port.close(); d2y_port_2.close(); d2y_port_skin.close(); d2y_thread.join(); }
        if(params.hpu_write)
            { params.hpu_write = false; y2d_port.close(); y2d_thread.join(); }
    }

    std::string status_message()
    {
        std::stringstream ss; ss.str("zynqGrabber running happily...\n");
        static double previous_time = 0;
        double dt = yarp::os::Time::now() - previous_time;

        if(params.hpu_write) {
            hpu_regs_t hpu_regs = {0x18, 0, 0};
            if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
                ss << "Y2D: Couldn't read dump status" << std::endl;
            }

            if(hpu_regs.data & 0x00100000) {
                ss << "[DUMP ] " << (int)(0.001 * y2d_eventcount / dt) << " k events/s ("
                    << y2d_port.getPendingReads() << " delayed packets)" << std::endl;;
            } else {
                ss << "[WRITE] " << (int)(0.001 * y2d_eventcount / dt) << " k events/s ("
                    << y2d_port.getPendingReads() << " delayed packets)" << std::endl;
            }

            y2d_eventcount = 0;
        }
        
        if(params.hpu_read) {
            ss << "[READ ] "
                    << (int)(d2y_eventcount/(1000.0*dt)) << "k events/s over "
                    << d2y_packetcount << " packets, in " 
                    << dt << " seconds ("
                    << (int)(100.0 * d2y_filtered / (double)d2y_eventcount) << "% filtered)" << std::endl;

            d2y_eventcount = 0;
            d2y_packetcount = 0;
            d2y_filtered = 0;
        }

        previous_time += dt;

        return ss.str();
    }

};

#endif
