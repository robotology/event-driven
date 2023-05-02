/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
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
    std::thread y2d_thread;
    std::thread d2y_thread;
    int y2d_eventcount{0};
    int d2y_eventcount{0};
    int d2y_packetcount{0};

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
        if(params.hpu_read)
            d2y_thread = std::thread([this]{d2y_run();});
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

    } params;

    bool configure()
    {
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
            if(!d2y_port.open(port_name)) {
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
            { params.hpu_read = false; d2y_port.close(); d2y_thread.join(); }
        if(params.hpu_write)
            { params.hpu_write = false; y2d_port.close(); y2d_thread.join(); }
    }

    std::string status_message()
    {
        std::stringstream ss; ss.str("zynqGrabber running happily...\n");
        static double previous_time = yarp::os::Time::now();
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
            ss << "[READ ]"
                    << (int)(d2y_eventcount/(1000.0*dt))
                    << "k events/s over"
                    << d2y_packetcount
                    << "packets" << std::endl;

            d2y_eventcount = 0;
            d2y_packetcount = 0;
        }

        previous_time += dt;

        return ss.str();
    }

};

#endif
