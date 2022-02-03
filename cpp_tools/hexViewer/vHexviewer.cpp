/*
 *   Copyright (C) 2022 Event-driven Perception for Robotics
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

#include <yarp/os/all.h>
#include "event-driven/core.h"
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace ev;
using namespace yarp::os;
using std::vector;

class hexViewer : public RFModule, public Thread {

private:

    ev::BufferedPort<ev::encoded> input_port;
    unsigned int mask;
    unsigned int bits_to_check;
    int cols;

public:

    hexViewer() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {

        //display help
        if(rf.check("help") || rf.check("h")) {
            yInfo() << "vHexviewer is designed to view the event-stream as"
                       " hexadecimal values with the option to mask certain"
                       " events that correspond to a criteria";
            yInfo() << "--cols: set the number of output columns to use."
                       " Fit to your terminal width";
            yInfo() << "--mask: only events corresponding to the bit pattern"
                       " are shown.";
            yInfo() << "x = don't care | 1 = bit must be set | 0 bit must be clear";
            yInfo() << "example --mask 10x011xx";
            return false;
        }

        /* initialize yarp network */
        yarp::os::Network yarp;
        if(!yarp.checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        //set the module name used to name ports
        setName((rf.check("name", Value("/vHexviewer")).asString()).c_str());

        //open io ports
        if(!input_port.open(getName("/AE:i"))) {
            yError() << "Could not open input port";
            return false;
        }

        if(yarp::os::Network::connect("/zynqGrabber/AE:o", getName("/AE:i"), "fast_tcp")) {
            yWarning() << "Automatically connected to /zynqGrabber/AE:o but"
                          " maybe that's not what you want!";
        }

        cols = rf.check("cols", Value(4)).asInt();
        std::stringstream ss; ss.str("");

        bits_to_check = 0; mask = 0;
        if(rf.check("mask")) {
            std::string maskstring = rf.check("mask", Value("x")).asString();
            if(maskstring.empty()) {
                ss.str(""); ss << rf.find("mask").asInt();
                maskstring = ss.str();
            }

            //navigate through
            int bit = 0;
            for(std::string::const_reverse_iterator c = maskstring.rbegin();
                c != maskstring.rend(); c++, bit++) {

                if(*c == '1') {
                    mask |= (1 << bit);
                    bits_to_check |= (1 << bit);
                } else if(*c == '0') {
                    bits_to_check |= (1 << bit);
                }

            }

            ss.str("");
            if(bits_to_check) {
                for(int i = 31; i >= 0; i--) {
                    if(bits_to_check & (1 << i)) {
                        if(mask & (1 << i))
                            ss << "1";
                        else
                            ss << "0";
                    } else {
                        ss << "x";
                    }
                }
                ss << "b";
                yInfo() << "Showing only events with bits: " << ss.str();
            } else {
                yInfo() << "No valid mask provided - showing all events";
            }

        } else {
            yInfo() << "No mask provided - showing all events";
        }

        //start the asynchronous and synchronous threads
        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 1.0; //period of synchrnous thread
    }

    bool interruptModule()
    {
        //if the module is asked to stop ask the asynchrnous thread to stop
        return Thread::stop();
    }

    void onStop()
    {
        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        input_port.close();
    }

    //synchronous thread
    virtual bool updateModule()
    {
        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        std::cout << std::hex << std::setfill('0') << std::internal << std::uppercase;
        int coli = 0;

        while(!Thread::isStopping()) {

            int qs = input_port.getPendingReads();
            if(qs < 1) qs = 1;

            for(int i = 0; i < qs; i++)
            {
                ev::packet<ev::encoded> * q = input_port.read();
                if(!q) return;
                for(auto &v : *q)
                {
                    if((v.data & bits_to_check) == mask) {
                        if(coli++ % cols == 0) std::cout << std::endl;
                        std::cout << "0x" << std::setw(8) << v.data << " ";
                    }
                }
            }

            if(coli) {
                coli = 0;
                std::cout << std::endl << "==";
                std::cout.flush();
            }
        }
    }
};

int main(int argc, char * argv[])
{


    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "vHexviewer.ini" );
    rf.configure( argc, argv );

    /* create the module */
    hexViewer instance;
    return instance.runModule(rf);
}
