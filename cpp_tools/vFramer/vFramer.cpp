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

#include "drawers.h"
#include <sstream>
#include <yarp/cv/Cv.h>

using namespace ev;
using namespace yarp::os;
using namespace yarp::sig;
using std::string;

/*////////////////////////////////////////////////////////////////////////////*/
//module
/*////////////////////////////////////////////////////////////////////////////*/
class vFramerModule : public yarp::os::RFModule
{

private:
    std::vector<drawerInterface *> publishers;

public:

    // configure all the module parameters and return true if successful
    bool configure(yarp::os::ResourceFinder &rf) override
    {
        if(rf.check("h") || rf.check("help"))
        {
            yInfo() << "vFramer - visualisation of event data";
            yInfo() << "======================";
            yInfo() << "--iso   [remote] : iso view,  optionally connect to remote (string)";
            yInfo() << "--eros  [remote] : eros view, optionally connect to remote (string)";
            yInfo() << "--grey  [remote] : grey view, optionally connect to remote (string)";
            yInfo() << "--black [remote] : black view (calibration), optionally connect to remote (string)";
            yInfo() << "======================";
            yInfo() << "--name : global module name for ports";
            yInfo() << "--height, --width : set resolution";
            yInfo() << "--window_size : parameter affecting event accumulation (seconds/count)";
            yInfo() << "--fps : frame-rate cap of display";
            yInfo() << "--yarp_publish : publish over yarp port (calibration) instead of opencv frame";
            yInfo() << "--flip : flip the image x and y";
            yInfo() << "======================";
            yInfo() << "--eros_kernel : kernel size for eros view";
            yInfo() << "--eros_decay  : decay rate for eros view";
            return false;
        }

        if (!yarp::os::Network::checkNetwork(2.0)) {
            yError() << "Could not find yarp network";
            return false;
        }

        //admin options
        std::string moduleName = rf.check("name", Value("/vFramer")).asString();
        setName(moduleName.c_str());

        int height = rf.check("height", Value(480)).asInt32();
        int width = rf.check("width", Value(640)).asInt32();

        double window_size = rf.check("window_size", Value(-1.0)).asFloat64();

        int frameRate = rf.check("fps", Value(60)).asInt32();
        double period = 1.0 / frameRate;

        bool yarp_publish = rf.check("yarp_publish") && rf.check("yarp_publish", Value(true)).asBool();

        int kernel_size = rf.check("eros_kernel", Value(5)).asInt32();
        double decay = rf.check("eros_decay", Value(0.3)).asFloat64();

        bool flip =
            rf.check("flip") && rf.check("flip", Value(true)).asBool();

        if (rf.check("grey") || rf.check("gray")) {
            string remote = rf.find("grey").asString();
            if(!remote.size()) remote = rf.find("gray").asString();
            publishers.push_back(new greyDrawer);
            if (!publishers.back()->initialise(getName("/grey"), height, width, window_size, yarp_publish, remote)) {
                yError() << "[GREY DRAW] failure";
                return false;
            } else {
                yInfo() << "[GREY DRAW] success";
            }
            publishers.back()->setPeriod(period);
        }

        if (rf.check("black")) {
            string remote = rf.find("black").asString();
            publishers.push_back(new blackDrawer);
            if (!publishers.back()->initialise(getName("/black"), height, width, window_size, yarp_publish, remote)) {
                yError() << "[BLACK DRAW] failure";
                return false;
            } else {
                yInfo() << "[BLACK DRAW] success";
            }
            publishers.back()->setPeriod(period);
        }

        if (rf.check("eros")) {
            string remote = rf.find("eros").asString();
            publishers.push_back(new erosDrawer(kernel_size, decay));
            if (!publishers.back()->initialise(getName("/eros"), height, width, window_size, yarp_publish, remote)) {
                yError() << "[EROS DRAW] failure";
                return false;
            } else {
                yInfo() << "[EROS DRAW] success";
            }
            publishers.back()->setPeriod(period);
            
        }

        if (rf.check("corner")) {
            string remote = rf.find("corner").asString();
            publishers.push_back(new cornerDrawer());
            if (!publishers.back()->initialise(getName("/corner"), height, width, window_size, yarp_publish, remote)) {
                yError() << "[CORNER DRAW] failure";
                return false;
            } else {
                yInfo() << "[CORNER DRAW] success";
            }
            publishers.back()->setPeriod(period);
        }

        if (rf.check("iso") || !publishers.size()) {
            string remote = rf.find("iso").asString();
            publishers.push_back(new isoDrawer);
            if (!publishers.back()->initialise(getName("/iso"), height, width, window_size, yarp_publish, remote)) {
                yError() << "[ISO DRAW] failure";
                return false;
            } else {
                yInfo() << "[ISO DRAW] success";
            }
            publishers.back()->setPeriod(period);
        }

        yInfo() << "Starting publishers";
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
        {
            if (!(*pub_i)->start())
            {
                yError() << "Could not start publisher" << (*pub_i)->drawerName();
                return false;
            }
        }

        yInfo() << "Configure done";
        return true;
    }

    bool interruptModule() override
    {
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
            (*pub_i)->stop();

        return true;
    }

    bool close() override
    {
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
            (*pub_i)->stop();

        return true;
    }

    bool updateModule() override
    {
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++) {
            if(!(*pub_i)->isRunning()) return false;
            (*pub_i)->connectToRemote();
        }
        return !isStopping();
    }

    double getPeriod() override
    {
        return 2.0;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    vFramerModule framerModule;
    return framerModule.runModule(rf);
}
