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
            yInfo() << "--<iso, grey, black, eros, corner, flow, scarf> : drawer style";
            yInfo() << "--src[1-9] : connect to up to 10 remotes";
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

        int height = rf.check("height", Value(720)).asInt32();
        int width = rf.check("width", Value(1280)).asInt32();

        double window_size = rf.check("window_size", Value(-1.0)).asFloat64();

        int frameRate = rf.check("fps", Value(60)).asInt32();
        double period = 1.0 / frameRate;

        bool yarp_publish = rf.check("yarp_publish") && rf.check("yarp_publish", Value(true)).asBool();

        int kernel_size = rf.check("eros_kernel", Value(5)).asInt32();
        double decay = rf.check("eros_decay", Value(0.3)).asFloat64();

        bool flip =
            rf.check("flip") && rf.check("flip", Value(true)).asBool();

        std::string style = "iso";
        if(rf.check("eros")) style = "eros";
        if(rf.check("grey")||rf.check("gray")) style = "grey";
        if(rf.check("black")) style = "black";
        if(rf.check("corner")) style = "corner";
        if(rf.check("scarf")) style = "scarf";
        if(rf.check("flow")) style = "flow";

        std::stringstream remote_id;
        for(int i = 0; i < 10; i++) 
        {
            remote_id.str("src");
            if(i > 0) remote_id << "src" << i;
            if(!rf.check(remote_id.str())) continue; //srcN not supplied

            std::string remote = rf.find(remote_id.str()).asString();
            
            //add a drawer with the source
            if(style=="iso") publishers.push_back(new isoDrawer);
            if(style=="grey" || style=="gray") publishers.push_back(new greyDrawer);
            if(style=="black") publishers.push_back(new blackDrawer);
            if(style=="eros") publishers.push_back(new erosDrawer(kernel_size, decay));
            if(style=="corner") publishers.push_back(new cornerDrawer);
            if(style=="scarf") publishers.push_back(new scarfDrawer);
            if(style=="flow") publishers.push_back(new flowDrawer);
            
            if(publishers.back()->initialise(remote, height, width, window_size, yarp_publish, remote))
            {
                yInfo() << "Drawing" << style << "from" << remote;
            } else {
                yError() << "[" << style << "DRAW ] failure";
            }
            publishers.back()->setPeriod(period);
        }

        if(publishers.empty()) {
            yError() << "No sources provided --src, --src[1-9]";
            return false;
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
    yarp::os::Network::init();
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    vFramerModule framerModule;
    return framerModule.runModule(rf);
}
