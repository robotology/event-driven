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

/**
 * @file zynqGrabberModule.cpp
 * @brief Implementation of the zynqGrabberModule (see header file).
 */

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include "hpuInterface.h"
#include "vsctrlInterface.h"

#include <iostream>
#include <ctime>
#include <fstream>
#include <string>

using namespace ev;
using yarp::os::Value;
using std::string;

class zynqGrabberModule : public yarp::os::RFModule {
    // HANDLES DEVICE CONFIGURATION
    //autoVisionController visCtrlManager;
    //vSkinCtrl skctrlMng;
    hpuInterface hpu;

   public:
    bool configure(yarp::os::ResourceFinder &rf) {

        setName(rf.check("name", yarp::os::Value("/zynqGrabber")).asString().c_str());


        readCameraTypes("/dev/i2c-0");
        // if (rf.check("i2cVision")) {
        //     std::cout << std::endl;
        //     std::cout.flush();
        //     yInfo() << "===== Vision Controller =====";
        //     visCtrlManager.connect(rf.find("i2cVision").asString());
        //     visCtrlManager.configureAndActivate(rf);
        //     std::cout << std::endl;
        //     std::cout.flush();
        // }

        // if (rf.check("skinCtrl")) {
        //     std::cout << std::endl;
        //     std::cout.flush();
        //     yInfo() << "===== Skin Controller =====";

        //     skctrlMng = vSkinCtrl(rf.find("skinCtrl").asString(), I2C_ADDRESS_AUX);

        //     if (!skctrlMng.connect()) {
        //         yError() << "Could not connect to skin controller";
        //         return false;
        //     }

        //     if (!skctrlMng.configure()) {
        //         yError() << "Could not set skin defaults";
        //         return false;
        //     }

        //     // config values
        //     yarp::os::Bottle &cnfglists = rf.findGroup("SKIN_CNFG");
        //     if (cnfglists.isNull()) {
        //         yWarning() << "No Skin Parameters Found";
        //     } else if (!skctrlMng.configureRegisters(cnfglists)) {
        //         yError() << "Could not configure ini parameters";
        //         return false;
        //     }
        //     std::cout << std::endl;
        //     std::cout.flush();
        // }

        if (rf.check("dataDevice")) {

            hpu.params.module = getName();
            hpu.params.device = rf.find("dataDevice").asString();
            hpu.params.spinnaker = rf.check("use_spinnaker") &&
                                   rf.check("use_spinnaker", yarp::os::Value(true)).asBool();
            hpu.params.spin_loopback = rf.check("loopback_debug") &&
                                       rf.check("loopback_debug", yarp::os::Value(true)).asBool();
            hpu.params.gtp = rf.check("gtp") &&
                             rf.check("gtp", Value(true)).asBool();
            hpu.params.hpu_read = rf.check("hpu_read") &&
                                  rf.check("hpu_read", yarp::os::Value(true)).asBool();
            hpu.params.hpu_write = rf.check("hpu_write") &&
                                   rf.check("hpu_write", yarp::os::Value(true)).asBool();
            hpu.params.max_packet_size = 8 * rf.check("packet_size", yarp::os::Value("5120")).asInt32();

            if(!hpu.configure())
                return false;
            if(!hpu.connectYARP())
                return false;
        }

        return true;
    }

    bool interruptModule() override {
        hpu.stop();
        //visCtrlManager.disconnect();
        return true;
    }

    double getPeriod() override {
        return 1.0;
    }

    bool updateModule() override {
        hpu.connectYARP();
        yInfo() << hpu.status_message();
        return true;
    }
};

int main(int argc, char *argv[]) {
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("zynqGrabber.ini");  // overridden by --from parameter
    rf.setDefaultContext("event-driven");        // overridden by --context parameter
    rf.configure(argc, argv);

    zynqGrabberModule module;
    return module.runModule(rf);
}