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

/**
 * @file zynqGrabberModule.cpp
 * @brief Implementation of the zynqGrabberModule (see header file).
 */

#include "zynqGrabberModule.h"
#include <fstream>
#include <ctime>
#include <string>

using namespace ev;
using namespace yarp::os;
using std::string;

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("zynqGrabber.ini"); //overridden by --from parameter
    rf.setDefaultContext("event-driven");   //overridden by --context parameter
    rf.configure(argc, argv);

    zynqGrabberModule module;
    return module.runModule(rf);

}


bool zynqGrabberModule::configure(yarp::os::ResourceFinder &rf) {

    setName(rf.check("name", yarp::os::Value("/zynqGrabber")).asString().c_str());
    
    if(rf.check("i2cVision"))
    { 
        std::cout << std::endl; std::cout.flush();
        yInfo() << "===== Vision Controller =====";
        visCtrlManager.connect(rf.find("i2cVision").asString());
        visCtrlManager.configureAndActivate(rf);
        std::cout << std::endl; std::cout.flush();
    }

    if(rf.check("skinCtrl")) {

        std::cout << std::endl; std::cout.flush();
        yInfo() << "===== Skin Controller =====";

        skctrlMng = vSkinCtrl(rf.find("skinCtrl").asString(), I2C_ADDRESS_AUX);

        if(!skctrlMng.connect())
        {
            yError() << "Could not connect to skin controller";
            return false;
        }

        if(!skctrlMng.configure()) {
            yError() << "Could not set skin defaults";
            return false;
        }

        //config values
        yarp::os::Bottle &cnfglists = rf.findGroup("SKIN_CNFG");
        if(cnfglists.isNull()) {
            yWarning() << "No Skin Parameters Found";
        } else if(!skctrlMng.configureRegisters(cnfglists)) {
            yError() << "Could not configure ini parameters";
            return false;
        }
        std::cout << std::endl; std::cout.flush();
    }

    //open rateThread device2yarp
    if(rf.check("dataDevice")) {

        std::cout << std::endl; std::cout.flush();
        yInfo() << "===== HPU Controller =====";

        string data_device = rf.find("dataDevice").asString();
        bool use_spinnaker = rf.check("use_spinnaker") &&
                             rf.check("use_spinnaker", yarp::os::Value(true)).asBool();
        bool loopback = rf.check("loopback_debug") &&
                        rf.check("loopback_debug", yarp::os::Value(true)).asBool();
        bool gtp = rf.check("gtp") &&
                   rf.check("gtp", Value(true)).asBool();
        bool record_mode = rf.check("record_mode") &&
                           rf.check("record_mode", Value(true)).asBool();

        if(!hpu.configureDevice(data_device, use_spinnaker, loopback, gtp))
            return false;

        bool read_flag = rf.check("hpu_read") &&
                         rf.check("hpu_read", yarp::os::Value(true)).asBool();
        bool write_flag = rf.check("hpu_write") &&
                          rf.check("hpu_write", yarp::os::Value(true)).asBool();
        int packet_size = 8 * rf.check("packet_size", yarp::os::Value("5120")).asInt32();

        if(read_flag)
            if(!hpu.openReadPort(getName(), packet_size, record_mode))
                return false;

        if(write_flag)
            if(!hpu.openWritePort(getName()))
                return false;

        yInfo() << "Starting HPU read/write threads";
        hpu.start();
        std::cout << std::endl; std::cout.flush();
    }

    // if (!handlerPort.open(moduleName)) {
    //     std::cout << "Unable to open RPC port @ /" << moduleName << std::endl;
    //     return false;
    // }
    // attach(handlerPort);

    return true;
}

bool zynqGrabberModule::interruptModule() {
    std::cout << "breaking YARP connections.. ";
   // handlerPort.close();        // rpc of the RF module
    hpu.stop();
    std::cout << "done" << std::endl;

    std::cout << "closing device drivers.. ";
    visCtrlManager.disconnect();
    std::cout << "done" << std::endl;
    return true;
}

bool zynqGrabberModule::close()
 {
    return true;
}

/* Called periodically every getPeriod() seconds */
bool zynqGrabberModule::updateModule() {

    //if(!vsctrlMngRight.activateAPSShutter())
    //    yWarning() << "Could not activate APS shutter";
    hpu.tryconnectToYARP();
    return !isStopping();
}

double zynqGrabberModule::getPeriod() 
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}



// RPC
 bool zynqGrabberModule::respond(const yarp::os::Bottle& command,
                                 yarp::os::Bottle& reply) {
                                     return false;
                                 }
//     bool ok = false;
//     bool rec = false; // is the command recognized?
//     std::string helpMessage =  std::string(getName().c_str()) +
//                                " commands are: \n" +
//                                "help \n" +
//                                "quit \n" +
//                                "set thr <n> ... set the threshold \n" +
//                                "(where <n> is an integer number) \n";

//     reply.clear();

//     if (command.get(0).asString()=="quit") {
//         reply.addString("quitting");
//         return false;
//     }
//     else if (command.get(0).asString()=="help") {
//         std::cout << helpMessage;
//         reply.addString("ok");
//     }

//     switch (command.get(0).asVocab()) {
//         case COMMAND_VOCAB_HELP:
//             rec = true;
//             {
//                 reply.addString("many");
//                 reply.addString("help");

//                 reply.addString("");

//                 ok = true;
//             }
//             break;
//             //    case COMMAND_VOCAB_SUSPEND:
//             //        rec = true;
//             //    {
//             //        //D2Y.suspend();
//             //        std::cout << "Not implemented" << std::endl;
//             //        ok = true;
//             //    }
//             //        break;
//             //    case COMMAND_VOCAB_RESUME:
//             //        rec = true;
//             //    {
//             //        //D2Y.resume();
//             //        std::cout << "Not implemented" << std::endl;
//             //        ok = true;
//             //    }
//             //        break;
//         case COMMAND_VOCAB_GETBIAS:
//             rec = true;
//             {
//                 std::string biasName = command.get(1).asString();
//                 std::string channel = command.get(2).asString();

//                 // setBias function
//                 if (channel == "left") {
//                     int val = vsctrlMngLeft.getBias(biasName);
//                     if(val >= 0) {
//                         reply.addString("Left: ");
//                         reply.addString(biasName);
//                         reply.addInt(val);
//                         ok = true;
//                     } else {
//                         reply.addString("Left Unknown Bias");
//                         ok = false;
//                     }
//                 } else if (channel == "right") {
//                     int val = vsctrlMngRight.getBias(biasName);
//                     if(val >= 0) {
//                         reply.addString("Right: ");
//                         reply.addString(biasName);
//                         reply.addInt(val);
//                         ok = true;
//                     } else {
//                         reply.addString("Right Unknown Bias");
//                         ok = false;
//                     }
//                 } else if (channel == "") {
//                     int val = vsctrlMngLeft.getBias(biasName);
//                     if(val >= 0) {
//                         reply.addString("Left: ");
//                         reply.addString(biasName);
//                         reply.addInt(val);
//                         ok = true;
//                     } else {
//                         reply.addString("Left Unknown Bias");
//                         ok = false;
//                     }
//                     val = vsctrlMngRight.getBias(biasName);
//                     if(val >= 0) {
//                         reply.addString("Right: ");
//                         reply.addString(biasName);
//                         reply.addInt(val);
//                         ok = true & ok;
//                     } else {
//                         reply.addString("RightUnknown Bias");
//                         ok = false;
//                     }
//                 }
//                 else {
//                     std::cout << "unrecognised channel" << std::endl;
//                     ok = false;
//                 }
//             }
//             break;

//         case COMMAND_VOCAB_SETBIAS:
//             rec = true;
//             {
//                 std::string biasName = command.get(1).asString();
//                 unsigned int biasValue = command.get(2).asInt32();
//                 std::string channel = command.get(3).asString();

//                 // setBias function
//                 if (channel == "left"){
//                     vsctrlMngLeft.setBias(biasName, biasValue);
//                     ok = true;
//                 } else if (channel == "right")
//                 {
//                     vsctrlMngRight.setBias(biasName, biasValue);
//                     ok = true;
//                 } else if (channel == "")
//                 {
//                     vsctrlMngLeft.setBias(biasName, biasValue);
//                     vsctrlMngRight.setBias(biasName, biasValue);
//                     ok = true;
//                 }
//                 else {
//                     std::cout << "unrecognised channel" << std::endl;
//                     ok =false;
//                 }
//             }
//             break;
//         case COMMAND_VOCAB_PROG:
//             rec= true;
//             {
//                 std::string channel = command.get(1).asString();

//                 // progBias function
//                 if (channel == "left"){
//                     vsctrlMngLeft.configureBiases();
//                     ok = true;
//                 } else if (channel == "right")
//                 {
//                     vsctrlMngRight.configureBiases();
//                     ok = true;
//                 }
//                 else if (channel == "")
//                 {
//                     vsctrlMngLeft.configureBiases();
//                     vsctrlMngRight.configureBiases();
//                     ok = true;
//                 } else {
//                     std::cout << "unrecognised channel" << std::endl;
//                     ok =false;
//                 }
//             }
//             break;
//         case COMMAND_VOCAB_PWROFF:
//             rec= true;
//             {   std::string channel = command.get(1).asString();

//                 if (channel == "left"){
//                     vsctrlMngLeft.suspend();
//                     ok = true;

//                 } else if (channel == "right") {
//                     vsctrlMngRight.suspend();
//                     ok = true;
//                 } else if (channel == "") { // if channel is not specified power off both
//                     vsctrlMngRight.suspend();
//                     vsctrlMngLeft.suspend();
//                     ok = true;
//                 } else {
//                     std::cout << "unrecognised channel" << std::endl;
//                     ok = false;

//                 }
//             }
//             break;

//         case COMMAND_VOCAB_PWRON:
//             rec= true;
//             {   std::string channel = command.get(1).asString();

//                 if (channel == "left"){
//                     vsctrlMngLeft.activate();
//                     ok = true;

//                 } else if (channel == "right") {
//                     vsctrlMngRight.activate();
//                     ok = true;
//                 } else if (channel == "") { // if channel is not specified power off both
//                     vsctrlMngRight.activate();
//                     vsctrlMngLeft.activate();
//                     ok = true;
//                 } else {
//                     std::cout << "unrecognised channel" << std::endl;
//                     ok = false;

//                 }
//             }
//             break;

//             //    case COMMAND_VOCAB_RST:
//             //        rec= true;
//             //    {   std::string channel = command.get(1).asString();

//             //        if (channel == "left"){
//             //            vsctrlMngLeft->chipReset();
//             //            ok = true;

//             //        } else if (channel == "right") {
//             //            vsctrlMngRight->chipReset();
//             //            ok = true;
//             //        } else if (channel == "") { // if channel is not specified power off both
//             //            vsctrlMngRight->chipReset();
//             //            vsctrlMngLeft->chipReset();
//             //            ok = true;
//             //        } else {
//             //            std::cout << "unrecognised channel" << std::endl;
//             //            ok = false;

//             //        }
//             //    }
//             //        break;

//         case COMMAND_VOCAB_SETSKIN:
//             rec = true;
//             {
//                 if(!skctrlMng.configureRegisters(command.tail()))
//                 {
//                     std::cout << "unable to set skin register " << command.get(1).asString() << std::endl;
//                     ok = false;
//                 } else {
//                     ok = true;
//                 }
//             }
//             break;

//     }
//     if (!rec)
//         ok = RFModule::respond(command,reply);

//     if (!ok) {
//         //reply.clear();
//         reply.addVocab(COMMAND_VOCAB_FAILED);
//     }
//     else
//         reply.addVocab(COMMAND_VOCAB_OK);

//     return ok;

// }


