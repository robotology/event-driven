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
 * @file chronocamGrabberModule.cpp
 * @brief Implementation of the chronocamGrabberModule (see header file).
 */

#include <chronocamGrabberModule.h>

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    //we do a network check only after programming the biases to ensure the
    //cameras are configured in a good operational state.

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("chronocamGrabber.ini"); //overridden by --from parameter
    rf.setDefaultContext("eventdriven");   //overridden by --context parameter
    rf.configure(argc, argv);

    chronocamGrabberModule module;
    return module.runModule(rf);

}


bool chronocamGrabberModule::configure(yarp::os::ResourceFinder &rf) {


    std::string moduleName = rf.check("name", yarp::os::Value("/chronocamGrabber")).asString();
    setName(moduleName.c_str());
    //bool strict = rf.check("strict") && rf.check("strict", yarp::os::Value(true)).asBool();
    bool errorcheck = rf.check("errorcheck") && rf.check("errorcheck", yarp::os::Value(true)).asBool();
    bool verbose = rf.check("verbose") && rf.check("verbose", yarp::os::Value(true)).asBool();
    bool biaswrite = rf.check("biaswrite") && rf.check("biaswrite", yarp::os::Value(true)).asBool();
    bool jumpcheck = rf.check("jumpcheck") && rf.check("jumpcheck", yarp::os::Value(true)).asBool();


    vsctrlMng = vDevCtrl();

    //bias values
    yarp::os::Bottle biaslist = rf.findGroup("ATIS_BIAS");

    bool con_success = false;

    if(!vsctrlMng.setBias(biaslist)) {
        std::cerr << "Bias file required to run chronocamGrabber" << std::endl;
        return false;
    }
    std::cout << std::endl;
    if(!vsctrlMng.connect())
        std::cerr << "Could not connect to vision controller" << std::endl;
    else
        if(!vsctrlMng.configure(verbose)) {
            std::cerr << "Could not configure camera" << std::endl;
        } else {
            con_success = true;
        }


    if(!con_success) {
        std::cerr << "A configuration device was specified but could not be connected" << std::endl;
        return false;
    }

    bool yarppresent = yarp::os::Network::checkNetwork();
    if(!yarppresent)
        yError() << "Could not connect to YARP network";

    if(!yarppresent || biaswrite) {
        vsctrlMng.disconnect(true);
        std::cout << "Camera off" << std::endl;
        return false;
    }


    int readPacketSize = 8 * rf.check("readPacketSize", yarp::os::Value("512")).asInt32();
    int bufferSize     = 8 * rf.check("bufferSize", yarp::os::Value("5120")).asInt32();
    int maxBottleSize  = 8 * rf.check("maxBottleSize", yarp::os::Value("5120")).asInt32();

    if(!D2Y.initialise(vsctrlMng.getStream(), moduleName, errorcheck, bufferSize, readPacketSize, maxBottleSize)) {
        std::cout << "A data device was specified but could not be initialised" << std::endl;
        return false;
    } else {
        //see if we want to apply a filter to the events
        if(rf.check("applyFilter", yarp::os::Value("false")).asBool()) {

            yarp::os::Bottle filp = rf.findGroup("FILTER_PARAMS");
            if(!filp.isNull()) {
                yInfo() << "APPLYING FILTER";
                std::cout << filp.toString() << std::endl;

                D2Y.initialiseFilter(true,
                                     filp.find("width").asInt32(),
                                     filp.find("height").asInt32(),
                                     filp.find("tsize").asInt32(),
                                     filp.find("ssize").asInt32());
        }

            if(jumpcheck) {
                yInfo() << "CHECKING FOR TIMESTAMP JUMPS";
                D2Y.checkForTSJumps();
            }
            D2Y.start();
        }

    }

    if (!handlerPort.open(moduleName)) {
        std::cout << "Unable to open RPC port @ /" << moduleName << std::endl;
        return false;
    }
    attach(handlerPort);

    return true;
}

bool chronocamGrabberModule::interruptModule() {
    std::cout << "breaking YARP connections.. ";
    handlerPort.close();        // rpc of the RF module
    D2Y.stop();                // bufferedport from yarp to device
    std::cout << "done" << std::endl;

    std::cout << "closing device drivers.. ";
    vsctrlMng.disconnect(true);
    std::cout << "done" << std::endl;
    return true;
}

bool chronocamGrabberModule::close() {

    return true;
}

/* Called periodically every getPeriod() seconds */
bool chronocamGrabberModule::updateModule() {

    return !isStopping();
}

double chronocamGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}


// RPC
bool chronocamGrabberModule::respond(const yarp::os::Bottle& command,
                                yarp::os::Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?
    std::string helpMessage =  std::string(getName().c_str()) +
            " commands are: \n" +
            "help \n" +
            "quit \n" +
            "set thr <n> ... set the threshold \n" +
            "(where <n> is an integer number) \n";

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }

    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
    {
        reply.addString("many");
        reply.addString("help");

        reply.addString("");

        ok = true;
    }
        break;
//    case COMMAND_VOCAB_SUSPEND:
//        rec = true;
//    {
//        //D2Y.suspend();
//        std::cout << "Not implemented" << std::endl;
//        ok = true;
//    }
//        break;
//    case COMMAND_VOCAB_RESUME:
//        rec = true;
//    {
//        //D2Y.resume();
//        std::cout << "Not implemented" << std::endl;
//        ok = true;
//    }
//        break;
    case COMMAND_VOCAB_GETBIAS:
        rec = true;
    {
        std::string biasName = command.get(1).asString();

        // setBias function
        int val = vsctrlMng.getBias(biasName);
        if(val >= 0) {
            reply.addString("Camera: ");
            reply.addString(biasName);
            reply.addInt32(val);
            ok = true;
        } else {
            reply.addString("Unknown Bias");
            ok = false;
        }
    }
        break;

    case COMMAND_VOCAB_SETBIAS:
        rec = true;
    {
        std::string biasName = command.get(1).asString();
        unsigned int biasValue = command.get(2).asInt32();

        // setBias function
        vsctrlMng.setBias(biasName, biasValue);
        ok = true;
    }
        break;
    case COMMAND_VOCAB_PROG:
        rec= true;
    {
        // progBias function
        vsctrlMng.configureBiases();
        ok = true;
    }
        break;
    case COMMAND_VOCAB_PWROFF:
        rec= true;

    {
        vsctrlMng.suspend();
        ok = true;
    }
        break;

    case COMMAND_VOCAB_PWRON:
        rec= true;
    {
            vsctrlMng.activate();
            ok = true;
    }
        break;

    }

    if (!rec)
        ok = RFModule::respond(command,reply);

    if (!ok) {
        //reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;

}


