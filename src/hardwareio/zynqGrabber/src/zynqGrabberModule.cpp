// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Chiara Bartolozzi, Arren Glover
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file zynqGrabberModule.cpp
 * @brief Implementation of the zynqGrabberModule (see header file).
 */

#include <iCub/zynqGrabberModule.h>

bool zynqGrabberModule::configure(yarp::os::ResourceFinder &rf) {

    //Process all parameters from both command-line and .ini file

    std::string moduleName =
            rf.check("name", yarp::os::Value("zynqGrabber")).asString();
    setName(moduleName.c_str());

    //get the device name which will be used to read events
    // zynq_hpu, zynq_spinn, ihead_aerfx2
    std::string device = rf.check("device", yarp::os::Value("zynq_sens")).asString();

    // get the correct clock of each device
    int clockPeriod = rf.check("clockPeriod", yarp::os::Value(100)).asInt(); //add check instead of default value

    // set the loopback (needed only for debug)
    std::string loopBack = rf.check("loopBack", yarp::os::Value("none")).asString(); //add check instead of default value




    //dvs or atis
    std::string chipName = rf.check("chip", yarp::os::Value("DVS")).asString();

    //bias values
    yarp::os::Bottle biaslistl = rf.findGroup(chipName + "_BIAS_LEFT");
    yarp::os::Bottle biaslistr = rf.findGroup(chipName + "_BIAS_RIGHT");

    //configuration classes for the zynq-based sensors
    vsctrlMngLeft = new vsctrlDevManager("left", chipName);
    vsctrlMngRight = new vsctrlDevManager("right", chipName);


    if(device == "zynq_sens") {

        if(!vsctrlMngLeft->setBias(biaslistl) || !vsctrlMngRight->setBias(biaslistr) ) {
            std::cerr << "Bias file required to run zynqGrabber" << std::endl;
            return false;
        }
        if(!vsctrlMngLeft->openDevice() || !vsctrlMngRight->openDevice()) {
            std::cerr << "Could not open the vsctrl devices" << std::endl;
            return false;
        }

    }


    if(device == "zynq_spinn" || device == "zynq_sens") {

        // class manageDevice for events
        aerManager = new aerDevManager(device, clockPeriod, loopBack);

        if(!aerManager->openDevice()) {
            std::cerr << "Could not open the aer device: " << device << std::endl;
            return false;
        }

    } else if(device == "ihead_sens") {

        // device manager for events
        aerManager = new aerfx2_0DevManager();

        if(!aerManager->openDevice()) {
            std::cerr << "Could not open the device: " << device << std::endl;
            return false;
        }

    } else {

        std::cout << "Device: " << device << " not known " << std::endl;
        return false;
    }

    //open rateThread device2yarp
    D2Y = new device2yarp();
    if(!D2Y->attachDeviceManager(aerManager))
    {
        //could not start the thread
        return false;
    }
    if(!D2Y->threadInit(moduleName, rf.check("strict"))) {
        //could not start the thread
        return false;
    }
    //open bufferedPort yarp2device
    if(!Y2D.attachDeviceManager(aerManager))
    {
        //could not start the thread
        return false;
    }
    if(!Y2D.open(moduleName))
    {
        std::cerr << " : Unable to open ports" << std::endl;
        return false;
    }

    // attach a port of the same name as the module (prefixed with a /)
    //to the module so that messages received from the port are redirected to
    //the respond method
    if (!handlerPort.open("/" + moduleName)) {
        std::cout << "Unable to open RPC port @ /" << moduleName << std::endl;
        return false;
    }
    attach(handlerPort);
    D2Y->start();

    return true;
}

bool zynqGrabberModule::interruptModule() {
    handlerPort.interrupt();
    Y2D.interrupt();
    // D2Y ???
    return true;
}

bool zynqGrabberModule::close() {

    handlerPort.close();        // rpc of the RF module
    Y2D.close();
    D2Y->stop();                // bufferedport from yarp to device

    aerManager->closeDevice();  // device
    vsctrlMngLeft->closeDevice();
    vsctrlMngRight->closeDevice();

    return true;
}

/* Called periodically every getPeriod() seconds */
bool zynqGrabberModule::updateModule() {

    return true;
}

double zynqGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}


// RPC
bool zynqGrabberModule::respond(const yarp::os::Bottle& command,
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
    case COMMAND_VOCAB_SUSPEND:
        rec = true;
    {
        D2Y->suspend();
        ok = true;
    }
        break;
    case COMMAND_VOCAB_RESUME:
        rec = true;
    {
        D2Y->resume();
        ok = true;
    }
        break;
    case COMMAND_VOCAB_SETBIAS:
        rec = true;
    {
        std::string biasName = command.get(1).asString();
        unsigned int biasValue = command.get(2).asInt();
        std::string channel = command.get(3).asString();

        // setBias function
        if (channel == "left"){
            vsctrlMngLeft->setBias(biasName, biasValue);
            ok = true;
        } else if (channel == "right")
        {
            vsctrlMngRight->setBias(biasName, biasValue);
            ok = true;
        } else if (channel == "")
        {
            vsctrlMngLeft->setBias(biasName, biasValue);
            vsctrlMngRight->setBias(biasName, biasValue);
            ok = true;
        }
        else {
            std::cout << "unrecognised channel" << std::endl;
            ok =false;
        }
    }
        break;
    case COMMAND_VOCAB_PROG:
        rec= true;
    {
        std::string channel = command.get(1).asString();

        // progBias function
        if (channel == "left"){
            vsctrlMngLeft->programBiases();
            ok = true;
        } else if (channel == "right")
        {
            vsctrlMngRight->programBiases();
            ok = true;
        }
        else if (channel == "")
        {
            vsctrlMngLeft->programBiases();
            vsctrlMngRight->programBiases();
            ok = true;
        } else {
            std::cout << "unrecognised channel" << std::endl;
            ok =false;
        }
    }
        break;
    case COMMAND_VOCAB_PWROFF:
        rec= true;
    {   std::string channel = command.get(1).asString();

        if (channel == "left"){
            vsctrlMngLeft->chipPowerDown();
            ok = true;

        } else if (channel == "right") {
            vsctrlMngRight->chipPowerDown();
            ok = true;
        } else if (channel == "") { // if channel is not specified power off both
            vsctrlMngRight->chipPowerDown();
            vsctrlMngLeft->chipPowerDown();
            ok = true;
        } else {
            std::cout << "unrecognised channel" << std::endl;
            ok = false;

        }
    }
        break;

    case COMMAND_VOCAB_PWRON:
        rec= true;
    {   std::string channel = command.get(1).asString();

        if (channel == "left"){
            vsctrlMngLeft->chipPowerUp();
            ok = true;

        } else if (channel == "right") {
            vsctrlMngRight->chipPowerUp();
            ok = true;
        } else if (channel == "") { // if channel is not specified power off both
            vsctrlMngRight->chipPowerUp();
            vsctrlMngLeft->chipPowerUp();
            ok = true;
        } else {
            std::cout << "unrecognised channel" << std::endl;
            ok = false;

        }
    }
        break;

    case COMMAND_VOCAB_RST:
        rec= true;
    {   std::string channel = command.get(1).asString();

        if (channel == "left"){
            vsctrlMngLeft->chipReset();
            ok = true;

        } else if (channel == "right") {
            vsctrlMngRight->chipReset();
            ok = true;
        } else if (channel == "") { // if channel is not specified power off both
            vsctrlMngRight->chipReset();
            vsctrlMngLeft->chipReset();
            ok = true;
        } else {
            std::cout << "unrecognised channel" << std::endl;
            ok = false;

        }
    }
        break;


    }

    if (!rec)
        ok = RFModule::respond(command,reply);

    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;

    return true;
}


