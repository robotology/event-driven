// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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

    /* Process all parameters from both command-line and .ini file */
    std::cout << "Configuring the zynqGrabberModule" << std::endl;
    
    //printf("moduleName  %s \n", moduleName.c_str());
    std::string moduleName =
            rf.check("name", yarp::os::Value("zynqGrabber")).asString();
    setName(moduleName.c_str());


     //get the device name which will be used to read events
    device = rf.check("device",
                          yarp::os::Value("zynq")).asString();

    
    std::string chipName =
    rf.check("name", yarp::os::Value("dvs")).asString();
    setName(chipName.c_str());

    
    //get the maximum buffer size to use for device reading
    int maxBufferSize = rf.check("bufferSize", yarp::os::Value(65536)).asInt();

    //TODO: get all the bias settings

    // class configManager for left and right sensors
    configLeft = new configManager("left", chipName);
    configRight = new configManager("right", chipName);


    if (device == "zynq"){

        std::string deviceName = "/dev/spinn2neu";
        // class manageDevice for events
        devManager = new deviceManager(deviceName, maxBufferSize);
        if(!devManager->openDevice()) {
            std::cerr << "Could not open the device: " << deviceName << std::endl;
            return false;
        }
        
        std::string configName = "/dev/i2c";
        // class manageDevice for configuration
        cfgManager = new deviceManager(configName, maxBufferSize);
        if(!cfgManager->openDevice()) {
            std::cerr << "Could not open the device: " << configName << std::endl;
            return false;
        }
        configLeft -> attachDeviceManager(cfgManager);
        configRight -> attachDeviceManager(cfgManager);

        configLeft -> programBiases();
        configRight -> programBiases();

        
    } else if (device == "ihead")
    {
        std::string deviceName = "/dev/aerfx2_0";
        // class manageDevice for events and configuration
        devManager = new deviceManager(deviceName, maxBufferSize);
        if(!devManager->openDevice()) {
            std::cerr << "Could not open the device: " << deviceName << std::endl;
            return false;
        }
        configLeft -> attachDeviceManager(devManager);
        configRight -> attachDeviceManager(devManager);

        configLeft -> programBiasesAex();
        configRight -> programBiasesAex();
        
    } else {
        std::cout << "Device: " << device << " not known " << std::endl;
        return false;
    }
    

    // todo --- program registers for ATIS
    
    //open rateThread device2yarp
    D2Y = new device2yarp();
    D2Y->attachDeviceManager(devManager);
    if(!D2Y->threadInit(moduleName)) {
        //could not start the thread
        return false;
    }
    D2Y->start();
    
    
    //open bufferedPort yarp2device
    Y2D.attachDeviceManager(devManager);
    if(!Y2D.open(moduleName))
    {
        std::cerr << " : Unable to open ports" << std::endl;
        return false;
    }

    // attach a port of the same name as the module (prefixed with a /)
    //to the module so that messages received from the port are redirected to
    //the respond method
    std::string handlerPortName =  "/" + moduleName;
    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << "Unable to open RPC port @ " << handlerPortName << std::endl;
        return false;
    }
    attach(handlerPort);
    
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
    
    devManager->closeDevice();  // device

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
                configLeft->setBias(biasName, biasValue);
                ok = true;
            } else if (channel == "right")
            {
                configRight->setBias(biasName, biasValue);
                ok = true;
            } else {
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
                if (device == "ihead") {
                    configLeft->programBiases();
                    ok = true;
                } else if (device == "zynq")
                {
                    configLeft->programBiasesAex();
                    ok = true;
                } else {
                    std::cout << "unrecognised device" << std::endl;
                    ok = false;
                }
                
            } else if (channel == "right")
            {
                if (device == "ihead") {
                    configRight->programBiases();
                    ok = true;
                } else if (device == "zynq")
                {
                    configRight->programBiasesAex();
                    ok = true;
                } else {
                    std::cout << "unrecognised device" << std::endl;
                    ok = false;
                }
                
            } else {
                std::cout << "unrecognised channel" << std::endl;
                ok =false;
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


