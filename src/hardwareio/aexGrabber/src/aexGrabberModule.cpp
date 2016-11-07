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
 * @file aexGrabberModule.cpp
 * @brief Implementation of the aexGrabberModule (see header file).
 */

#include <aexGrabberModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/*
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 *  equivalent of the "open" method.
 */

bool aexGrabberModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    printf("configure in aexGrabberModule \n");

    //printf("moduleName  %s \n", moduleName.c_str());
    moduleName            = rf.check("name",
                           Value("/aexGrabber"),
                           "module name (string)").asString();
    printf("extracted the module name \n");


    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    printf("setting the module rootname \n");
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot",
                           Value("icub"),
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    /*
    * get the device name which will be used to read events
    */
    deviceName             = rf.check("deviceName",
                           Value("/dev/aerfx2_0"),
                           "Device name (string)").asString();
    devicePortName         =  deviceName ;
    printf("trying to connect to the device %s \n",devicePortName.c_str());







    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);                  // attach to port

    //bool _save = false;
    std::string deviceNum = "0";

    /*
    * get the file name of binaries when the biases are read from this file
    */
    binaryName             = rf.check("file",
                           Value("none"),
                           "filename of the binary (string)").asString();
    printf("trying to read %s  for biases \n",binaryName.c_str());
    binaryNameComplete = rf.findFile(binaryName.c_str());

    /*
    * get the file name of binaries when the biases are read from this file
    */
    dumpNameComplete = "";
    dumpName             = rf.check("dumpFile",
                           Value("none"),
                           "filename of the binary (string)").asString();
    printf("trying to save events in %s  \n",dumpName.c_str());
    dumpNameComplete = rf.findFile(dumpName.c_str());



    if(!strcmp(binaryName.c_str(),"none")) {
        printf("not reading from binary \n");
        D2Y=new device2yarp(devicePortName, false,binaryName);
        //D2Y->setFromBinary(false);
    }
    else {
        printf("reading from binary \n");
        //D2Y->setFromBinary(true);
        D2Y=new device2yarp(devicePortName, true, binaryNameComplete);
        //D2Y->setBinaryFile(f);
    }

    if(rf.check("onlyLeft")){
        D2Y->setOnlyLeft();
    }

    if (strcmp(dumpNameComplete.c_str(),"" )) {
        D2Y->setDumpEvent(true);
        D2Y->setDumpFile(dumpNameComplete);
    }
    else {
        D2Y->setDumpEvent(false);
    }

    D2Y->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool aexGrabberModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool aexGrabberModule::close() {
    handlerPort.close();
    D2Y->stop();
    /* stop the thread */
    return true;
}

bool aexGrabberModule::respond(const Bottle& command, Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?
    string helpMessage =  string(getName().c_str()) +
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
        cout << helpMessage;
        reply.addString("ok");
    }

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            reply.addString("");
            reply.addString("set fn \t: general set command ");
            reply.addString("");
            reply.addString("");
            reply.addString("");
            reply.addString("set left pr <int> \t: setting the costant time between saccadic events (default 3000) ");
            reply.addString("set left foll <double> \t: setting the coefficients to the default value ");
            reply.addString("set left diff <double> \t: setting of linear combination coefficient (map1) ");
            reply.addString("set left diffon<double> \t: setting of linear combination coefficient (map2) ");
            reply.addString("set left puy <double> \t: setting of linear combination coefficient (map3) ");
            reply.addString("set left refr <double> \t: setting of linear combination coefficient (map4)  ");
            reply.addString("set left req <double> \t: setting of linear combination coefficient (map5)  ");
            reply.addString("set left diffoff <double> \t: setting of linear combination coefficient (map6)  ");
            reply.addString("set left pux <double> \t: setting of linear combination coefficient (mapc1)  ");
            reply.addString("set left reqpd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("set left injgnd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("set left cas <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("");
            reply.addString("set right pr <int> \t: setting the costant time between saccadic events (default 3000) ");
            reply.addString("set right foll <double> \t: setting the coefficients to the default value ");
            reply.addString("set right diff <double> \t: setting of linear combination coefficient (map1) ");
            reply.addString("set right diffon<double> \t: setting of linear combination coefficient (map2) ");
            reply.addString("set right puy <double> \t: setting of linear combination coefficient (map3) ");
            reply.addString("set right refr <double> \t: setting of linear combination coefficient (map4)  ");
            reply.addString("set right req <double> \t: setting of linear combination coefficient (map5)  ");
            reply.addString("set right diffoff <double> \t: setting of linear combination coefficient (map6)  ");
            reply.addString("set right pux <double> \t: setting of linear combination coefficient (mapc1)  ");
            reply.addString("set right reqpd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("set right injgnd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("set right cas <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("");

            reply.addString("get fn \t: general get command ");
            reply.addString("");
            reply.addString("");
            reply.addString("get left pr <double> \t: setting the costant time between saccadic events (default 3000) ");
            reply.addString("get left foll <double> \t: setting the coefficients to the default value ");
            reply.addString("get left diff <double> \t: setting of linear combination coefficient (map1) ");
            reply.addString("get left diffon<double> \t: setting of linear combination coefficient (map2) ");
            reply.addString("get left puy <double> \t: setting of linear combination coefficient (map3) ");
            reply.addString("get left refr <double> \t: setting of linear combination coefficient (map4)  ");
            reply.addString("get left req <double> \t: setting of linear combination coefficient (map5)  ");
            reply.addString("get left diffoff <double> \t: setting of linear combination coefficient (map6)  ");
            reply.addString("get left pux <double> \t: setting of linear combination coefficient (mapc1)  ");
            reply.addString("get left reqpd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("get left injgnd <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("get left cas <double> \t: setting of linear combination coefficient (flow motion)  ");
            reply.addString("");
            reply.addString("");

            reply.addString("prog fn \t: general prog command ");
            reply.addString("");
            reply.addString("");
            reply.addString("prog left bias \t: asks to reprogram biases ");
            reply.addString("prog right bias \t: asks to reprogram biases ");

            reply.addString("dump on \t: asks to start dumping event in default file ");
            reply.addString("dump off \t: asks to stop dumping event in default file ");

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
    case COMMAND_VOCAB_NAME:
        rec = true;
        {
            // check and change filter name to pass on to the next filter
            string fName(command.get(1).asString());
            string subName;
            Bottle subCommand;
            int pos=1;
            //int pos = fName.find_first_of(filter->getFilterName());
            if (pos == 0){
                pos = fName.find_first_of('.');
                if (pos  > -1){ // there is a subfilter name
                    subName = fName.substr(pos + 1, fName.size()-1);
                    subCommand.add(command.get(0));
                    subCommand.add(Value(subName.c_str()));
                }
                for (int i = 2; i < command.size(); i++)
                    subCommand.add(command.get(i));
                //ok = filter->respond(subCommand, reply);
            }
            else{
                printf("filter name  does not match top filter name ");
                ok = false;
            }
        }
        break;
    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_LEFT: {
                switch(command.get(2).asVocab()) {
                case COMMAND_VOCAB_PR:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPR(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_FOLL:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setFOLL(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFF:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFF(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFFON:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFFON(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_PUY:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPUY(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REFR:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREFR(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REQ:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREQ(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFFOFF:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFFOFF(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_PUX:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPUX(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REQPD:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREQPD(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_INJGND:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setINJGND(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_CAS:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setCAS(w);
                    ok = true;
                } break;
                default: {
                } break;
                } //closing the switch internal

            } break;
            case COMMAND_VOCAB_RIGHT:{
                switch(command.get(2).asVocab()) {

                case COMMAND_VOCAB_PR:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPRRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_FOLL:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setFOLLRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFF:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFFRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFFON:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFFONRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_PUY:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPUYRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REFR:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREFRRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REQ:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREQRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_DIFFOFF:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setDIFFOFFRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_PUX:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setPUXRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_REQPD:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setREQPDRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_INJGND:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setINJGNDRight(w);
                    ok = true;
                } break;
                case COMMAND_VOCAB_CAS:{
                    double w = command.get(3).asDouble();
                    if(D2Y!=0)
                        D2Y->setCASRight(w);
                    ok = true;
                } break;
                default: {
                } break;
                }
            } break; //closing the RIGHT
            } break; //closing the SET
    case COMMAND_VOCAB_GET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_PR:{
                double w = D2Y->getPR();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_FOLL:{
                double w = D2Y->getFOLL();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_DIFF:{
                double w = D2Y->getDIFF();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_DIFFON:{
                double w = D2Y->getDIFFON();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_PUY:{
                double w = D2Y->getPUY();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_REFR:{
                double w = D2Y->getREFR();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_REQ:{
                double w = D2Y->getREQ();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_DIFFOFF:{
                double w = D2Y->getDIFFOFF();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_PUX:{
                double w = D2Y->getPUX();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_REQPD:{
                double w = D2Y->getREQPD();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_INJGND:{
                double w = D2Y->getINJGND();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CAS:{
                double w = D2Y->getCAS();
                reply.addDouble(w);
                ok = true;
            }
            break;
            default: {

            }
                break;
            }
        }
        break;
        case COMMAND_VOCAB_PROG:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_LEFT:{
                printf("request of reprogrammming biases arrived \n");
                D2Y->closeDevice();
                Time::delay(1);
                D2Y->setFromBinary(false);
                D2Y->prepareBiases();

                ok = true;
            }
            break;
            case COMMAND_VOCAB_RIGHT:{
                printf("request of reprogrammming biases arrived \n");
                D2Y->closeDevice();
                Time::delay(1);
                D2Y->setFromBinary(false);
                D2Y->prepareBiasesRight();

                ok = true;
            }
            break;
            default: {

            }
                break;
            }
        }
        break;
        case COMMAND_VOCAB_DUMP:
            rec = true;
            printf("recognised the command DUMP \n");
            {
                switch(command.get(1).asVocab()) {
                case COMMAND_VOCAB_ON: {
                    printf("request of start dumping events arrived \n");
                 string S = command.get(2).asString().c_str();
                   printf("Writing to file: %s \n",S.c_str());
    //	 D2Y->setDumpFile("dump.txt");
                D2Y->setDumpFile(S.c_str());
                    D2Y->setDumpEvent(true);
                    printf("success in opening the dump file \n");
                  ok = true;
                }
                    break;
                case COMMAND_VOCAB_OFF: {
                    printf("request of stop dumping events arrived \n");
                    D2Y->setDumpEvent(false);
                    ok = true;
                }
                    break;
                }
            }
            break;
        case COMMAND_VOCAB_SYNC:
            rec = true;
            printf("recognised the command SYNC \n");
            {
                    printf("request of adding the sync bit \n");
                    D2Y->setSyncBit();
                    printf("success in adding the sync bit \n");
                    ok = true;
            }
            break;
        }
    }
    mutex.post();

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

/* Called periodically every getPeriod() seconds */
bool aexGrabberModule::updateModule() {
    return true;
}

double aexGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

