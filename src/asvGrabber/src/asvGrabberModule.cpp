// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file asvGrabberModule.cpp
 * @brief Implementation of the asvGrabberModule (see header file).
 */

#include <iCub/asvGrabberModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool asvGrabberModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/asvGrabber"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
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

    bool _save = false;
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
        D2Y=new asvGrabberThread(devicePortName, false,binaryName);
        //D2Y->setFromBinary(false);
    }
    else {
        printf("reading from binary \n");
        //D2Y->setFromBinary(true);
        D2Y=new asvGrabberThread(devicePortName, true, binaryNameComplete);        
        //D2Y->setBinaryFile(f);
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

bool asvGrabberModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool asvGrabberModule::close() {
    handlerPort.close();
    D2Y->stop();
    /* stop the thread */
    return true;
}

bool asvGrabberModule::respond(const Bottle& command, Bottle& reply) {
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
                                
            case COMMAND_VOCAB_SYTH:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynThr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SYTA:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynTau(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SYPA:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynPxlTau(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SYPH:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynPxlThr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_TPB:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setTestPbias(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDR:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDRefr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDS:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDSf(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDP:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDPr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_RPX:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setReqPuX(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_RPY:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setReqPuY(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_IFR:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setIFRf(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_IFT:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setIFThr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_IFL:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setIFLk(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDOF:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDOffThr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SYPW:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynPxlW(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SYW:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSynW(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDON:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDOnThr(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDD:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDDiff(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_EMCH:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setEMCompH(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_EMCT:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setEMCompT(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDI:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDIoff(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDRG:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDRGnd(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_SELF:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setSelf(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_FOLL:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setFollBias(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_ARBP:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setArbPd(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_EMVL:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setEMVrefL(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_CDC:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setCDCas(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_EMVH:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setEMVrefH(w);
                ok = true;
            } break;
            case COMMAND_VOCAB_I2V:{
                double w = command.get(2).asDouble();
                if(D2Y!=0)
                    D2Y->setI2V(w);
                ok = true;
            } break;
            
            default: {
            } break;
            
            
            } // closing the inner switch
                       
        } break; //closing the SET
    case COMMAND_VOCAB_GET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SYTH:{
                double w = D2Y->getSynThr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SYTA:{
                double w = D2Y->getSynTau();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SYPA:{
                double w = D2Y->getSynPxlTau();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SYPH:{
                double w = D2Y->getSynPxlThr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_TPB:{
                double w = D2Y->getTestPBias();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDR:{
                double w = D2Y->getCDRefr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDS:{
                double w = D2Y->getCDSf();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDP:{
                double w = D2Y->getCDPr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_RPX:{
                double w = D2Y->getReqPuX();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_RPY :{
                double w = D2Y->getReqPuY();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_IFR :{
                double w = D2Y->getIFRf();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_IFT :{
                double w = D2Y->getIFThr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_IFL :{
                double w = D2Y->getIFLk();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDOF:{
                double w = D2Y->getCDOffThr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SYPW:{
                double w = D2Y->getSynPxlW();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SYW:{
                double w = D2Y->getSynW();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDON:{
                double w = D2Y->getCDOnThr();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDD:{
                double w = D2Y->getCDDiff();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_EMCH:{
                double w = D2Y->getEMCompH();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_EMCT:{
                double w = D2Y->getEMCompT();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDI:{
                double w = D2Y->getCDIoff();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDRG:{
                double w = D2Y->getCDRGnd();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SELF:{
                double w = D2Y->getSelf();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_FOLL:{
                double w = D2Y->getFollBias();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_ARBP:{
                double w = D2Y->getArbPd();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_EMVL:{
                double w = D2Y->getEMVrefL();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_CDC :{
                double w = D2Y->getCDCas();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_EMVH:{
                double w = D2Y->getEMVrefH();
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_I2V:{
                double w = D2Y->getI2V();
                reply.addDouble(w);
                ok = true;
            }
            break;;
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

    } //end of the outer switch




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
bool asvGrabberModule::updateModule() {
    return true;
}

double asvGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

