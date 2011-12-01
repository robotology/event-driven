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
 * @file main.cpp
 * @brief main code for launching the pythonInterface
 */

#define COMMAND_VOCAB_ON      VOCAB2('o','n')
#define COMMAND_VOCAB_OFF     VOCAB3('o','f','f')
#define COMMAND_VOCAB_SET     VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET     VOCAB3('g','e','t')
#define COMMAND_VOCAB_RIGHT   VOCAB4('r','i','g','h')
#define COMMAND_VOCAB_LEFT    VOCAB4('l','e','f','t')
#define COMMAND_VOCAB_DUMP    VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_SYTH    VOCAB4('s','y','t','h')
#define COMMAND_VOCAB_SYTA    VOCAB4('s','y','t','a')
#define COMMAND_VOCAB_SYPA    VOCAB4('s','y','p','a')
#define COMMAND_VOCAB_SYPH    VOCAB4('s','y','p','h')
#define COMMAND_VOCAB_TPB     VOCAB3('t','p','b')
#define COMMAND_VOCAB_CDR     VOCAB3('c','d','r')
#define COMMAND_VOCAB_CDS     VOCAB3('c','d','s')
#define COMMAND_VOCAB_CDP     VOCAB3('c','d','p')
#define COMMAND_VOCAB_RPX     VOCAB3('r','p','x')
#define COMMAND_VOCAB_RPY     VOCAB3('r','p','y') 
#define COMMAND_VOCAB_IFR     VOCAB3('i','f','r')
#define COMMAND_VOCAB_IFT     VOCAB3('i','f','t')
#define COMMAND_VOCAB_IFL     VOCAB3('i','f','l'    )
#define COMMAND_VOCAB_CDOF    VOCAB4('c','d','o','f')
#define COMMAND_VOCAB_SYPW    VOCAB4('s','y','p','w')
#define COMMAND_VOCAB_SYW     VOCAB3('s','y','w')
#define COMMAND_VOCAB_CDON    VOCAB4('c','d','o','n')
#define COMMAND_VOCAB_CDD     VOCAB3('c','d','d')
#define COMMAND_VOCAB_EMCH    VOCAB4('e','m','c','h')
#define COMMAND_VOCAB_EMCT    VOCAB4('e','m','c','t')
#define COMMAND_VOCAB_CDI     VOCAB3('c','d','i')
#define COMMAND_VOCAB_CDRG    VOCAB4('c','d','r','g')
#define COMMAND_VOCAB_SELF    VOCAB4('s','e','l','f')
#define COMMAND_VOCAB_FOLL    VOCAB4('f','o','l','l')
#define COMMAND_VOCAB_ARBP    VOCAB4('a','r','b','p')
#define COMMAND_VOCAB_EMVL    VOCAB4('e','m','v','l')
#define COMMAND_VOCAB_CDC     VOCAB3('c','d','c')
#define COMMAND_VOCAB_EMVH    VOCAB4('e','m','v','h')
#define COMMAND_VOCAB_I2V     VOCAB3('i','2','v')

#define COMMAND_VOCAB_HELP    VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_NAME    VOCAB4('n','a','m','e')
#define COMMAND_VOCAB_SET     VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET     VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN     VOCAB3('r','u','n')
#define COMMAND_VOCAB_PROG    VOCAB4('p','r','o','g')
#define COMMAND_VOCAB_SUSPEND VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME  VOCAB3('r','e','s')
#define COMMAND_VOCAB_IS      VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED  VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK      VOCAB2('o','k')
#define COMMAND_VOCAB_LEFT    VOCAB4('l','e','f','t')
#define COMMAND_VOCAB_RIGHT   VOCAB4('r','i','g','h')
#define COMMAND_VOCAB_BIAS    VOCAB4('b','i','a','s')
#define COMMAND_VOCAB_DUMP    VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_OFF     VOCAB3('o','f','f')
#define COMMAND_VOCAB_ON      VOCAB2('o','n')

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

inline int fetchCommand(string str) {
    switch(str.length()) {
    case 2: {
        return VOCAB2(str[0], str[1]);
    }
        break;
    case 3: {
        return VOCAB3(str[0], str[1], str[2]);
    }
        break;
    case 4: {
        return VOCAB4(str[0], str[1], str[2],str[3]);
    
    }
        break;
    }

}


int main(int argc, char * argv[])
{
    Network yarp;

    Time::turboBoost(); 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("pythonInterface.ini");   //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    printf("HELP \n");
    printf("--rpcport /icub/rpc \n");
    printf("--request   'command:help;command:quit;command:set,int:10;command:set,double:10.0 string:Hello ' \n");
    
    // extracting running paramenter
    /* get the module name which will form the stem of all module port names */
    ConstString moduleName = rf.check("name", 
                           Value("/logSort"), 
                           "module name (string)").asString();

    /* get the module name which will form the stem of all module port names */
    ConstString externPort = rf.check("rpcport", 
                           Value("null"), 
                           "rpc port name (string)").asString();
    
    /* get the module name which will form the stem of all module port names */
    string requestList     = (string) rf.check("request", 
                           Value("null"), 
                           "requests list (string)").asString();
    
    //initialisation
    size_t foundColon, foundSemicolon, foundComma;
    string subpart, typeCommand, valueCommand;
    Bottle in;
    Bottle bot; //= _pOutPort->prepare();
    bot.clear();

    printf("Performing request on %s \n", externPort.c_str());
    Port outPort;
    outPort.open("/pythonInterface/request");
    printf("Connecting ports... \n");
    Network::connect("/pythonInterface/request", externPort.c_str());
    printf("Connection ultimated \n");
    
    // extracting commands
    printf("Request list: %s \n", requestList.c_str());
    foundSemicolon = requestList.find(';');

    while(foundSemicolon!=string::npos) {
        foundComma = requestList.find(',');
        while(foundComma<foundSemicolon) {
            subpart = requestList.substr(0,foundComma);
            printf("subpart : %s \n", subpart.c_str());
            requestList = requestList.substr(foundComma + 1);
            printf("requestList: %s \n", requestList.c_str());
            //interpreting the type of request
            foundColon = subpart.find(':');
            typeCommand = subpart.substr(0,foundColon);
            printf("       typeCommand : %s \n", typeCommand.c_str());
            if(!strcmp(typeCommand.c_str(),"command")) {
                printf("      this is a command \n");
                valueCommand = subpart.substr(foundColon + 1);
                printf("       valueCommand : %s \n", valueCommand.c_str());
                bot.addVocab(fetchCommand(valueCommand));
            }
            else if(!strcmp(typeCommand.c_str(),"int")) {
                printf("      this is the integer \n");
                bot.addInt(atoi(valueCommand.c_str()));
            }            
            foundComma = requestList.find(',');            
            foundSemicolon = requestList.find(';');
        }
        subpart = requestList.substr(0,foundSemicolon);
        printf("subpart : %s \n", subpart.c_str());
        requestList = requestList.substr(foundSemicolon + 1);
        printf("requestList: %s \n", requestList.c_str());
        //interpreting the type of request
        foundColon = subpart.find(':');
        typeCommand = subpart.substr(0,foundColon);
        printf("       typeCommand : %s \n", typeCommand.c_str());
        if(!strcmp(typeCommand.c_str(),"command")) {
            printf("      this is a command \n");
            valueCommand = subpart.substr(foundColon + 1);
            printf("       valueCommand : %s \n", valueCommand.c_str());
            bot.addVocab(fetchCommand(valueCommand));
        }
        else if(!strcmp(typeCommand.c_str(),"int")) {
            printf("      this is the integer \n");
            bot.addInt(atoi(valueCommand.c_str()));
        }            
        outPort.write(bot,in);
        printf("Answer : %s \n", in.toString().c_str());
        bot.clear();
        foundSemicolon = requestList.find(';');
    }
    
    // interpreting commands
    // sending commands

    
    bot.addVocab(COMMAND_VOCAB_RIGHT);
    bot.addInt(10);
    //_pOutPort->Content() = _outBottle;
    

    //closing the module
    printf("Closing the module ..... \n");
    outPort.close();
    
    return 0;
}



