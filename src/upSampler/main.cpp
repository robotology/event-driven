// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

#define COMMAND_VOCAB_ON    VOCAB2('o','n')
#define COMMAND_VOCAB_OFF   VOCAB3('o','f','f')
#define COMMAND_VOCAB_DUMP  VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_SYNC  VOCAB4('s','y','n','c')

#define DELTAENC 0.0000001

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

void moveJoints(IPositionControl *_pos, Vector& _command)
{
    _pos->positionMove(_command.data());
    Time::delay(0.1);
}

int main(int argc, char *argv[]) 
{
    Network yarp;

    Port* _pOutPort = new Port;
    //_options.portName+="/command:o";
    std::string portName="/simpleSaccade/cmd:o";
    _pOutPort->open(portName.c_str());

    Property params;
    params.fromCommand(argc, argv);
    if(params.check("help"))
    {
        fprintf(stderr, "%s --robot robotName --loop numberOfLoop", argv[0]);
    }
    
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    if (!params.check("loop"))
    {
        fprintf(stderr, "Please specify the number of repetition\n");
        fprintf(stderr, "--loop number\n");
        return -1;
    }
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/head"; //"/right_arm"

    //int nOl=atoi(params.find("loop").asString().c_str());
    int nOl=params.find("loop").asInt();

    Network::connect(portName.c_str(), "/aexGrabber");

    std::string localPorts="/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 100.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    //pos->setRefSpeeds(tmp.data()))
    
    //fisrst zero all joints
    //
    for(i=0; i<nj; i++)
        command[i]=0;
    /******************************
    * SPECIFIC STARTING POSITIONS *
    ******************************/
    command[0]=-30;

    pos->positionMove(command.data());//(4,deg);

    double startPos3;
    double startPos4;

    encs->getEncoder(3, &startPos3);
    encs->getEncoder(4, &startPos4);
    printf("start position value of joint 3: %lf\n", startPos3);
    printf("start position value of joint 4: %lf\n", startPos4);
    bool first=true;
    int deltaSacc=2;
    int times=0;
    yarp::os::Bottle bot; //= _pOutPort->prepare();
    bot.clear();
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_ON);
    Bottle inOn;
    _pOutPort->write(bot,inOn);

    Time::delay(0.1);
    
    fprintf(stderr, "Start saccade(s), number of repetition: %d\n", nOl);
    while(times<nOl)
    {
        times++;
        
	/*Horizontal saccade*/
	command[4]=-deltaSacc;
	//moveJoints(pos, command);
    pos->positionMove(command.data());
    if(first)
    {
        double curPos;
        encs->getEncoder(4, &curPos);
    	printf("current position value of joint 4: %lf\n", curPos);
        while((curPos>=startPos4-DELTAENC) && (curPos<=startPos4+DELTAENC))
        {
    	    printf("current position value of joint 4: %lf\n", curPos);
            encs->getEncoder(4, &curPos);
        }
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SYNC);
        Bottle inStart;
        _pOutPort->write(bot,inStart);
    	printf("1st synch asked\n");
        first=false;
    }
    Time::delay(0.1);


	/*command[4]=0;
	moveJoints(pos, command);*/
	command[4]=deltaSacc;
	moveJoints(pos, command);	
	command[4]=0;
	moveJoints(pos, command);


	/*Vertical saccade*/
	command[3]=-deltaSacc;
	moveJoints(pos, command);
	/*command[3]=0;
	moveJoints(pos, command);*/
	command[3]=deltaSacc;
	moveJoints(pos, command);
	command[3]=0;
	moveJoints(pos, command);
    if(times>=nOl)
    {
        double curPos;
        encs->getEncoder(3, &curPos);
    	printf("current position value of joint 3: %lf\n", curPos);
/*        while((curPos<startPos3-DELTAENC) || (curPos>startPos3+DELTAENC))
        {
	        command[3]=0;
    	    printf("current position value of joint 3: %lf\n", curPos);
            encs->getEncoder(3, &curPos);
        }*/
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SYNC);
        Bottle inEnd;
        _pOutPort->write(bot,inEnd);
    	printf("2nd synch asked\n");
    }

/*        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                encs->getEncoders(encoders.data());
                printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);
            }
*/
    }
	Time::delay(0.1);
    /*bot.clear();
    bot.addVocab(COMMAND_VOCAB_SYNC);
    Bottle inEnd;
    _pOutPort->write(bot,inEnd);
    */

    bot.clear();
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_OFF);
    Bottle inOff;
    _pOutPort->write(bot,inOff);

    _pOutPort->close();
    robotDevice.close();
    
    return 0;
}
