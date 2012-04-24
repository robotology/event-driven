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
    command=0;
    //now set the shoulder to some value
    command[0]=0;
    command[1]=0;
    command[2]=0;
    command[3]=0;
    command[4]=0;
    command[5]=0;

    pos->positionMove(command.data());//(4,deg);
    
    bool done=false;

    /*while(!done)
    {
        pos->checkMotionDone(&done);
        Time::delay(0.1);
    }
*/
    int times=0;
    yarp::os::Bottle bot; //= _pOutPort->prepare();
    bot.clear();
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_ON);
    Bottle inOn;
    _pOutPort->write(bot,inOn);
    Time::delay(0.1);
    fprintf(stderr, "Start saccade(s), number of repetition: %d", nOl);
    while(times<nOl)
    {
        times++;
        

/*        if (times%2)
        {
            command[0]=0;
            command[1]=0;
            command[2]=0;   
            command[3]=0;
            command[4]=11;
            command[5]=0;
        }
        else
        {
            command[0]=0;
            command[1]=0;
            command[2]=0;
            command[3]=0;
            command[4]=0;
            command[5]=0;
        }
        pos->positionMove(command.data());
        Time::delay(0.1);*/


	/*Horizontal saccade*/
	command[4]=-5;
	moveJoints(pos, command);
	/*command[4]=0;
	moveJoints(pos, command);*/
	command[4]=5;
	moveJoints(pos, command);	
	command[4]=0;
	moveJoints(pos, command);
	
	/*Vertical saccade*/
	command[3]=-5;
	moveJoints(pos, command);
	/*command[3]=0;
	moveJoints(pos, command);*/
	command[3]=5;
	moveJoints(pos, command);
	command[3]=0;
	moveJoints(pos, command);

        
/*        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                encs->getEncoders(encoders.data());
                printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);
            }
*/
    }
    bot.clear();
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_OFF);
    Bottle inOff;
    _pOutPort->write(bot,inOff);

    _pOutPort->close();
    robotDevice.close();
    
    return 0;
}
