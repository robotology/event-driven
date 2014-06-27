#include "basicCFrameConverter.hpp"

#include <iostream>
#include <yarp/os/Network.h>

using namespace yarp::os;
int main(int argc, char *argv[])
{
	Network yarp;

    yarp::os::Property params;
    params.fromCommand(argc, argv);
    if(!params.check("source"))
    {
        fprintf(stderr, "Please specify the source of the events\n");
        fprintf(stderr, "--src source (available [icub dvs])\n");
        return -1;
    }
    string source=params.find("source").asString().c_str();
    uint type=0;
    if(!source.compare("dvs"))
    {
        if(!params.check("type"))
        {
            fprintf(stderr, "dvs source selected, please specify the type\n");
            fprintf(stderr, "--type num (available [4 6 8])\n");
            return -1;
        }
        else
            type=(unsigned int)params.find("type").asInt();
    }    
    if(!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    unsigned int accTime=5000;
    if(params.check("acc"))
        accTime=(unsigned int)params.find("acc").asInt();
    unsigned int ori=0;
    if(params.check("ori"))
        ori=(unsigned int)params.find("ori").asInt();

    string robotName=params.find("robot").asString().c_str();
    string localPort="/";
    localPort+=robotName;
    localPort+="/bfc:i";
    
    
    basicCFrameConverter bfc(128, 128, accTime, source, type, ori);
	bfc.useCallback();
	bfc.open(localPort.c_str());

	std::string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
	bfc.close();
    bfc.disableCallback();
    return 0;
}
