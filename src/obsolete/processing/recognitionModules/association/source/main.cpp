#include "association.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

using namespace std;
int main(int argc, char *argv[])
{
    yarp::os::Network yarpNet;

    yarp::os::Property params;
    params.fromCommand(argc, argv);
//    if(!params.check("bufsz"))
//    {
//        fprintf(stderr, "Please specify the size of the buffer to be received\n");
//        fprintf(stderr, "--bufsz size_of_the_buffer (e.g. 32768)\n");
//        return -1;
//    }
    if(!params.check("source"))
    {
        fprintf(stderr, "Please specify the address event representation origin\n");
        fprintf(stderr, "--source type (e.g. icub)\n");
        return -1;
    }
    if(!params.check("f"))
    {
        fprintf(stderr, "Please specify the features file to use\n");
        fprintf(stderr, "--f fileName (e.g. ???)\n");
        return -1;
    }
    if(!params.check("swin"))
    {
        fprintf(stderr, "Please specify the spacial window size\n");
        fprintf(stderr, "--swin size (e.g. 5)\n");
        return -1;
    }
    if(!params.check("twin"))
    {
        fprintf(stderr, "Please specify the temporal window size\n");
        fprintf(stderr, "--twin ms (e.g. 5000)\n");
        return -1;
    }
    if(!params.check("evalsim"))
    {
        fprintf(stderr, "Please specify the minimum eigenvalue similarity\n");
        fprintf(stderr, "--evalsim val (e.g. 0.99)\n");
        return -1;
    }
    if(!params.check("evecsim"))
    {
        fprintf(stderr, "Please specify the minimum eigenvector similarity\n");
        fprintf(stderr, "--evecsim val (e.g. 0.98481)\n");
        return -1;
    }
    if(!params.check("dim"))
    {
        fprintf(stderr, "Please specify the dimensionality of the pca (possible value [2 3])\n");
        fprintf(stderr, "--dim val (e.g. 2)\n");
        return -1;
    }
    if(!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    if(!params.check("eye"))
    {
        fprintf(stderr, "Please specify the eye\n");
        fprintf(stderr, "--eye side (e.g. left)\n");
        return -1;
    }
    string source=params.find("source").asString().c_str();
    unsigned int type=0;
    if(!source.compare("dvs"))
    {
        if(!params.check("type"))
            std::cout << "Error: source set at 'dvs' but type missing" << std::endl;
        else
            type=(unsigned int)params.find("type").asInt();
    }
//    bool save=false;
//    if(params.check("save"))
//        save=true;
    string robotName=params.find("robot").asString().c_str();
    string eye=params.find("eye").asString().c_str();
    string localPort="/";
    localPort+=robotName;
    localPort+="/";
    localPort+=eye;
    localPort+="/association:i";

    yarp::os::BufferedPort<emorph::ehist::eventHistBuffer> outputPort;
    string outPort="/";
    outPort+=robotName;
    outPort+="/";
    outPort+=eye;
    outPort+="/association:o";
    outputPort.open(outPort.c_str());

    association associator( source,
                            type,
                            eye,
                            params.find("f").asString().c_str(),
                            (unsigned int)params.find("swin").asInt(),
                            (unsigned int)params.find("twin").asInt(),
                            params.find("evalsim").asDouble(),
                            params.find("evecsim").asDouble(),
                            (unsigned int)params.find("dim").asInt(),
                            params.check("save"),
                            &outputPort);
    associator.useCallback();
    associator.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    associator.close();
    associator.disableCallback();
}
