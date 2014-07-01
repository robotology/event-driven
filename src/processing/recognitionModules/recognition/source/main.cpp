#include "recognition.hpp"

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
    if(!params.check("f"))
    {
        fprintf(stderr, "Please specify the features file to use\n");
        fprintf(stderr, "--f fileName (e.g. ???)\n");
        return -1;
    }
    if(!params.check("saccD"))
    {
        fprintf(stderr, "Please specify the saccade mean duration\n");
        fprintf(stderr, "--saccD us (e.g. 3.5E6)\n");
        return -1;
    }
    if(!params.check("saccB"))
    {
        fprintf(stderr, "Please specify the temporal window size\n");
        fprintf(stderr, "--saccB us (e.g. 5000)\n");
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
        fprintf(stderr, "Please specify the eye of the robot affected\n");
        fprintf(stderr, "--eye side (e.g. left)\n");
        return -1;
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
    localPort+="/recognition/hist:i";

    string outPort="/";
    outPort+=robotName;
    outPort+="/";
    outPort+=eye;
    outPort+="/recognition/res:o";

    yarp::os::BufferedPort<emorph::reco::objDistBuffer> portOut;
    portOut.open(outPort.c_str());

    recognition recognitor( params.find("f").asString().c_str(),
                            (unsigned int)params.find("saccD").asInt(),
                            (unsigned int)params.find("saccB").asInt(),
                            eye
                            );
    recognitor.setOutPort(&portOut);

    recognitor.useCallback();
    recognitor.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    recognitor.close();
    recognitor.disableCallback();
}

