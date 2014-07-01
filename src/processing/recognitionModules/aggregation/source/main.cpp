#include "aggregation.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

using namespace std;
int main(int argc, char *argv[])
{
    yarp::os::Network yarpNet;

    yarp::os::Property params;
    params.fromCommand(argc, argv);
    if(!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    if(!params.check("f"))
    {
        fprintf(stderr, "Please specify the file containing the img's roots\n");
        fprintf(stderr, "--f file (e.g my/img/root/file.img)\n");
        return -1;
    }
    string robotName=params.find("robot").asString().c_str();
    string localPort="/";
    localPort+=robotName;
    localPort+="/aggregation/dist:i";

    string outPort="/";
    outPort+=robotName;
    outPort+="/aggregation/res:o";

    yarp::os::BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > portOut;
    portOut.open(outPort.c_str());
    
    std::string imFile=params.find("f").asString().c_str();
    aggregation aggregator( imFile,
                            &portOut);

    aggregator.useCallback();
    aggregator.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    aggregator.close();
    aggregator.disableCallback();
}

