#include "extraction.hpp"

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
    if(!params.check("source") || !params.check("eye") || !params.check("swin") || !params.check("twin") || !params.check("evalsim") || !params.check("evecsim") || !params.check("dim") || !params.check("robot"))
    {
        std::cout << "Error: missing parameter(s)" << std::endl;
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
    
    string robotName=params.find("robot").asString().c_str();
    string localPort="/";
    localPort+=robotName;
    localPort+="/extraction:i";

    extraction extractor(   params.find("source").asString().c_str(),
                            type,
                            params.find("eye").asString().c_str(),
                            (unsigned int)params.find("swin").asInt(),
                            (unsigned int)params.find("twin").asInt(),
                            params.find("evalsim").asDouble(),
                            params.find("evecsim").asDouble(),
                            (unsigned int)params.find("dim").asInt());
    extractor.useCallback();
    extractor.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    extractor.close();
    extractor.disableCallback();
}
