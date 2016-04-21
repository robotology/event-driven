#include "iCub/zynqGrabberModule.h"


int main(int argc, char * argv[])
{
    yarp::os::Network::init();

    zynqGrabberModule module;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("zynqGrabber.ini"); //overridden by --from parameter
    rf.setDefaultContext("eMorph");   //overridden by --context parameter
    rf.configure(argc, argv);

    module.runModule(rf);

    yarp::os::Network::fini();

    return 0;
}


