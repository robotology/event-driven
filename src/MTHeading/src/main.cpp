#include <iostream>

#include <yarp/os/Thread.h>


#include <iCub/HeadingModule.h>


int main(int argc, char *argv[])
{

   yarp::os::Network yarp;

    HeadingModule headingModule;

    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("eventBasedOpticalFlow.ini"); //overriden by --from parameter
    //rf.setDefaultContext("eventOpticalFlow/conf"); //overriden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    if (!headingModule.configure(rf)){
        cerr << "Error in Configuring Heading Module, returning" << endl;
        return -1;
    }

    headingModule.runModule();
    cout << "Heading Module shutting down." << endl;

   return 0;
}
