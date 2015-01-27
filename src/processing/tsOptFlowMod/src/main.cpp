/**
 * @file main.cpp
 * @brief main code for the computation of the optical flow
 */

#include "tsOptFlow.hpp"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network::init();

    /* instantiate the module */
    tsOptFlow module;

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("tsOptFlow.ini");    //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");    //overridden by --context parameter
    rf.configure(argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    /* deinitilize yarp network */
    Network::fini();

    return 0;
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
