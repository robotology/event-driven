#include "opticalFlowViewer.h"
#include <string>
#include <iostream>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;
	
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.configure("ICUB_ROOT",argc,argv);

    double gain=rf.check("gain")?rf.find("gain").asDouble():8.5;

	opticalFlowViewer optFviewer(gain);
	optFviewer.useCallback();
	optFviewer.open("/optflow/vectors:i");
    
    std::string in;
    while(true)
    {
        yarp::os::Time::delay (0.002);
//        std::cin >> in;
  //      if (in=="quit") break;
    }
	
    optFviewer.close();
    optFviewer.disableCallback();
    
    return EXIT_SUCCESS;
}
