#include "opticalFlowViewer.h"
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;
	
	opticalFlowViewer optFviewer;
	optFviewer.useCallback();
	optFviewer.open("/optflow/vectors:i");
    
    std::string in;
    while(true)
    {
        std::cin >> in;
        if (in=="quit") break;
    }
	
    optFviewer.close();
    optFviewer.disableCallback();
    
    return EXIT_SUCCESS;
}
