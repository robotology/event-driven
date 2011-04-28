#include "opticalFlowViewer.h"
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;
	
    bool color=false;

    if (argc==2)
    {
        if (std::string(argv[1])=="--color")
        {
            color=true;
        }
    }

	opticalFlowViewer optFviewer(color);
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
