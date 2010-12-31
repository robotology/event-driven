#include "opticalFlowViewer.h"
#include <cstring>

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;
	std::string in;

	opticalFlowViewer optFviewer;
	optFviewer.useCallback();
	optFviewer.open("/image/opticalFlowViewer:i");
//	Network::connect("/image/opticalFlow:o","/image/opticalFlowViewer:i");
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
	optFviewer.close();
    optFviewer.disableCallback();
    return EXIT_SUCCESS;
}
