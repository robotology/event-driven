/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include "tsOptFlowViewer.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

using namespace std;
int main(int argc, char *argv[])
{
    yarp::os::Network yarpNet;

    yarp::os::Property params;
    params.fromCommand(argc, argv);
    if(!params.check("name"))
    {
        fprintf(stderr, "Error param\n");
        return -1;
    }
    double norm=30;
    if(params.check("norm"))
        norm=params.find("norm").asDouble();
    string display="arrow";
    if(params.check("display"))
        display=params.find("display").asString().c_str();

    string robotName=params.find("name").asString().c_str();
    string localPort="/";
    localPort+=robotName;
    localPort+="/flow:i";

    string outPort="/";
    outPort+=robotName;
    outPort+="/img:o";

    tsOptFlowViewer tsViewer(display, norm);

    //BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > portOut;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb> > portOut;
    portOut.open(outPort.c_str());
    tsViewer.setOutPort(&portOut);

    tsViewer.useCallback();
    tsViewer.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    tsViewer.close();
    tsViewer.disableCallback();

    portOut.close();
}
