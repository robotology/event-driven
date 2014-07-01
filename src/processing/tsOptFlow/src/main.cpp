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
#include "tsOptFlow.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include "VelocityBuffer.h"

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
    if(!params.check("height") || !params.check("width") || !params.check("source") || !params.check("name"))
    {
        fprintf(stderr, "Error param\n");
        return -1;
    }
    string source=params.find("source").asString().c_str();
    unsigned int type=0;
    if(!source.compare("dvs"))
    {
        if(!params.check("type"))
        {
        fprintf(stderr, "Err: source set as \"dvs\", please set the type [4, 6, 8]");
        return -1;
        }
        else
            type=(unsigned int)params.find("type").asInt();
    }

    unsigned int height=(unsigned int)params.find("height").asInt();
    unsigned int width=(unsigned int)params.find("width").asInt();
    unsigned int acc=20000;
    if(params.check("acc"))
        acc=(unsigned int)params.find("acc").asInt();
    unsigned int binAcc=1000;
    if(params.check("bin"))
        binAcc=(unsigned int)params.find("bin").asInt();
    double threshold=2;
    if(params.check("threshold"))
        threshold=params.find("threshold").asDouble();
    unsigned int nNeighBor=5;
    if(params.check("nNeighBor"))
        nNeighBor=(unsigned int)params.find("nNeighBor").asInt();
    unsigned int szSobel=3;
    if(params.check("szSobel"))
        szSobel=(unsigned int)params.find("szSobel").asInt();
    unsigned int tsValidity=100000;
    if(params.check("tsValidity"))
        tsValidity=(unsigned int)params.find("tsValidity").asInt();
    double alpha=0.5;
    if(params.check("alpha"))
        alpha=params.find("alpha").asDouble();
    if(alpha>1) alpha=1;
    if(alpha<0) alpha=0;
    double tauD=25;
    if(params.check("tauD"))
        tauD=params.find("tauD").asDouble();
    int pol=0;
    if(params.check("pol"))
        pol=params.find("pol").asInt();
    std::string eye="left";
    if(params.check("eye"))
        eye=params.find("eye").asString();
 
    bool ori=params.check("swap_xy");
    bool save=params.check("save");

    string appName=params.find("name").asString().c_str();
    string localPort="/";
    localPort+=appName;
    localPort+="/evts:i";

    yarp::os::BufferedPort<VelocityBuffer> outPort;
    string outPortName="/";
    outPortName+=appName;
    outPortName+="/flow:o";
    outPort.open(outPortName.c_str());

    tsOptFlow tsOptFlower(  height,
                            width,
                            source,
                            type,
                            acc,
                            binAcc,
                            threshold,
                            nNeighBor,
                            szSobel,
                            tsValidity,
                            alpha,
                            tauD,
                            pol,
                            eye,
                            ori,
                            save,
                            &outPort);

    tsOptFlower.useCallback();
    tsOptFlower.open(localPort.c_str());

    string in;
    bool end = false;
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
    tsOptFlower.close();
    tsOptFlower.disableCallback();

    outPort.close();
}
