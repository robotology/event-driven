/*
 * Copyright (C) 2010 eMorph Group iCub Facility
 * Authors: Arren Glover
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

#include "depthGT.h"


/*////////////////////////////////////////////////////////////////////////////*/
//depthgt
/*////////////////////////////////////////////////////////////////////////////*/

bool depthgt::configure(yarp::os::ResourceFinder &rf)
{

    setName(rf.check("name", yarp::os::Value("depthgt")).asString().c_str());

    roiSize = rf.check("roisize", yarp::os::Value(80)).asInt();
    roiY = rf.check("roiy", yarp::os::Value(120)).asInt();
    roiX = rf.check("roix", yarp::os::Value(220)).asInt();
    offset = rf.check("offset", yarp::os::Value(-550)).asInt();

    if(!depthImIn.open("/"+getName()+"/depthim:i")) return false;
    if(!depthImOut.open("/"+getName()+"/depthim:o")) return false;
    if(!depthOut.open("/"+getName()+"/gt:o")) return false;

    if(!yarp::os::Network::connect("/OpenNI2/depthFrame:o", depthImIn.getName()));

    return true;

}

bool depthgt::interruptModule()
{
    depthImIn.interrupt();
    depthImOut.interrupt();
    depthOut.interrupt();
    return true;
}

bool depthgt::close()
{
    depthImIn.close();
    depthImOut.close();
    depthOut.close();

    return true;
}

bool depthgt::updateModule()
{

    if(isStopping()) return false;

    yarp::sig::ImageOf<yarp::sig::PixelMono16> *imin = depthImIn.read();

    yarp::os::NetUint16 closest = 65535;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> &roiim = depthImOut.prepare();
    roiim.resize(roiSize, roiSize);
    for(int y = 0; y < roiSize; y++) {
        for(int x = 0; x < roiSize; x++) {
            roiim(x, y) = (*imin)(roiX + x - roiSize / 2, roiY + y - roiSize / 2);
            if(roiim(x, y) > 480 && roiim(x, y) < 1480) {
                closest = std::min(closest, roiim(x, y));
            }
        }
    }

    if(depthImOut.getOutputCount())
        depthImOut.write();


    std::cout << closest << std::endl;
    if(closest != 65535) {
        yarp::os::Bottle &gtval = depthOut.prepare();
        gtval.clear(); gtval.addInt(closest + offset);
        depthOut.write();
    }

    return true;

}

double depthgt::getPeriod()
{
    return 0.1;
}
