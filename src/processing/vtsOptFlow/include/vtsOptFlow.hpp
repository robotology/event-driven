/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author: Charles Clercq, edited by Valentina Vasco (01/15)
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

/*
 * @file tsOptFlow.h
 * @brief A module that reads vBottle from a yarp port and computes optical flow
 */

#ifndef VTSOPTFLOW_HPP
#define VTSOPTFLOW_HPP

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/emorph/all.h>

class vtsOptFlowManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    yarp::os::BufferedPort<emorph::vBottle> outPort;

    emorph::vSurface *surface;

    int height;
    int width;
    int fRad;
    int halfCount;
    int minEvtsOnPlane;

    yarp::sig::Matrix At;
    yarp::sig::Matrix AtA;
    yarp::sig::Matrix A2;
    yarp::sig::Vector abc;

    int eventsComputed;
    int eventsPotential;
    int bottleCount;

    emorph::FlowEvent compute();
    int computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
                      double cx, double cy, double cz,
                      double &dtdy, double &dtdx);
    int computeGrads(emorph::vQueue &subsurf, emorph::AddressEvent &cen,
                      double &dtdy, double &dtdx);

public:

    vtsOptFlowManager(int height, int width, int filterSize,
                      int minEvtsOnPlane);

    bool    open(std::string moduleName);
    void    close();
    void    interrupt();
    void    onRead(emorph::vBottle &inBottle);

};


class vtsOptFlow:public yarp::os::RFModule {


    vtsOptFlowManager *vtsofManager;

public:

    virtual bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    virtual bool interruptModule();                       // interrupt, e.g., the ports
    virtual bool close();                                 // close and shut down the module
    virtual bool updateModule();
    virtual double getPeriod();
};
#endif //VTSOPTFLOW_HPP

//----- end-of-file --- ( next line intentionally left blank ) ------------------
