/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VFLOW__
#define __VFLOW__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/eventdriven/all.h>

class vFlowManager : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //parameters
    int fRad;               //! radius of the fitted plane
    unsigned int planeSize; //! area of the fitted plane
    int minEvtsOnPlane;     //! minimum number of events for a valid plane
    bool strictness;        //! don't lose events!

    //ports
    yarp::os::BufferedPort<ev::vBottle> outPort;

    //data structures
    ev::vSurface2 *surfaceOnL;
    ev::vSurface2 *surfaceOfL;
    ev::vSurface2 *surfaceOnR;
    ev::vSurface2 *surfaceOfR;

    yarp::sig::Matrix At;
    yarp::sig::Matrix AtA;
    yarp::sig::Matrix A2;
    yarp::sig::Vector abc;

    //coputation functions
    bool compute(ev::vSurface2 *surf, double &vx, double &vy);
    int computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
                      double cx, double cy, double cz,
                      double &dtdy, double &dtdx);
    int computeGrads(const ev::vQueue &subsurf, ev::event<ev::AddressEvent> cen,
                      double &dtdy, double &dtdx);

public:

    vFlowManager(int height, int width, int filterSize, int minEvtsOnPlane);

    bool    open(std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();
    void    onRead(ev::vBottle &inBottle);

};

class vFlowModule:public yarp::os::RFModule {

    vFlowManager *flowmanager;

public:

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual bool updateModule();
    virtual double getPeriod();
};

#endif //__VFLOW__

