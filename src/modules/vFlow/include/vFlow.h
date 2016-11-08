/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

/// \defgroup Modules Modules
/// \defgroup vFlow vFlow
/// \ingroup Modules
/// \brief calculates optical flow using plane fitting

#ifndef __VFLOW__
#define __VFLOW__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/eventdriven/all.h>

///
/// \brief The vFlowManager class performs event-based optical flow
///
class vFlowManager : public yarp::os::BufferedPort<eventdriven::vBottle>
{
private:

    bool strictness;
    yarp::os::BufferedPort<eventdriven::vBottle> outPort;

    eventdriven::vSurface2 *surfaceOnL;
    eventdriven::vSurface2 *surfaceOfL;
    eventdriven::vSurface2 *surfaceOnR; ///< eventdriven::vSurface for on polarity events
    eventdriven::vSurface2 *surfaceOfR; ///< eventdriven::vSurface for off polarity events
    eventdriven::vSurface2 *cSurf;     //! pointer to current surface (on or off)

    int height; //! sensor height
    int width;  //! sensor width
    int fRad;   //! filter radius
    unsigned int planeSize; //! edge length of fitted plane
    int halfCount; //! plane area divided by 2
    int minEvtsOnPlane; //! minimum number of events for plane validity

    yarp::sig::Matrix At;
    yarp::sig::Matrix AtA;
    yarp::sig::Matrix A2;
    yarp::sig::Vector abc;

    int eventsComputed;
    int eventsPotential;
    int bottleCount;

    eventdriven::FlowEvent *compute();
    int computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
                      double cx, double cy, double cz,
                      double &dtdy, double &dtdx);
    int computeGrads(const eventdriven::vQueue &subsurf, eventdriven::AddressEvent &cen,
                      double &dtdy, double &dtdx);

public:

    vFlowManager(int height, int width, int filterSize, int minEvtsOnPlane);

    ///
    /// \brief open the port and start callback
    /// \param moduleName defines port names (default vFlow)
    /// \param strictness read and write strcitly such that events are not lost
    /// \return true on successful open
    ///
    bool    open(std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();
    void    onRead(eventdriven::vBottle &inBottle);

};


class vFlowModule:public yarp::os::RFModule {


    vFlowManager *flowmanager;

public:

    virtual bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    virtual bool interruptModule();                       // interrupt, e.g., the ports
    virtual bool close();                                 // close and shut down the module
    virtual bool updateModule();
    virtual double getPeriod();
};
#endif //__VFLOW__

