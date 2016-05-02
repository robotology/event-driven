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

#ifndef __VCIRCLE__
#define __VCIRCLE__

/// \defgroup processing Processing
/// \defgroup vCircle vCircle
/// \ingroup processing
/// \brief track circular shapes in event stream
///
/// \section parameters Parameters
///
/// - \c name \c vCircle \n
/// set the root string for port naming e.g. /vCircle/vBottle:i

/**
 *
 * This module uses the Hough transform at several radii to detect circles in
 * the event-driven data stream.
 *
 * \section parameters_sec Parameters
 *
 * - \c name \c vCircle \n
 *   set the root string for port naming e.g. /vCircle/vBottle:i
 *
 * - \c strict \c true | false \n
 *   set port writing strictness of this module
 *
 * - \c houghType \c directed | full \n
 *   set the Hough transform method: full (standard) or directed
 *
 * - \c qType \c Fixed | Lifetime \n
 *   set the vQueue method using a fixed size or using the event-lifetime
 *
 * - \c inlierThreshold \c 25 \n
 *   set the threshold on positive circle detection
 *
 * \section portsa_sec Ports Accessed
 *
 *
 * \section portsc_sec Ports Created
 *
 *  <b> Input ports </b>
 *
 *  - \c /vCircle/vBottle:i \n
 *   input port for vBottles containing AddressEvents or FlowEvents
 *
 * <b> Output ports </b>
 *
 *  - \c /vCircle/vBottle:o \n
 *   appends the input bottle with circle detection events
 *
 *  - \c /vCircle/debug:o \n
 *   a yarp image visualising the hough space (slows down detection rate)
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>aexGrabber --name vCircle --strict true --houghType directed
 * -- qType Fixed --inlierThreshold 25 </tt>
 *
 * \author Arren Glover
 *
 */

#include <fstream>
#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <iCub/ctrl/kalman.h>
#include "vCircleObserver.h"


/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEREADER
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleReader : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > houghOut;

    emorph::vtsHelper unwrap;
    double pTS;

    //our filter/tracker

    bool strictness;
    bool debugFlag;
    std::ofstream datawriter;
    yarp::os::Stamp pstamp;
    int pstampcounter;

public:

    //we actually allow our observers and trackers
    vCircleTracker circleTracker;
    vCircleMultiSize * cObserverL;
    vCircleMultiSize * cObserverR;
    double inlierThreshold;
    bool hough;
    double timecounter;

    vCircleReader();

    bool    open(const std::string &name, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &inBot);

    bool setDataWriter(std::string datafilename);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vCircleReader      circleReader;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
