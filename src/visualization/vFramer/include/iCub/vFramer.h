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

/// \defgroup visualisation Visualisation
/// \defgroup vFramer vFramer
/// \ingroup visualisation
/// \brief frame events in a temporal window which are drawn with a
/// emorph::vDraw

#ifndef __vFramer__
#define __vFramer__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>
#include <map>

#include "vDraw.h"

namespace emorph {


/**
 * @brief The vReadAndSplit class splits events into different vWindows based
 * on the channel parameter. At any point in time a snapshot of all windows can
 * be performed, after which the resulting vQueues can be accessed.
 */
class vReadAndSplit : public yarp::os::BufferedPort<emorph::vBottle>
{
    //has an onRead() function that updates an eImage based on the draw
    //functions and then outputs the image at a certain rate

private:

    //! storage of vWindows
    std::map<int, vTempWindow> windows;
    //! storage of window snapshots
    std::map<int, vQueue> snaps;

    yarp::os::Stamp yarptime;

    yarp::os::Mutex safety;

public:

    ///
    /// \brief vReadAndSplit constructor with default windowsize parameter
    ///
    //vReadAndSplit() {}

    yarp::os::Stamp getYarpTime() { return yarptime; }

    ///
    /// \brief snapshotAllWindows freeze the current list of events for each
    /// channel
    ///
    void snapshotAllWindows();

    ///
    /// \brief getSnap get a list of snapshotted events
    /// \param channel the channel value to access
    /// \return the list of events in a vQueue
    ///
    const emorph::vQueue & getSnap(const int channel);

    ///
    /// \brief onRead splitting is performed as an asynchronous onRead function
    /// \param incoming the vBottle with events of all channels
    ///
    virtual void onRead(emorph::vBottle &incoming);
    virtual bool open(const std::string portName, bool strict = false);
};

/**
 * @brief The vFramerModule class runs the event reading and channel splitting,
 * the drawing modules, and the yarp image output buffers. Images are created
 * at a rate equal to the rate of this thread.
 */
class vFramerModule : public yarp::os::RFModule {

private:

    //! the period between images being published
    double period;

    double pyarptime;


    //! the vBottle reading port that splits events by channel
    vReadAndSplit vReader;

    //! the list of channels for each output image
    std::vector<int> channels;

    //! the list of drawers for each output image
    std::vector<std::vector<vDraw *> > drawers;

    //! the list of output ports for images
    std::vector<yarp::os::BufferedPort<
        yarp::sig::ImageOf<yarp::sig::PixelBgr> > *> outports;

public:

    virtual ~vFramerModule();

    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the modulereturn

    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
};


} //namespace emorph

#endif //vFramer

