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

#ifndef __VWINDOW__
#define __VWINDOW__

#include <yarp/os/all.h>
#include "vCodec.h"
#include "vtsHelper.h"

namespace emorph {

/**
 * @brief The vWindow class holds a list of events for a period of time as
 * specified. Event expiry is checked each time new events are added and
 * expired events are removed. At any point in time a copy of the current list
 * of events can be requested.
 */
class vWindow {

private:

    //! event storage
    vQueue q;
    //! the length of time to store events (in us)
    int windowSize;
    //! for safe copying of q in the multi-threaded environment
    yarp::os::Semaphore mutex;

public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vWindow(int windowSize = 50000) : windowSize(windowSize){q.setOwner(false);}

    ~vWindow() { q.setOwner(true); }

    ///
    /// \brief setWindowSize sets the length of time to store events
    /// \param windowSize the time period (in us)
    ///
    void setWindowSize(int windowSize)  { this->windowSize = windowSize; }

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    void addEvent(emorph::vEvent &event);

    ///
    /// \brief getCurrentWindow returns the current list of active events
    /// \param sample_q is a vQueue to which events are added
    /// \return the number of events added to sample_q
    ///
    int getCurrentWindow(emorph::vQueue &sample_q);

    const vQueue& getWindow();

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param x x centre
    /// \param y y centre
    /// \param d distance of the half-length of a square window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue& getSpatialWindow(int x, int y, int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param xl lower x value of window
    /// \param xh upper x value of window
    /// \param yl lower y value of window
    /// \param yh upper y value of window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue& getSpatialWindow(int xl, int xh, int yl, int yh);

    vEvent *getMostRecent();

};

}

#endif
