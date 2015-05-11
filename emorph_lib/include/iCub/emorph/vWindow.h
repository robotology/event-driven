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
#include <yarp/sig/all.h>
#include <vector>
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
    int width;
    int height;
    int duration;
    //! whether to deep copy or shallow copy
    bool asynchronous;
    //! for safe copying of q in the multi-threaded environment
    yarp::os::Semaphore mutex;
    //! for quick spatial accessing and surfacing
    std::vector< std::vector <vQueue> > spatial;


public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vWindow(int width = 128, int height = 128, int duration = 20000, bool asynch = true);

    vWindow operator=(const vWindow&);

    ///
    /// \brief setWindowSize sets the length of time to store events
    /// \param windowSize the time period (in us)
    ///
    void setTemporalWindowSize(int duration)  { this->duration = duration; }

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    void addEvent(emorph::vEvent &event);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    vEvent *getMostRecent();

    ///
    /// \brief getWindow
    /// \return
    ///
    const vQueue getWindow();

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param x x centre
    /// \param y y centre
    /// \param d distance of the half-length of a square window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue getSpatialWindow(int x, int y, int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param xl lower x value of window
    /// \param xh upper x value of window
    /// \param yl lower y value of window
    /// \param yh upper y value of window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue getSpatialWindow(int xl, int xh, int yl, int yh);



};

}

#endif
