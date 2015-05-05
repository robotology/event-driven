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
#include <vector>

namespace emorph {

/**
 * @brief The vWindow class
 */
class vSurf {

private:

    //! data storage
    std::vector<std::vector<vEvent *> > surface;

    //! \brief mostRecent
    AddressEvent * mostRecent;

    //! for safe copying of q in the multi-threaded environment
    yarp::os::Semaphore mutex;

public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vSurf();

    ~vSurf();

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    void addEvent(emorph::AddressEvent &event);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    AddressEvent *getMostRecent();

    ///
    /// \brief getSpatialWindow
    /// \return
    ///
    vQueue& getSpatialWindow();

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param x x centre
    /// \param y y centre
    /// \param d distance of the half-length of a square window
    /// \return a vQueue containing a copy of the events
    ///
    vQueue& getSpatialWindow(int x, int y, int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param xl lower x value of window
    /// \param xh upper x value of window
    /// \param yl lower y value of window
    /// \param yh upper y value of window
    /// \return a vQueue containing a copy of the events
    ///
    vQueue& getSpatialWindow(int xl, int xh, int yl, int yh);


};

}

#endif
