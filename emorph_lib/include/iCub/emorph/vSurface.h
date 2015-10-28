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

#ifndef __VSURFACE__
#define __VSURFACE__

//#include <yarp/os/all.h>
#include <vector>
#include <iCub/emorph/vCodec.h>
#include <iCub/emorph/vQueue.h>
#include <iCub/emorph/vtsHelper.h>

namespace emorph {

/**
 * @brief The vWindow class holds a list of events for a period of time as
 * specified. Event expiry is checked each time new events are added and
 * expired events are removed. At any point in time a copy of the current list
 * of events can be requested.
 */
class vSurface {

private:

    //Local Memory
    //! for quick spatial accessing and surfacing
    std::vector< std::vector <vEvent *> > spatial;

    //Parameters
    //! the sensor width
    int width;
    //! the sensor height
    int height;

    //Local Variables
    vEvent * mostRecent;
    vEvent * justRemoved;
    //! for safe copying of q in the multi-threaded environment
    yarp::os::Semaphore mutex;
    //! member variable for quick memory allocation
    vQueue subq;

public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vSurface(int width = 128, int height = 128);

    vSurface(const vSurface &);
    vSurface operator=(const vSurface&);

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    vEvent * addEvent(emorph::AddressEvent &event);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    vEvent *getMostRecent();


    void clear();

    const vQueue& getSURF(int d);
    const vQueue& getSURF(int x, int y, int d);
    const vQueue& getSURF(int xl, int xh, int yl, int yh);


};

}

#endif
