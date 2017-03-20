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

/// \defgroup Library Library
/// \defgroup vSurface vSurface
/// \ingroup Library
/// \brief
/// A data representation which stores only the most recent event at each pixel
/// location

#ifndef __VWINDOW_BASIC__
#define __VWINDOW_BASIC__

#include <vector>
#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

/**
 * @brief The vWindow class holds a list of events for a period of time as
 * specified. Event expiry is checked each time new events are added and
 * expired events are removed. At any point in time a copy of the current list
 * of events can be requested.
 */
class vSurface
{

protected:

    //Local Memory
    //! for quick spatial accessing and surfacing
    std::vector< std::vector < event<AE> > > spatial;

    //Parameters
    resolution res;

    //Local Variables
    event<AE> mostRecent;
    int eventCount;

public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vSurface(int width = 128, int height = 128);

    vSurface(const vSurface &);
    vSurface operator=(const vSurface&);
    virtual ~vSurface() {}

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    event<AE> addEvent(event<AE> v);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    event<AE> getMostRecent();

    int getEventCount() { return eventCount; }
    void clear();

    const vQueue getSurf(int d);
    const vQueue getSurf(int x, int y, int d);
    virtual const vQueue getSurf(int xl, int xh, int yl, int yh);


};

/******************************************************************************/

class vTempWindow {

protected:

    //! event storage
    vQueue q;
    //!precalculated thresholds
    int tUpper;
    int tLower;

public:

    vTempWindow();
    void addEvent(event<> v);
    void addEvents(const vQueue &events);
    vQueue getWindow();
};

}

#endif
