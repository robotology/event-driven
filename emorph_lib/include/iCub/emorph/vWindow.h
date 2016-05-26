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

/// \defgroup emorphLib emorphLib
/// \defgroup vWindow vWindow
/// \ingroup emorphLib
/// \brief A storage class which automatically discards events after a given timeperiod

#ifndef __VWINDOW__
#define __VWINDOW__

#include <yarp/os/all.h>
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
class vSurface2 {

protected:

    //! event storage
    vQueue q;

    //! for quick spatial accessing and surfacing
    std::vector< std::vector <vEvent *> > spatial;

    //!retina size
    int width;
    int height;

    //! active events
    int count;

public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vSurface2(int width = 128, int height = 128);
    virtual ~vSurface2() {}

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    virtual vQueue addEvent(emorph::vEvent &event);

    virtual vQueue removeEvents(vEvent &toAdd) = 0;

    ///
    /// \brief getMostRecent
    /// \return
    ///
    vEvent *getMostRecent();

    int getEventCount() { return count; }

    ///
    /// \brief getWindow
    /// \return
    ///
    vQueue getSurf();



    vQueue getSurf(int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param x x centre
    /// \param y y centre
    /// \param d distance of the half-length of a square window
    /// \return a vQueue containing a copy of the events
    ///
    vQueue getSurf(int x, int y, int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param xl lower x value of window
    /// \param xh upper x value of window
    /// \param yl lower y value of window
    /// \param yh upper y value of window
    /// \return a vQueue containing a copy of the events
    ///
    vQueue getSurf(int xl, int xh, int yl, int yh);

};
/******************************************************************************/
class temporalSurface : public vSurface2
{
private:

    int duration;

public:

    temporalSurface(int width = 128, int height = 128,
                   int duration = vtsHelper::maxStamp() * 0.5) :
        vSurface2(width, height), duration(duration) {}
    virtual vQueue removeEvents(vEvent &toAdd);

    void setTemporalSize(int duration) {this->duration = duration;}

};

/******************************************************************************/
class fixedSurface : public vSurface2
{
private:

    int qlength;

public:

    fixedSurface(int qlength = 2000, int width = 128, int height = 128)  :
        vSurface2(width, height), qlength(qlength) {}
    virtual vQueue removeEvents(vEvent &toAdd);

    void setFixedWindowSize(int length) {this->qlength = length;}
};

/******************************************************************************/
class lifetimeSurface : public vSurface2
{

public:

    lifetimeSurface(int width = 128, int height = 128) :
        vSurface2(width, height) {}
    virtual vQueue addEvent(emorph::vEvent &event);
    virtual vQueue removeEvents(vEvent &toAdd);
};

/******************************************************************************/
class vTempWindow {

protected:

    //! event storage
    vQueue q;
    //! camera resolution
    int width;
    int height;
    //!precalculated thresholds
    int tUpper;
    int tLower;

public:

    vTempWindow(int width = 128, int height = 128);
    void addEvent(emorph::vEvent &event);
    void addEvents(const vQueue &events);
    vQueue getWindow();
};

}



#endif
