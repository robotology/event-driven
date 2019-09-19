/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VWINDOW_ADV__
#define __VWINDOW_ADV__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <vector>
#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"
#include "iCub/eventdriven/vWindow_basic.h"

namespace ev {

/// \brief a spatial-temporal surface storage data structure
class vSurface2 {

protected:

    //! event storage
    vQueue q;

    //! for quick spatial accessing and surfacing
    std::vector< std::vector < event<> > > spatial;

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
    virtual vQueue addEvent(event<> v);
    void fastAddEvent(event <> v, bool onlyAdd = false);

    virtual vQueue removeEvents(event<> toAdd) = 0;
    virtual void fastRemoveEvents(event<> toAdd) = 0;

    ///
    /// \brief getMostRecent
    /// \return
    ///
    event<> getMostRecent();

    int getEventCount() { return count; }

    vQueue getEverything() { return q; }

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

    vQueue getSurf_Tlim(int dt);
    vQueue getSurf_Tlim(int dt, int d);
    vQueue getSurf_Tlim(int dt, int x, int y, int d);
    vQueue getSurf_Tlim(int dt, int xl, int xh, int yl, int yh);

    vQueue getSurf_Clim(int c);
    vQueue getSurf_Clim(int c, int d);
    vQueue getSurf_Clim(int c, int x, int y, int d);
    vQueue getSurf_Clim(int c, int xl, int xh, int yl, int yh);

    void getSurfSorted(vQueue &fillq);

};
/******************************************************************************/

/// \brief a spatio-temporal surface storing events for a limited time
class temporalSurface : public vSurface2
{
private:

    int duration;

public:

    temporalSurface(int width = 128, int height = 128, int duration = 2.0 * vtsHelper::vtsscaler);
    virtual vQueue removeEvents(event<> toAdd);
    virtual void fastRemoveEvents(event<> toAdd);

    void setTemporalSize(int duration) {this->duration = duration;}

};

/******************************************************************************/

/// \brief a spatio-temporal surface storing only a fixed number of events
class fixedSurface : public vSurface2
{
private:

    int qlength;

public:

    fixedSurface(int qlength = 2000, int width = 128, int height = 128)  :
        vSurface2(width, height), qlength(qlength) {}
    virtual vQueue removeEvents(event<> toAdd);
    virtual void fastRemoveEvents(event<> toAdd);

    void setFixedWindowSize(int length) {this->qlength = length;}
};

/******************************************************************************/

/// \brief a spatio-temporal surface storing events for a "lifetime" given by
/// the inverse of velocity
class lifetimeSurface : public vSurface2
{

public:

    lifetimeSurface(int width = 128, int height = 128) :
        vSurface2(width, height) {}
    virtual vQueue addEvent(event<> toAdd);
    virtual vQueue removeEvents(event<> toAdd);
    virtual void fastRemoveEvents(event<> toAdd);
};

/******************************************************************************/

/// \brief a spatio-temporal surface storing events along edges as given by
/// plane fitting
class vEdge : public vSurface
{
private:

    //set the previous add event to be private so it cannot be used in vEdge
    //there could be a better way to achieve this.
    int thickness;
    bool trackCount;
    event<> addEvent(event<AddressEvent> v);

    bool addressremove(vQueue &removed, event<AddressEvent> v);
    bool flowremove(vQueue &removed, event<FlowEvent> vf);
    bool pepperCheck(int y, int x);

public:

    vEdge(int width = 128, int height = 128) :
        vSurface(width, height), thickness(2), trackCount(false) {}

    vQueue addEventToEdge(event<AddressEvent> v);
    void setThickness(int pixels) {thickness = pixels;}
    void track(bool trackCount = true) {this->trackCount = trackCount;}

    virtual const vQueue getSurf(int xl, int xh, int yl, int yh);
    using vSurface::getSurf;

};

/// \brief a vEdge structure that keeps events given a "fuzzy" scoring system
class vFuzzyEdge : public vEdge
{

private:

    double delta;
    std::vector < std::vector <double> > scores;

public:

    vFuzzyEdge(int width = 128, int height = 128, double delta = 0.4);
    vQueue addEventToEdge(event<AddressEvent> event);
    virtual const vQueue getSURF(int xl, int xh, int yl, int yh);

};

/// \brief a surface that can be queried at any time in the past.
class historicalSurface : public vTempWindow
{
private:

    yarp::sig::ImageOf<yarp::sig::PixelMono> surface;

public:

    historicalSurface() {}

    void initialise(int height, int width);

    vQueue getSurface(int queryTime, int queryWindow);
    vQueue getSurface(int queryTime, int queryWindow, int d);
    vQueue getSurface(int queryTime, int queryWindow, int d, int x, int y);
    vQueue getSurface(int queryTime, int queryWindow, int xl, int xh, int yl, int yh);
    void getSurfaceN(ev::vQueue &qret, int queryTime, int numEvents, int d);
    void getSurfaceN(ev::vQueue &qret, int queryTime, int numEvents, int d, int x, int y);
    void getSurfaceN(ev::vQueue &qret, int queryTime, int numEvents, int xl, int xh, int yl, int yh);

};

}



#endif
