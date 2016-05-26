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
/// \defgroup vSurface vSurface
/// \ingroup emorphLib
/// \brief
/// A data representation which stores only the most recent event at each pixel
/// location

#ifndef __VSURFACE__
#define __VSURFACE__

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
class vSurface
{

protected:

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
    vEvent * addEvent(emorph::AddressEvent &event);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    vEvent * getMostRecent();

    int getEventCount() { return eventCount; }
    void clear();

    const vQueue& getSurf(int d);
    const vQueue& getSurf(int x, int y, int d);
    virtual const vQueue& getSurf(int xl, int xh, int yl, int yh);


};

class vEdge : public vSurface
{
private:

    //set the previous add event to be private so it cannot be used in vEdge
    //there could be a better way to achieve this.
    int thickness;
    bool trackCount;
    vEvent * addEvent(emorph::AddressEvent &event);

    bool addressremove(vQueue &removed, AddressEvent * v);
    bool flowremove(vQueue &removed, FlowEvent *vf);
    bool pepperCheck(int y, int x);

public:

    vEdge(int width = 128, int height = 128) :
        vSurface(width, height), thickness(2), trackCount(false) {}

    vQueue addEventToEdge(AddressEvent *event);
    void setThickness(int pixels) {thickness = pixels;}
    void track(bool trackCount = true) {this->trackCount = trackCount;}

    virtual const vQueue& getSurf(int xl, int xh, int yl, int yh);
    using vSurface::getSurf;

};

class vFuzzyEdge : public vEdge
{

private:

    double delta;
    std::vector < std::vector <double> > scores;

public:

    vFuzzyEdge(int width = 128, int height = 128, double delta = 0.4);
    vQueue addEventToEdge(AddressEvent *event);
    virtual const vQueue& getSURF(int xl, int xh, int yl, int yh);

};

}

#endif
