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

#include "iCub/eventdriven/vWindow_basic.h"

namespace ev {

vSurface::vSurface(int width, int height)
{
    res.width = width;
    res.height = height;
    mostRecent = event<AE>(nullptr);
    eventCount = 0;

    spatial.resize(res.height);
    for(int y = 0; y < res.height; y++) {
        spatial[y].resize(res.width, 0);
    }

}

vSurface::vSurface(const vSurface& that)
{
    *this = that;
}

vSurface vSurface::operator=(const vSurface& that)
{
    //destroy all current events
    this->clear();

    //remake like 'that'
    this->res.width = that.res.width;
    this->res.height = that.res.height;
    this->eventCount = that.eventCount;
    this->mostRecent = that.mostRecent;

    spatial.resize(res.height);
    for(int y = 0; y < res.height; y++) {
        spatial[y].resize(res.width, 0);
    }

    for(int i = 0; i < res.height; i++) {
        for(int j = 0; j < res.width; j++) {
            spatial[i][j] = that.spatial[i][j];
        }
    }

    return *this;
}

void vSurface::clear()
{
    for(int i = 0; i < res.height; i++) {
        for(int j = 0; j < res.width; j++) {
            spatial[i][j] = event<AE>(nullptr);
        }
    }

    mostRecent = event<AE>(nullptr);
    eventCount = 0;

}

event<AE> vSurface::addEvent(event<AE> v)
{

    event<AE> removed;

    //if we previously had an event move it to the justRemoved
    if(spatial[v->y][v->x]) {
        removed = spatial[v->y][v->x];
        eventCount--;
    }

    //put our new event in and add a reference
    spatial[v->y][v->x] = v;
    eventCount++;

    //then set our mostRecent
    mostRecent = v;

    return removed;
}

const vQueue vSurface::getSurf(int d)
{
    if(!mostRecent) return vQueue();
    return getSurf(mostRecent->x, mostRecent->y, d);
}

const vQueue vSurface::getSurf(int x, int y, int d)
{
    return getSurf(x - d, x + d, y - d, y + d);
}

const vQueue vSurface::getSurf(int xl, int xh, int yl, int yh)
{

    xl = std::max(xl, 0);
    xh = std::min(xh, res.width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, res.height-1);

    vQueue subq;

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(spatial[y][x]) subq.push_back(spatial[y][x]);
        }
    }

    return subq;

}


event<AE> vSurface::getMostRecent()
{
    return mostRecent;
}


/******************************************************************************/

vTempWindow::vTempWindow()
{
    //whichever is smaller: 2 seconds or 1/2 of the max stamp
    tLower = std::min(vtsHelper::max_stamp * 0.45, vtsHelper::vtsscaler * 2.0);
    tUpper = vtsHelper::max_stamp - tLower;
}

void vTempWindow::addEvent(event<> v)
{
    int ctime = v->stamp;
    int upper = ctime + tUpper;
    int lower = ctime - tLower;

//    while(q.size()) {
//        int vtime = q.back()->stamp;
//        if((vtime >= ctime && vtime < upper) || vtime < lower) {
//            q.pop_back();
//        } else {
//            break;
//        }
//    }

    while(q.size()) {

        int vtime = q.front()->stamp;
        if((vtime >= ctime && vtime < upper) || vtime < lower) {
            q.pop_front();
        } else {
            break;
        }
    }

    q.push_back(v);
}

void vTempWindow::addEvents(const vQueue &events)
{
    vQueue::const_iterator qi;
    for(qi = events.begin(); qi != events.end() - 1; qi++)
        q.push_back(*qi);

    addEvent(events.back());
}

vQueue vTempWindow::getWindow()
{
    return q;
}


}
