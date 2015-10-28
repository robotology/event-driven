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

#include "iCub/emorph/vSurface.h"

namespace emorph {

vSurface::vSurface(int width, int height)
{
    this->width = width;
    this->height = height;
    this->mostRecent = NULL;
    this->justRemoved = NULL;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
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
    this->width = that.width;
    this->height = that.height;
    this->mostRecent = that.mostRecent;
    this->mostRecent->referto();

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(that.spatial[i][j]) spatial[i][j] = that.spatial[i][j];
            spatial[i][j]->referto();
        }
    }

    return *this;
}

void vSurface::clear()
{
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(spatial[i][j]) spatial[i][j]->destroy();
            spatial[i][j] = NULL;
        }
    }

    subq.clear();

    if(mostRecent) {
        mostRecent->destroy();
        mostRecent = NULL;
    }

    if(justRemoved) {
        justRemoved->destroy();
        mostRecent = NULL;
    }

}

vEvent * vSurface::addEvent(AddressEvent &event)
{

    //enter critical section
    mutex.wait();

    //remove the event that was previously destroyed
    if(justRemoved) {
        justRemoved->destroy();
        justRemoved = 0;
    }

    //if we previously had an event move it to the justRemoved
    int x = event.getX(); int y = event.getY();
    if(spatial[y][x]) justRemoved = spatial[y][x];

    //put our new event in and add a reference
    spatial[y][x] = &event;
    event.referto();

    //then set our mostRecent with a second reference
    if(mostRecent) mostRecent->destroy();
    mostRecent = spatial[y][x];
    mostRecent->referto();

    //leave section
    mutex.post();
}

const vQueue& vSurface::getSURF(int d)
{
    return getSURF(mostRecent->getUnsafe<AddressEvent>()->getX(),
                   mostRecent->getUnsafe<AddressEvent>()->getY(), d);
}

const vQueue& vSurface::getSURF(int x, int y, int d)
{
    return getSURF(x - d, x + d, y - d, y + d);
}

const vQueue& vSurface::getSURF(int xl, int xh, int yl, int yh)
{

    xl = std::max(xl, 0);
    xh = std::min(xh, width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, height-1);

    subq.clear();

    //critical section
    mutex.wait();

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(spatial[y][x]) subq.push_back(spatial[y][x]);
        }
    }

    mutex.post();

    return subq;

}


vEvent *vSurface::getMostRecent()
{
    return mostRecent;
}



}
