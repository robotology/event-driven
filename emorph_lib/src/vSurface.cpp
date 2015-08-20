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

vSurface::vSurface(int width, int height, bool asynch)
{
    this->width = width;
    this->height = height;
    this->asynchronous = asynch;
    this->mostRecent = 0;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    subq = vQueue(asynchronous);

}

vSurface::vSurface(const vSurface& that)
{
    *this = that;
}

vSurface vSurface::operator=(const vSurface& that)
{
    this->width = that.width;
    this->height = that.height;
    this->asynchronous = that.asynchronous;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            //if(that.spatial[i][j]) spatial[i][j] = that.spatial[i][j]->clone();
        }
    }

    //we should also copy the data!!

    subq = vQueue(asynchronous);

    return *this;
}

void vSurface::addEvent(AddressEvent &event)
{

    //enter critcal section
    mutex.wait();

    //if we previously had an event, delete it
    int x = event.getX(); int y = event.getY();
    if(spatial[y][x]) delete spatial[y][x];
    //put our new event in
    spatial[y][x] = event.clone();
    mostRecent = spatial[y][x];

    //leave section
    mutex.post();
}

const vQueue& vSurface::getSURF(int d)
{
    return getSURF(mostRecent->getAs<AddressEvent>()->getX(),
                   mostRecent->getAs<AddressEvent>()->getY(), d);
}

const vQueue& vSurface::getSURF(int x, int y, int d)
{
    return getSURF(x - d, x + d, y - d, y + d);
}

const vQueue& vSurface::getSURF(int xl, int xh, int yl, int yh)
{
    subq.clear();

    xl = std::max(xl, 0);
    xh = std::min(xh, width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, height-1);

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
    if(asynchronous) return mostRecent->clone();
    else return mostRecent;
}



}
