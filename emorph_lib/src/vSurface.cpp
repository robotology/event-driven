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

vSurface::vSurface(int width, int height, int coverage, bool asynch)
{
    this->width = width;
    this->height = height;
    if(coverage > 90) {
        std::cout << "Coverage set to maximum (90%)" << std::endl;
        coverage = 90;
    } else if(coverage < 1) {
        std::cout << "Coverage set to minimum (1%)" << std::endl;
        coverage = 1;
    }
    this->countLimit = (0.01 * coverage) * width * height;
    this->asynchronous = asynch;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    subq = vQueue(asynchronous);

    count = 0;
}

vSurface::vSurface(const vSurface& that)
{
    *this = that;
}

vSurface vSurface::operator=(const vSurface& that)
{
    this->width = that.width;
    this->height = that.height;
    this->countLimit = that.countLimit;
    this->asynchronous = that.asynchronous;

    //we don't copy data in a vWindow for now
    q.clear();

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    subq = vQueue(asynchronous);

    return *this;
}

void vSurface::addEvent(AddressEvent &event)
{

    //enter critcal section
    mutex.wait();

    //add the event to the storage
    q.push_back(&event);
    //if we don't already have a spatial event we are going to add one
    if(!spatial[event.getY()][event.getX()]) count++;
    //else we just replace it
    spatial[event.getY()][event.getX()] = q.back();

    //remove any events falling out the back of the window
    while(count > countLimit) {

        AddressEvent * f = q.front()->getAs<AddressEvent>();
        if(f == spatial[f->getY()][f->getX()]) {
            //this is the current surface event we need to remove
            spatial[f->getY()][f->getX()] = 0;
            count--;
        }
        //in either case remove the event from the queue
        q.pop_front();
    }

    if(q.size() > countLimit * 1.1) {
        vQueue::iterator qi = q.begin();
        while(qi != q.end()) {
            AddressEvent * f = (*qi)->getAs<AddressEvent>();
            if(*qi == spatial[f->getY()][f->getX()]) qi++;
            else qi = q.erase(qi);
        }
    }

    //leave section
    mutex.post();
}

const vQueue& vSurface::getSURF(int d)
{
    return getSURF(q.back()->getAs<AddressEvent>()->getX(),
                   q.back()->getAs<AddressEvent>()->getY(), d);
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
    if(!q.size()) return 0;
    if(asynchronous) return q.back()->clone();
    else return q.back();
}



}
