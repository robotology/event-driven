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
#include <math.h>

namespace emorph {

vSurface::vSurface(int width, int height)
{
    this->width = width;
    this->height = height;
    this->mostRecent = NULL;
    this->justRemoved = NULL;
    eventCount = 0;

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
    this->eventCount = that.eventCount;
    this->mostRecent = that.mostRecent;
    this->justRemoved = that.justRemoved;
    if(this->justRemoved) justRemoved->referto();
    //this->mostRecent->referto();

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            spatial[i][j] = that.spatial[i][j];
            if(that.spatial[i][j]) spatial[i][j]->referto();
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

    mostRecent = NULL;

    if(justRemoved) {
        justRemoved->destroy();
        mostRecent = NULL;
    }

    eventCount = 0;

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
    if(spatial[y][x]) {
        justRemoved = spatial[y][x];
        eventCount--;
    }

    //put our new event in and add a reference
    spatial[y][x] = &event;
    event.referto();
    eventCount++;

    //then set our mostRecent
    mostRecent = spatial[y][x];

    //leave section
    mutex.post();

    return justRemoved;
}

const vQueue& vSurface::getSURF(int d)
{
    if(!mostRecent) return subq;
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


#define COS135on2 0.38268

vQueue vEdge::flowremove(FlowEvent *vf)
{
    vQueue removed;

    int x = vf->getX(); int y = vf->getY();
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x]->destroy();
    }

    double vx = vf->getVy(); double vy = vf->getVx();
    double mag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    vx /= (mag * COS135on2);
    vy /= (mag * COS135on2);
    int dx = 0, dy = 0;
    if(vx > 1) dx = 1; if(vx < -1) dx = -1;
    if(vy > 1) dy = 1; if(vy < -1) dy = -1;

    dx *= -1;
    dy *= -1;

    x += (thickness - 1) * dx;
    y += (thickness - 1) * dy;

    int px, py;

    //corners
    if(dx && dy) {
        px = x + dx * 2; py = y + dy * 2;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = x;      py = y + dy * 2;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx * 2+x; py = 0+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx+x; py = dy+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx*2 + x;      py = dy + y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx + x; py = dy*2 + y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
    }

    //velocity in x
    else if(dx) {
        px = dx * 2+x; py = -1+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx * 2+x; py = 0+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx * 2+x; py = 1+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx+x; py = -1+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx+x; py = 0+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = dx+x; py = 1+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }

    }

    //velocity in y
    else if(dy) {
        px = -1+x; py = dy * 2+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = 0+x; py = dy * 2+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = 1+x; py = dy * 2+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = -1+x; py = dy+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = 0+x; py = dy+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }
        px = 1+x; py = dy+y;
        if(px >= 0 && px < width && py >= 0 && py < height) {
            if(spatial[py][px]) {
                removed.push_back(spatial[py][px]);
                spatial[py][px]->destroy();
                spatial[py][px] = NULL;
            }
        }

    } else {
        std::cerr << "Flow not set correctly" << std::endl;
    }

    return removed;

}

vQueue vEdge::addressremove(AddressEvent * v)
{
    vQueue removed;

    int x = v->getX(); int y = v->getY();
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x]->destroy();
    }

    return removed;

    for(int y = -1 + v->getY(); y <= 1 + v->getY(); y++) {
        for(int x = -1 + v->getX(); x <= 1 + v->getX(); x++) {
            if(x < 0 || y < 0 || x >= width || y >= height) continue;
            if(!spatial[y][x]) continue;
            FlowEvent *vf2 = spatial[y][x]->getAs<FlowEvent>();
            if(!vf2) continue;

            double vx = vf2->getVy(); double vy = vf2->getVx();
            double mag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
            vx /= (mag * COS135on2);
            vy /= (mag * COS135on2);
            int dx = 0, dy = 0;
            if(vx > 1) dx = 1; if(vx < -1) dx = -1;
            if(vy > 1) dy = 1; if(vy < -1) dy = -1;

            if(x + dx == v->getX() && y + dy == v->getY()) {
                removed.push_back(spatial[y][x]);
                spatial[y][x]->destroy();
                spatial[y][x] = NULL;
            }

        }

    }

    return removed;


}



vQueue vEdge::addEventToEdge(AddressEvent *event)
{
    vQueue removed;

    if(!event) return removed;

    //enter critical section
    mutex.wait();

    //remove non edge events
    FlowEvent * vf = event->getAs<FlowEvent>();
    if(!vf) {
        removed = addressremove(event);
    } else {
        removed = flowremove(vf);

    }

    if(trackCount) {
        if(vf) eventCount++;
        for(vQueue::iterator qi = removed.begin(); qi != removed.end(); qi++)
            if((*qi)->getAs<FlowEvent>())
                eventCount--;
    }

//    if(!vf) {
//        mutex.post();
//        return removed;
//    }

    //remove previous event at this pixel
    int x = event->getX(); int y = event->getY();
//    if(spatial[y][x]) {
//        removed.push_back(spatial[y][x]);
//        spatial[y][x]->destroy();
//    }

    //put our new event in and add a reference
    spatial[y][x] = event;
    event->referto();

    //then set our mostRecent
    mostRecent = spatial[y][x];


    //leave section
    mutex.post();

    return removed;

}

const vQueue& vEdge::getSURF(int xl, int xh, int yl, int yh)
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
            if(spatial[y][x] && spatial[y][x]->getAs<FlowEvent>())
                subq.push_back(spatial[y][x]);
        }
    }

    mutex.post();

    return subq;

}



}
