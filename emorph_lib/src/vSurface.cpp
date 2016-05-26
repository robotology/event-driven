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

const vQueue& vSurface::getSurf(int d)
{
    if(!mostRecent) return subq;
    return getSurf(mostRecent->getUnsafe<AddressEvent>()->getX(),
                   mostRecent->getUnsafe<AddressEvent>()->getY(), d);
}

const vQueue& vSurface::getSurf(int x, int y, int d)
{
    return getSurf(x - d, x + d, y - d, y + d);
}

const vQueue& vSurface::getSurf(int xl, int xh, int yl, int yh)
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

bool vEdge::flowremove(vQueue &removed, FlowEvent *vf)
{

    int x = vf->getX(); int y = vf->getY();
    double vx = vf->getVy(); double vy = vf->getVx();
    double vmag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    vx = vx / vmag; vy = vy / vmag;
    int px, py;

    //remove event at this location
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x]->destroy();
        spatial[y][x] = NULL;
    }

    double a = 0.5;
    double t = 0.8;//0.708;
    int f = 3;
    double vx1 = vx + a * vy; double vy1 = vy - a * vx;
    double vx2 = vx - a * vy; double vy2 = vy + a * vx;

    //remove events not on the perpendicular line
    for(int yi = -f; yi <= f; yi++) {
        for(int xi = -f; xi <= f; xi++) {
            double d1 = xi * vx1 + yi * vy1;
            double d2 = xi * vx2 + yi * vy2;
            if(fabs(d1) > t && fabs(d2) > t && d1 * d2 >= 0) {
                px = x + xi; py = y + yi;
                if(px >= 0 && py >= 0 && px < width && py < height) {
                    if(spatial[py][px]) {
                        removed.push_back(spatial[py][px]);
                        spatial[py][px]->destroy();
                        spatial[py][px] = NULL;
                    }
                }
            }
        }
    }

    return true;

}

bool vEdge::addressremove(vQueue &removed, AddressEvent * v)
{

    int x = v->getX(); int y = v->getY();
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x]->destroy();
        spatial[y][x] = NULL;
    }


    for(int yi = -1 + y; yi <= 1 + y; yi++) {
        for(int xi = -1 + x; xi <= 1 + x; xi++) {

            if(yi < 0 || xi < 0 || yi >= height || xi >= width) continue;
            if(spatial[yi][xi] && pepperCheck(yi, xi)) {
                removed.push_back(spatial[yi][xi]);
                spatial[yi][xi]->destroy();
                spatial[yi][xi] = NULL;
            }
        }
    }

    return !pepperCheck(y, x);


}

bool vEdge::pepperCheck(int y, int x)
{
    if(y == 0 || y == height-1 || x == 0 || x == width -1)
        return true;

    if(spatial[y-1][x] && spatial[y-1][x]->getAs<FlowEvent>())
        return false;
    if(spatial[y+1][x] && spatial[y+1][x]->getAs<FlowEvent>())
        return false;
    if(spatial[y][x-1] && spatial[y][x-1]->getAs<FlowEvent>())
        return false;
    if(spatial[y][x+1] && spatial[y][x+1]->getAs<FlowEvent>())
        return false;
    if(spatial[y+1][x+1] && spatial[y+1][x+1]->getAs<FlowEvent>())
        return false;
    if(spatial[y-1][x+1] && spatial[y-1][x+1]->getAs<FlowEvent>())
        return false;
    if(spatial[y+1][x-1] && spatial[y+1][x-1]->getAs<FlowEvent>())
        return false;
    if(spatial[y-1][x-1] && spatial[y-1][x-1]->getAs<FlowEvent>())
        return false;

    return true;
}



vQueue vEdge::addEventToEdge(AddressEvent *event)
{
    vQueue removed;
    bool add = false;

    if(!event) return removed;

    //enter critical section
    mutex.wait();

    //remove non edge events
    FlowEvent * vf = event->getAs<FlowEvent>();
    if(!vf) {
        addressremove(removed, event);
    } else {
        add = flowremove(removed, vf);
    }


    if(trackCount) {
        if(add) eventCount++;
        eventCount -= removed.size();
        //for(vQueue::iterator qi = removed.begin(); qi != removed.end(); qi++)
        //    if((*qi)->getAs<FlowEvent>())
        //        eventCount--;
    }

    if(add) {
        //put our new event in and add a reference
        spatial[event->getY()][event->getX()] = event;
        event->referto();

        //then set our mostRecent
        mostRecent = spatial[event->getY()][event->getX()];
    }





    //leave section
    mutex.post();

    return removed;

}

const vQueue& vEdge::getSurf(int xl, int xh, int yl, int yh)
{
    xl = std::max(xl, 1);
    xh = std::min(xh, width-2);
    yl = std::max(yl, 1);
    yh = std::min(yh, height-2);

    subq.clear();

    //critical section
    mutex.wait();

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(spatial[y][x])
            //if(spatial[y][x] && !pepperCheck(y, x))
            //if(spatial[y][x] && spatial[y][x]->getAs<FlowEvent>())
                subq.push_back(spatial[y][x]);
        }
    }

    mutex.post();

    return subq;

}

vFuzzyEdge::vFuzzyEdge(int width, int height, double delta) :
    vEdge(width, height)
{
    this->delta = delta;
    scores.resize(height);
    for(int y = 0; y < height; y++) {
        scores[y].resize(width, 0);
    }


}
vQueue vFuzzyEdge::addEventToEdge(AddressEvent *event)
{

    if(!event) return vQueue();
    emorph::FlowEvent * vf = event->getAs<emorph::FlowEvent>();
    if(!vf) return vQueue();

    int x = vf->getX(); int y = vf->getY();
    double vx = vf->getVy(); double vy = vf->getVx();
    double vmag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    //vx = vx / vmag; vy = vy / vmag;
    int px, py;

    //remove event at this location

    //double a = 0;
    double t = 0.708;//0.708;
    int f = 3;

    int y1 = -std::min(y, f);
    int y2 = std::min(height - 1 - y, f);
    int x1 = -std::min(x, f);
    int x2 = std::min(width - 1 - x, f);

    //double vx1 = vx + a * vy; double vy1 = vy - a * vx;
    //double vx2 = vx - a * vy; double vy2 = vy + a * vx;

    //remove events not on the perpendicular line
    for(int yi = y1; yi <= y2; yi++) {
        for(int xi = x1; xi <= x2; xi++) {
            double dcenter = sqrt(pow(yi, 2.0) + pow(xi, 2.0));
            if(dcenter > f) continue; //use LUT here

            //if(px < 0 || py < 0 || px > width-1 || py > height-1) continue;

            double d = (xi * vx + yi * vy) / vmag;
            double p = 0.4 * pow(2.718281, -pow(d, 2.0) / t) - 0.2;
            //p *= (f - dcenter/2) / f;
            px = x + xi; py = y + yi;
//            if(scores[py][px] < 0.5) {
//                scores[py][px] += p;
//                scores[py][px] = std::max(scores[py][px], 0.0);
//                if(scores[py][px] > 0.5) scores[py][px] = 1.0;
//            } else {
//                scores[py][px] += p;
//                scores[py][px] = std::min(scores[py][px], 1.0);
//                if(scores[py][px] < 0.5) scores[py][px] = 0.0;
//            }

            scores[py][px] += p;
            scores[py][px] = std::min(scores[py][px], 1.0);
            scores[py][px] = std::max(scores[py][px], 0.0);

//            double d1 = xi * vx1 + yi * vy1;
//            double d2 = xi * vx2 + yi * vy2;
//            if(fabs(d1) > t && fabs(d2) > t && d1 * d2 >= 0) {
//                scores[py][px] = std::max(scores[py][px] - 0.2, 0.0);
//            } else {
//                scores[py][px] = std::min(scores[py][px] + 0.2, 1.0);
//            }
        }
    }

    return vQueue();

}

const vQueue& vFuzzyEdge::getSURF(int xl, int xh, int yl, int yh)
{
    xl = std::max(xl, 1);
    xh = std::min(xh, width-2);
    yl = std::max(yl, 1);
    yh = std::min(yh, height-2);

    subq.clear();

    //critical section
    mutex.wait();

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(scores[y][x] > 0.5) {
                emorph::AddressEvent * ae = new emorph::AddressEvent();
                ae->setX(x); ae->setY(y); ae->setChannel(0); ae->setPolarity(0); ae->setStamp(0);
                subq.push_back(ae);
            }
        }
    }

    mutex.post();

    return subq;
}



}
