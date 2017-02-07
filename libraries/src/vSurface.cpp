/*
 * Copyright (C) 2010 iCub Facility
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

#include <iCub/eventdriven/vSurface.h>
#include <math.h>

namespace ev {

vSurface::vSurface(int width, int height)
{
    this->width = width;
    this->height = height;
    this->mostRecent = event<AddressEvent>(nullptr);
    this->justRemoved = event<>(nullptr);
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
    //if(this->justRemoved) justRemoved->referto();
    //this->mostRecent->referto();

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width, 0);
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            spatial[i][j] = that.spatial[i][j];
        }
    }

    return *this;
}

void vSurface::clear()
{
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            spatial[i][j] = 0;//event<>(nullptr);
        }
    }

    subq.clear();

    mostRecent = event<AddressEvent>(nullptr);
    justRemoved = event<>(nullptr);
    eventCount = 0;

}

event<> vSurface::addEvent(event<AddressEvent> v)
{

    //enter critical section
    mutex.wait();

    //if we previously had an event move it to the justRemoved
    int x = v->getX(); int y = v->getY();
    if(spatial[y][x]) {
        justRemoved = spatial[y][x];
        eventCount--;
    }

    //put our new event in and add a reference
    spatial[y][x] = v;
    eventCount++;

    //then set our mostRecent
    mostRecent = v;

    //leave section
    mutex.post();

    return justRemoved;
}

const vQueue& vSurface::getSurf(int d)
{
    if(!mostRecent) return subq;
    //return getSurf(mostRecent->getUnsafe<AddressEvent>()->getX(),
    //               mostRecent->getUnsafe<AddressEvent>()->getY(), d);
    return getSurf(mostRecent->getX(), mostRecent->getY(), d);
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


event<> vSurface::getMostRecent()
{
    return mostRecent;
}

bool vEdge::flowremove(vQueue &removed, event<FlowEvent> vf)
{

    int x = vf->getX(); int y = vf->getY();
    double vx = vf->getVy(); double vy = vf->getVx();
    double vmag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    vx = vx / vmag; vy = vy / vmag;
    int px, py;

    //remove event at this location
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x] = event<>(nullptr);
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
                        spatial[py][px] = event<>(nullptr);
                    }
                }
            }
        }
    }

    return true;

}

bool vEdge::addressremove(vQueue &removed, event<AddressEvent> v)
{

    int x = v->getX(); int y = v->getY();
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x] = event<>(nullptr);
    }


    for(int yi = -1 + y; yi <= 1 + y; yi++) {
        for(int xi = -1 + x; xi <= 1 + x; xi++) {

            if(yi < 0 || xi < 0 || yi >= height || xi >= width) continue;
            if(spatial[yi][xi] && pepperCheck(yi, xi)) {
                removed.push_back(spatial[yi][xi]);
                spatial[yi][xi] = event<>(nullptr);;
            }
        }
    }

    return !pepperCheck(y, x);


}

bool vEdge::pepperCheck(int y, int x)
{
    if(y == 0 || y == height-1 || x == 0 || x == width -1)
        return true;

    if(spatial[y-1][x] && getas<FlowEvent>(spatial[y-1][x]))
        return false;
    if(spatial[y+1][x] && getas<FlowEvent>(spatial[y+1][x]))
        return false;
    if(spatial[y][x-1] && getas<FlowEvent>(spatial[y][x-1]))
        return false;
    if(spatial[y][x+1] && getas<FlowEvent>(spatial[y][x+1]))
        return false;
    if(spatial[y+1][x+1] && getas<FlowEvent>(spatial[y+1][x+1]))
        return false;
    if(spatial[y-1][x+1] && getas<FlowEvent>(spatial[y-1][x+1]))
        return false;
    if(spatial[y+1][x-1] && getas<FlowEvent>(spatial[y+1][x-1]))
        return false;
    if(spatial[y-1][x-1] && getas<FlowEvent>(spatial[y-1][x-1]))
        return false;

    return true;
}



vQueue vEdge::addEventToEdge(event<AddressEvent> v)
{
    vQueue removed;
    bool add = false;

    if(!v) return removed;

    //enter critical section
    mutex.wait();

    //remove non edge events
    event<FlowEvent> vf = getas<FlowEvent>(v);
    if(!vf) {
        addressremove(removed, v);
    } else {
        add = flowremove(removed, vf);
    }


    if(trackCount) {
        if(add) eventCount++;
        eventCount -= removed.size();
    }

    if(add) {
        //put our new event in and add a reference
        spatial[v->getY()][v->getX()] = v;

        //then set our mostRecent
        mostRecent = v;
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
vQueue vFuzzyEdge::addEventToEdge(event<AddressEvent> v)
{

    if(!v) return vQueue();
    event<FlowEvent> vf = getas<FlowEvent>(v);
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
                event<AddressEvent> ae = event<AddressEvent>(new AddressEvent());
                ae->setX(x); ae->setY(y); ae->setChannel(0); ae->setPolarity(0); ae->setStamp(0);
                subq.push_back(ae);
            }
        }
    }

    mutex.post();

    return subq;
}



}
