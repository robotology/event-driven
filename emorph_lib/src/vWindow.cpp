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

#include "iCub/emorph/vWindow.h"

namespace emorph {

vSurface2::vSurface2(int width, int height)
{
    this->width = width;
    this->height = height;
    this->count = 0;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width);
    }
}

vQueue vSurface2::addEvent(vEvent &event)
{

    vQueue removed = removeEvents(event);

    q.push_back(&event);
    AddressEvent *c = event.getAs<AddressEvent>();
    if(c) {
        if(spatial[c->getY()][c->getX()])
            removed.push_back(spatial[c->getY()][c->getX()]);
        else
            count++;

        spatial[c->getY()][c->getX()] = c;
    }

    return removed;

}

vQueue vSurface2::getSurf()
{
    return getSurf(0, width, 0, height);
}

vQueue vSurface2::getSurf(int d)
{
    AddressEvent *v = 0;
    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        v = (*qi)->getAs<AddressEvent>();
        if(v) break;
    }
    if(!v) return vQueue();

    return getSurf(v->getX(), v->getY(), d);

}

vQueue vSurface2::getSurf(int x, int y, int d)
{
    return getSurf(x - d, x + d, y - d, y + d);
}

vQueue vSurface2::getSurf(int xl, int xh, int yl, int yh)
{
    vQueue qcopy;

    xl = std::max(xl, 0);
    xh = std::min(xh, width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, height-1);

    for(int y = yl; y <= yh; y++)
        for(int x = xl; x <= xh; x++)
            if(spatial[y][x]) qcopy.push_back(spatial[y][x]);

    return qcopy;

}

vEvent *vSurface2::getMostRecent()
{
    if(!q.size()) return NULL;
    return q.back();
}

/******************************************************************************/
vQueue temporalSurface::removeEvents(vEvent &toAdd)
{
    vQueue removed;

    //calculate event window boundaries based on latest timestamp
    int ctime = toAdd.getStamp();
    int upper = ctime + vtsHelper::maxStamp() - duration;
    int lower = ctime - duration;

    //remove any events falling out the back of the window
    while(q.size()) {

        AddressEvent * v = q.front()->getAs<AddressEvent>();
        if(v && v != spatial[v->getY()][v->getX()]) {
            q.pop_front();
            continue;
        }

        int vtime = q.front()->getStamp();

        if((vtime > ctime && vtime < upper) || vtime < lower) {
            removed.push_back(q.front());
            if(v) spatial[v->getY()][v->getX()] = NULL;
            q.pop_front();
            count--;
        } else {
            break;
        }
    }

    while(q.size()) {

        AddressEvent * v = q.back()->getAs<AddressEvent>();
        if(v && v != spatial[v->getY()][v->getX()]) {
            q.pop_back();
            continue;
        }

        int vtime = q.back()->getStamp();

        if((vtime > ctime && vtime < upper) || vtime < lower) {
            removed.push_back(q.back());
            if(v) spatial[v->getY()][v->getX()] = NULL;
            q.pop_back();
            count--;
        } else {
            break;
        }
    }

    return removed;
}

/******************************************************************************/
vQueue fixedSurface::removeEvents(vEvent &toAdd)
{

    vQueue removed;

    while(q.size()) {

        AddressEvent * v = q.front()->getAs<AddressEvent>();
        if(v && v != spatial[v->getY()][v->getX()]) {
            q.pop_front();
        } else {
            break;
        }
    }

    if(count > qlength) {

        removed.push_back(q.front());
        AddressEvent * v = q.front()->getAs<AddressEvent>();
        if(v) spatial[v->getY()][v->getX()] = NULL;
        q.pop_front();
        count--;
    }

    return removed;

}

/******************************************************************************/
vQueue lifetimeSurface::addEvent(emorph::vEvent &event)
{

    FlowEvent *v = event.getAs<FlowEvent>();
    if(!v) return vQueue();
    return vSurface2::addEvent(event);
}

vQueue lifetimeSurface::removeEvents(vEvent &toAdd)
{

    vQueue removed;
    //this could be better if q is sorted by death.


    //lifetime requires a flow event only
    FlowEvent *toAddflow = toAdd.getAs<FlowEvent>();
    if(!toAddflow)
        return vQueue();

    int cts = toAddflow->getStamp();
    int cx = toAddflow->getX(); int cy = toAddflow->getY();


    vQueue::iterator i = q.begin();
    while(i != q.end()) {
        FlowEvent *v = (*i)->getUnsafe<emorph::FlowEvent>();
        int modts = cts;
        if(cts < v->getStamp()) //we have wrapped
            modts += emorph::vtsHelper::maxStamp();

        bool samelocation = v->getX() == cx && v->getY() == cy;

        if(modts > v->getDeath() || samelocation) {
            //it could be dangerous if spatial gets more than 1 event per pixel
            removed.push_back(*i);
            spatial[v->getY()][v->getX()] = NULL;
            i = q.erase(i);
            count--;
        } else {
            i++;
        }
    }

    return removed;

}



}
