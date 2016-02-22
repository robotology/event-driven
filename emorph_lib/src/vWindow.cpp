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

vWindow::vWindow(int width, int height)
{
    this->width = width;
    this->height = height;

    this->mostrecent = NULL;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width);
    }
}

vWindow::vWindow(const vWindow& that)
{
    *this = that;
}

vWindow& vWindow::operator=(const vWindow& that)
{
    this->width = that.width;
    this->height = that.height;
    if(this->mostrecent) mostrecent->destroy();
    mostrecent = NULL;

    //we don't copy data in a vWindow for now
    q.clear();

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width);
        for(int x = 0; x < width; x++) {
            spatial[y][x].clear();
        }
    }


    return *this;
}

vQueue vWindow::addEvent(vEvent &event)
{

    vQueue removed = removeEvents(event);

    //add event in critical section
    mutex.wait();

    q.push_back(&event);
    AddressEvent *c = event.getAs<AddressEvent>();
    if(c) spatial[c->getY()][c->getX()].push_back(q.back());

    mutex.post();

    return removed;

}

void vWindow::copyTWTO(vQueue &that)
{
    mutex.wait();
    that = q;
    mutex.post();
}

const vQueue& vWindow::getTW()
{
    mutex.wait(); 
    subq = q;
    mutex.post();

    return subq;

}

const vQueue& vWindow::getSMARTSTW(int d)
{
    AddressEvent *v = 0;
    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        v = (*qi)->getAs<AddressEvent>();
        if(v) break;
    }
    if(!v) {
        subq.clear();
        return subq;
    }
    return getSTW(v->getX(), v->getY(), d);

}

const vQueue& vWindow::getSTW(int x, int y, int d)
{
    return getSTW(x - d, x + d, y - d, y + d);
}

const vQueue& vWindow::getSTW(int xl, int xh, int yl, int yh)
{
    subq.clear();

    xl = std::max(xl, 0);
    xh = std::min(xh, width);
    yl = std::max(yl, 0);
    yh = std::min(yh, height);

    //critical section
    mutex.wait();

    for(int y = yl; y < yh; y++) {
        for(int x = xl; x < xh; x++) {
            for (vQueue::iterator qi = spatial[y][x].begin();
                 qi != spatial[y][x].end();
                 qi++)
            {
                subq.push_back(*qi);
            }
        }
    }


    mutex.post();

    return subq;

}

vEvent *vWindow::getMostRecent()
{
    //return nothing if we are empty
    if(!q.size()) return 0;

    //if we had a previous most recent event allow it to be deallocated
    if(mostrecent) {
        mostrecent->destroy();
        mostrecent = NULL;
    }

    //then set the most recent to be the back of the q and make sure it has
    //a reference so it doesn't get deallocted
    mostrecent = q.back();
    mostrecent->referto();
    return mostrecent;

}

vQueue temporalWindow::removeEvents(vEvent &toAdd)
{
    vQueue removed;
    //calculate event window boundaries based on latest timestamp
    int ctime = toAdd.getStamp();
    int upper = ctime + vtsHelper::maxStamp() - duration;
    int lower = ctime - duration;


    //enter critcal section
    mutex.wait();

    //remove any events falling out the back of the window
    while(q.size()) {

        int vtime = q.front()->getStamp();
        if((vtime > ctime && vtime < upper) || vtime < lower) {
            removed.push_back(q.front());
            AddressEvent * v = q.front()->getAs<AddressEvent>();
            if(v) spatial[v->getY()][v->getX()].pop_front();
            q.pop_front();
        } else {
            break;
        }
    }

    mutex.post();

    return removed;


}

vQueue fixedWindow::removeEvents(vEvent &toAdd)
{

    vQueue removed;

    //if address event use the spatial values for event removal
    AddressEvent * toAddae = toAdd.getAs<emorph::AddressEvent>();
    int x, y;
    if(toAddae) {
        x = toAddae->getX();
        y = toAddae->getY();
    }

    //enter critcal section
    mutex.wait();

    AddressEvent * v;
    if(toAddae && spatial[y][x].size()) {
        //search for specific event to remove
        v = spatial[y][x].front()->getAs<emorph::AddressEvent>();
        for(vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
            if(v == *qi) {
                //delete this event
                removed.push_back(*qi);
                spatial[y][x].pop_front();
                q.erase(qi);
                break;
            }
        }
    }

    while(q.size() > qlength) {

        removed.push_back(q.front());
        v = q.front()->getAs<AddressEvent>();
        if(v) spatial[v->getY()][v->getX()].pop_front();
        q.pop_front();
    }


    mutex.post();
    return removed;

}

vQueue lifetimeWindow::addEvent(emorph::vEvent &event)
{

    FlowEvent *v = event.getAs<FlowEvent>();
    if(!v) return vQueue();

    vQueue removed = removeEvents(event);

    //add event in critical section
    mutex.wait();

    q.push_back(&event);
    AddressEvent *c = event.getAs<AddressEvent>();
    if(c) spatial[c->getY()][c->getX()].push_back(q.back());

    mutex.post();

    return removed;
}

vQueue lifetimeWindow::removeEvents(vEvent &toAdd)
{

    vQueue removed;
    //this could be better if q is sorted by death.


    //lifetime requires a flow event only
    FlowEvent *toAddflow = toAdd.getAs<FlowEvent>();
    if(!toAddflow)
        return vQueue();

    int cts = toAddflow->getStamp();
    int cx = toAddflow->getX(); int cy = toAddflow->getY();

    mutex.wait();

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
            spatial[v->getY()][v->getX()].pop_front();
            i = q.erase(i);
        } else {
            i++;
        }
    }

    mutex.post();
    return removed;

}



}
