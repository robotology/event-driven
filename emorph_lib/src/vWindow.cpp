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

vWindow::vWindow(int width, int height, int duration)
{
    this->width = width;
    this->height = height;
    this->duration = duration;
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

vWindow vWindow::operator=(const vWindow& that)
{
    this->width = that.width;
    this->height = that.height;
    this->duration = that.duration;
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

void vWindow::addEvent(vEvent &event)
{

    //calculate event window boundaries based on latest timestamp
    int ctime = event.getStamp();
    int upper = ctime + vtsHelper::maxStamp() - duration;
    int lower = ctime - duration;


    //enter critcal section
    mutex.wait();

    q.push_back(&event);
    AddressEvent *c = event.getAs<AddressEvent>();
    if(c) spatial[c->getY()][c->getX()].push_back(q.back());

    //remove any events falling out the back of the window
    while(true) {

        int vtime = q.front()->getStamp();
        if((vtime > ctime && vtime < upper) || vtime < lower) {
            AddressEvent * v = q.front()->getAs<AddressEvent>();
            if(v) spatial[v->getY()][v->getX()].pop_front();
            q.pop_front();
        } else {
            break;
        }
    }
    mutex.post();

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



}
