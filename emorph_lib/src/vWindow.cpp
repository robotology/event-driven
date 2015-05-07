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

const vQueue& vWindow::getWindow()
{

    //first apply limits

    vQueue *qcopy;

    if(asynchronous) {
        qcopy = new vQueue;
        mutex.wait();
        *qcopy = this->q;
        mutex.post();
    } else {
        qcopy = &q.softcopy();
    }

    return *qcopy;

}

const vQueue& vWindow::getSpatialWindow(int x, int y, int d)
{
    return getSpatialWindow(x - d, x + d, y - d, y + d);
}

const vQueue& vWindow::getSpatialWindow(int xl, int xh, int yl, int yh)
{

    //first apply limits

    vQueue *qcopy = new vQueue;
    if(!asynchronous) qcopy->setOwner(false);

    //critical section
    mutex.wait();

    vQueue::iterator qi;
    for (qi = q.begin(); qi != q.end(); qi++) {
        AddressEvent *v = (*qi)->getAs<AddressEvent>();
        if(!v) continue;
        int x = v->getX(); int y = v->getY();
        if(x < xl || x > xh || y < yl || y > yh) continue;
        if(asynchronous) {
            qcopy->push_back((*qi)->clone());
        } else {
            qcopy->push_back(*qi);
        }
    }

    mutex.post();

    return *qcopy;

}

vEvent *vWindow::getMostRecent()
{
    if(!q.size()) return 0;
    if(asynchronous) return q.back()->clone();
    else return q.back();
    return 0;
}

void vWindow::addEvent(vEvent &event)
{
    //make a copy of the event to add to the circular buffer
    //vEvent * newcopy = emorph::createEvent(event.getType());
    //*newcopy = event;
    //vEvent * newcopy = event.clone();

    int ctime = event.getStamp();
    int upper = ctime + vtsHelper::maxStamp() - windowSize;
    int lower = ctime - windowSize;


    mutex.wait();

    //add the event
    q.push_back(event.clone());

    //check if any events need to be removed
    //int lifeThreshold = event.getStamp() - windowSize;

    while(true) {

        int vtime = q.front()->getStamp();
        if((vtime > ctime && vtime < upper) || vtime < lower) {
            delete q.front();
            q.pop_front();
        }
        else
            break;
    }
    mutex.post();

}

}
