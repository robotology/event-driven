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

int vWindow::getCurrentWindow(vQueue &sample_q)
{
    if(q.empty())
        return 0;

    //critical section
    mutex.wait();

    //do a copy and a clean
    //int lowerthesh = q.back()->getStamp() - windowSize;
    //int upperthesh = q.back()->getStamp() + windowSize;

    vQueue::iterator qi;
    for (qi = q.begin(); qi != q.end();) {
        //int etime = (*qi)->getStamp();
        //if(etime < upperthesh && etime > lowerthesh)
        //{
            //copy it into the new q
            //vEvent * newcopy = emorph::createEvent((*qi)->getType());
            //*newcopy = **qi;
            //sample_q.push_back((newcopy));
            sample_q.push_back((*qi)->clone());

            //on to the next event
            qi++;
        //}
        //else
        //{
            //clean it (set qi to next event)
         //   delete (*qi);
         //   qi = q.erase(qi);
        //}
    }

    mutex.post();

    return q.size();

}

void vWindow::addEvent(vEvent &event)
{
    //make a copy of the event to add to the circular buffer
    //vEvent * newcopy = emorph::createEvent(event.getType());
    //*newcopy = event;
    vEvent * newcopy = event.clone();

    mutex.wait();

    //add the event
    q.push_back(newcopy);

    //check if any events need to be removed
    //int lifeThreshold = event.getStamp() - windowSize;
    int ctime = event.getStamp();
    int upper = ctime + vtsHelper::maxStamp() - windowSize;
    int lower = ctime - windowSize;
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
