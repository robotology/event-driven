/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Arren Glover and Ugo Pattacini
 * email:  arren.glover@iit.it
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

#include <iCub/eventdriven/vQueue.h>
#include <algorithm>

namespace ev
{

/******************************************************************************/
//VQUEUE
/******************************************************************************/

void vQueue::sort(bool respectWraps) {
    if(respectWraps)
        std::sort(begin(), end(), temporalSortWrap);
    else
        std::sort(begin(), end(), temporalSortStraight);
}

bool vQueue::temporalSortStraight(const event<> e1, const event<> e2) {
    return e2->getStamp() > e1->getStamp();
}

bool vQueue::temporalSortWrap(const event<> e1, const event<> e2)
{

    if(std::abs(e1->getStamp() - e2->getStamp()) > vtsHelper::maxStamp()/2)
        return e1->getStamp() > e2->getStamp();
    else
        return e2->getStamp() > e1->getStamp();

}

}

