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

#include "iCub/emorph/vQueue.h"
#include <algorithm>

namespace emorph
{

/******************************************************************************/
//VQUEUE
/******************************************************************************/
vQueue::~vQueue()
{
    this->clear();
}

void vQueue::clear()
{
    for(vQueue::iterator qi = this->begin(); qi != this->end(); qi++)
        (*qi)->destroy();

    deque::clear();
}

void vQueue::push_back(const value_type &__x)
{
    __x->referto();
    deque::push_back(__x);
}

void vQueue::push_front(const value_type &__x)
{
    __x->referto();
    deque::push_front(__x);
}

void vQueue::pop_back()
{
    back()->destroy();
    deque::pop_back();
}

void vQueue::pop_front()
{
    front()->destroy();
    deque::pop_front();
}

vQueue::iterator vQueue::erase(iterator __first, iterator __last)
{

    for(iterator i = __first; i != __last; i++)
        (*i)->destroy();

    return deque::erase(__first, __last);
}

vQueue::iterator vQueue::erase(iterator __position)
{
    (*__position)->destroy();
    return deque::erase(__position);
}

vQueue::vQueue(const vQueue& that)
{
    *this = that;
}

vQueue& vQueue::operator=(const vQueue& that)
{
    for(vQueue::const_iterator qi = that.begin(); qi != that.end(); qi++)
        (*qi)->referto();

    for(vQueue::iterator qi = this->begin(); qi != this->end(); qi++)
        (*qi)->destroy();

    deque::operator =(that);

    return *this;
}

void vQueue::sort(bool respectWraps) {
    if(respectWraps)
        std::sort(begin(), end(), temporalSortWrap);
    else
        std::sort(begin(), end(), temporalSortStraight);
}

bool vQueue::temporalSortStraight(const vEvent *e1, const vEvent *e2) {
    return e2->getStamp() > e1->getStamp();
}

bool vQueue::temporalSortWrap(const vEvent *e1, const vEvent *e2)
{

    if(std::abs(e1->getStamp() - e2->getStamp()) > vtsHelper::maxStamp()/2)
        return e1->getStamp() > e2->getStamp();
    else
        return e2->getStamp() > e1->getStamp();

}

}

