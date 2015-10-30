// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini, edited by Arren Glover(10/14)
 * email:  ugo.pattacini@iit.it
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

void vQueue::destroyall()
{
    for(vQueue::const_iterator qi = this->begin(); qi != this->end(); qi++)
        (*qi)->destroy();
}

void vQueue::referall()
{
    for(vQueue::const_iterator qi = this->begin(); qi != this->end(); qi++)
        (*qi)->referto();
}

vQueue::~vQueue()
{
    this->clear();
}

void vQueue::clear()
{
    destroyall();
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
    //deallocate memory
    (*__position)->destroy();

    //normal erase
    return deque::erase(__position);

}

vQueue::vQueue(const vQueue& that)
{
    *this = that;
}

vQueue& vQueue::operator=(const vQueue& that)
{
    this->clear();

    deque<vEvent *> * lp = static_cast<deque<vEvent *> *>(this);
   *lp = static_cast<const deque<vEvent *> &>(that);

    referall();

    return *this;
}

void vQueue::sort() {
    std::sort(begin(), end(), temporalSortStraight);
}

void vQueue::wrapSort() {
    std::sort(begin(), end(), temporalSortWrap);
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

