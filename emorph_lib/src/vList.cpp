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


#include "iCub/emorph/vList.h"
#include <algorithm>

namespace emorph
{

/******************************************************************************/
//VLIST
/******************************************************************************/

void vList::destroyall()
{
    for(vList::const_iterator qi = this->begin(); qi != this->end(); qi++)
        if((*qi)->destroy())
            delete *qi;
}

void vList::referall()
{
    for(vList::const_iterator qi = this->begin(); qi != this->end(); qi++)
        (*qi)->referto();
}

vList::~vList()
{
    this->clear();
}

void vList::clear()
{
    destroyall();
    deque::clear();
}

void vList::push_back(const value_type &__x)
{
    __x->referto();
    deque::push_back(__x);
}

void vList::push_front(const value_type &__x)
{  
    __x->referto();
    deque::push_front(__x);
}

void vList::pop_back()
{
    if(back()->destroy()) delete back();
    deque::pop_back();
}

void vList::pop_front()
{
    if(front()->destroy()) delete front();
    deque::pop_front();
}

vList::iterator vList::erase(iterator __first, iterator __last)
{
    for(iterator i = __first; i != __last; i++)
        if((*i)->destroy()) delete *i;

    return deque::erase(__first, __last);
}

vList::iterator vList::erase(iterator __position)
{
    //deallocate memory
    if((*__position)->destroy()) delete *__position;

    //normal erase
    return deque::erase(__position);

}

vList::vList(const vList& that)
{
    deque * lp = this;
    *lp = deque(*(deque *)(&that));

    referall();
}

vList vList::operator=(const vList& that)
{
    this->clear();

    deque * lp = this;
    *lp = (deque)that;

    referall();

    return *this;
}

void vList::sort() {
    std::sort(begin(), end(), temporalSortStraight);
}

void vList::wrapSort() {
    std::sort(begin(), end(), temporalSortWrap);
}

bool vList::temporalSortStraight(const vEvent *e1, const vEvent *e2) {
    return e2->getStamp() > e1->getStamp();
}

bool vList::temporalSortWrap(const vEvent *e1, const vEvent *e2)
{

    if(std::abs(e1->getStamp() - e2->getStamp()) > vtsHelper::maxStamp()/2)
        return e1->getStamp() > e2->getStamp();
    else
        return e2->getStamp() > e1->getStamp();

}

}

