/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

/// \defgroup emorphLib emorphLib
/// \defgroup vQueue vQueue
/// \ingroup emorphLib
/// \brief
/// A wrapper for a dequeue of event pointers with functionality for memory
/// management of events


#ifndef __EMORPH_VQUEUE__
#define __EMORPH_VQUEUE__

#include <iCub/emorph/vtsHelper.h>
#include <iCub/emorph/vCodec.h>
#include <deque>

namespace emorph {


class vQueue : public std::deque<vEvent*>
{
private:

    //! sorting events by timestamp comparitor
    static bool temporalSortWrap(const vEvent *e1, const vEvent *e2);
    static bool temporalSortStraight(const vEvent *e1, const vEvent *e2);

public:

    vQueue() {}
    ~vQueue();

    virtual void clear();

    vQueue(const vQueue&);
    vQueue& operator=(const vQueue&);

    virtual void push_back(const value_type &__x);
    virtual void push_front(const value_type &__x);

    virtual void pop_back();
    virtual void pop_front();

    virtual iterator erase(iterator __first, iterator __last);
    virtual iterator erase(iterator __position);

    void sort(bool respectWraps = false);

};

}

#endif
