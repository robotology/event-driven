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

/// \defgroup Library Library
/// \defgroup vQueue vQueue
/// \ingroup Library
/// \brief
/// A wrapper for a dequeue of event pointers with functionality for memory
/// management of events


#ifndef __VQUEUE__
#define __VQUEUE__

#include <iCub/eventdriven/vtsHelper.h>
#include <iCub/eventdriven/vCodec.h>
#include <deque>
#include <memory>

namespace ev {

class vQueue : public std::deque< event<vEvent> >
{
private:

    //! sorting events by timestamp comparitor
    static bool temporalSortWrap(const event<> e1, event<> e2);
    static bool temporalSortStraight(const event<> e1, event<> e2);

public:

    vQueue() {}

    void sort(bool respectWraps = false);

};

}

#endif
