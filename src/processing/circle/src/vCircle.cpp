/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "vCircle.h"

void vCircle::localCircleEstimate(emorph::AddressEvent &event)
{
    int x = event.getX();
    int y = event.getY();
    //update the activity here
    activity.addEvent(event);
    return;

    //search for the best activity around the event
    double ba = -1; int bv, bu;
    for(int v = std::max(y - sRadius, 0); v = std::min(y + sRadius, height-1); v++) {
        for(int u = std::max(x - sRadius, 0); u = std::min(x + sRadius, width-1); u++) {
            double a = activity.queryActivity(u, v);
            if(a > ba) {
                ba = a;
                bv = v;
                bu = u;
            }
        }
    }



}

