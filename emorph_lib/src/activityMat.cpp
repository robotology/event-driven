/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Arren Glover (@itt.it)
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

#include "iCub/emorph/activityMat.h"
namespace emorph {

void activityMat::decayActivity(int x, int y)
{
    activity(y, x) *= exp(-(ctime - timestamps(y, x))/decayrate);
    timestamps(y, x) = ctime;
}

double activityMat::addEvent(emorph::AddressEvent &event)
{
    int x = event.getX();
    int y = event.getY();

    if(x < 0 || x > width-1 || y < 0 || y > height-1) return -1;

    //update the current time
    ctime = unwrap(event.getStamp());

    //decay the activity (given the current time)
    //decayActivity(x, y);

    //add new activity
    for(int v = std::max(0, y-injectionradius);
        v <= std::min(height-1, y+injectionradius);
        v++) {

        for(int u = std::max(0, x-injectionradius);
            u <= std::min(width-1, x+injectionradius);
            u++) {

            decayActivity(u, v);
            activity(v, u) += injectionamount;
        }
    }

    return activity(y, x);

}

double activityMat::queryActivity(int x, int y)
{
    if(x < 0 || x > width-1 || y < 0 || y > height-1) return -1;
    decayActivity(x, y);
    return activity(y, x);

}


}
