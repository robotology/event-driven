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

void activityMat::decayActivity(int x, int y, double ts)
{
    activity(y, x) = activity(y, x) * exp(-(ts - timestamps(y, x))/decayrate);
    timestamps(y, x) = ts;
}

double activityMat::addEvent(emorph::AddressEvent &event)
{
    int x = event.getX();
    int y = event.getY();

    decayActivity(x, y, unwrap(event.getStamp()));
    activity(y, x) += injectionamount;
    return activity(y, x);

}

double activityMat::queryActivity(emorph::AddressEvent &event)
{
    int x = event.getX();
    int y = event.getY();

    decayActivity(x, y, unwrap(event.getStamp()));
    return activity(y, x);

}


}
