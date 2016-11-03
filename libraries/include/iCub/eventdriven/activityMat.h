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

#ifndef __ACTIVITYMAT__
#define __ACTIVITYMAT__

#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>

namespace emorph {


class activityMat {

private:

    //timestampe helper
    emorph::vtsHelper unwrap;

    //data
    yarp::sig::Matrix activity;
    yarp::sig::Matrix timestamps;
    double ctime;

    //parameters
    double decayrate;
    double injectionamount;
    int injectionradius;
    int height;
    int width;

    //private functions
    void decayActivity(int x, int y);


public:

    activityMat(int height = 128, int width = 128, double decayRate = 1000,
                double injectionAmount = 1, int injectionRadius = 0 )
        : height(height), width(width) {

        decayrate = decayRate;
        injectionamount = injectionAmount;
        injectionradius = injectionRadius;
        ctime = 0;
        activity.resize(height, width);
        activity.zero();
        timestamps.resize(height, width);
        activity.zero();
    }

    double addEvent(emorph::AddressEvent &event);
    double queryActivity(int x, int y);
    yarp::sig::Matrix copyAllActivity() { return activity; }

};

} //namespace emorph

#endif //ACTIVITYMAT
