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

    //parameters
    double decayrate;
    double injectionamount;

    //private functions
    void decayActivity(int x, int y, double ts);


public:

    activityMat(int height = 128, int width = 128) {
        decayrate = 1000;
        injectionamount = 1;
        activity.resize(height, width);
        activity.zero();
        timestamps.resize(height, width);
        activity.zero();
    }

    double addEvent(emorph::AddressEvent &event);
    double queryActivity(emorph::AddressEvent &event);

};

} //namespace emorph

#endif //ACTIVITYMAT
