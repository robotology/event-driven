/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco (@itt.it)
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
/// \defgroup vtsHelper vtsHelper
/// \ingroup emorphLib
/// \brief
/// a helper class for unwrapping timestamps into long ints when overflow occurs

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/all.h>

namespace emorph {

class vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;
    static const long int max_stamp = 16777215; //2^24

public:

    vtsHelper(): last_stamp(0), n_wraps(0) {}

    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return (unsigned long int)timestamp + (max_stamp*n_wraps);
    }

    static long int maxStamp() { return max_stamp; }
    static double tstosecs() { return 0.000001; }

};

}

#endif //__VTSHELPER

//----- end-of-file --- ( next line intentionally left blank ) ------------------
