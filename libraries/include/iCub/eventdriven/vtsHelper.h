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

/// \defgroup Library Library
/// \defgroup vtsHelper vtsHelper
/// \ingroup Library
/// \brief
/// a helper class for unwrapping timestamps into long ints when overflow occurs

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/all.h>

namespace ev {

class vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;

public:

#ifdef LARGETS
    static const long int max_stamp = 2147483647; //2^31
#else
    static const long int max_stamp = 16777215; //2^24
#endif

    vtsHelper(): last_stamp(0), n_wraps(0) {}

    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return currentTime();
    }

    static long int maxStamp() { return max_stamp; }
    static double tstosecs() { return 0.000001; }
    unsigned long int currentTime() { return (unsigned long int)last_stamp + (max_stamp*n_wraps); }


};

struct resolution {
    unsigned int width:10;
    unsigned int height:10;
};

}

#endif //__VTSHELPER

//----- end-of-file --- ( next line intentionally left blank ) ------------------
