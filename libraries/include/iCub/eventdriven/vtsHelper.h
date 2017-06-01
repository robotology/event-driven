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

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/all.h>

namespace ev {

/// \brief helper class to deal with timestamp conversion and wrapping
class vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;

public:

    static long int max_stamp;
    static double tsscaler;
    static double vtsscaler;

    vtsHelper(): last_stamp(0), n_wraps(0) {}

    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return currentTime();
    }

    static long int maxStamp() { return max_stamp; }
    static double tstosecs() { return tsscaler; }
    unsigned long int currentTime() { return (unsigned long int)last_stamp + (max_stamp*n_wraps); }


};

/// \brief an efficient structure for storing sensor resolution
struct resolution {
    unsigned int width:10;
    unsigned int height:10;
};

}

#endif //__VTSHELPER

//----- end-of-file --- ( next line intentionally left blank ) ------------------
